#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
polargraph.py -- turn an SVG file into movements for a 2-motor string plotter.

Pipeline (all geometry happens here on the PC, the Arduino just steps motors):

    SVG file
      -> drawable strokes (flattened polylines)
      -> scaled / centred into the drawing area, in millimetres
      -> inverse kinematics:  L = sqrt((x+a)^2 + y^2 + gap^2)
                              R = sqrt((x-a)^2 + y^2 + gap^2)
         (gap = how far the string exits sit in front of the board plane)
      -> motor step positions (relative to the HOME zero point)
      -> streamed over serial as   G01 L<steps> R<steps> F<feed> [C1]

Machine coordinate system:  origin = midpoint between the two motor
string-exit points (projected on the board), x to the right, y DOWNWARD
positive (matches SVG, no flipping needed).

CALIBRATION IS MEASUREMENT, NOT GUESSING.  The two most sensitive knobs:

  * STEPS_PER_MM_L / _R  -- set them ONLY from the ruler test in
    calibrate.py.  Do not compute them from the driver chip label, and do
    not trust a drawing that "looks right vertically": with a 25% steps/mm
    error a vertical line still comes out within ~10% while horizontal
    lines are off by 25% AND sag into an arc.
  * MOTOR_SEPARATION_MM -- measure between the points where the STRINGS
    leave the two spools, not between motor bodies or brackets.

Run   python calibrate.py   (wizard, ~10 min, ruler needed once) and it
will write all measured values into this file for you.

Typical use:
    python polargraph.py drawing.svg --simulate     # preview, no hardware
    python polargraph.py drawing.svg --port COM5    # draw for real
    python polargraph.py --check                    # self-test the math
"""

import argparse
import math
import os
import sys
import time

# =====================================================================
#  MACHINE CONFIG  --  measure your machine and edit these numbers!
#  (calibrate.py measures them and can write them here automatically)
# =====================================================================
MOTOR_SEPARATION_MM = 480.0    # distance between the two STRING EXIT points
                               # (where the thread leaves each spool)
AREA_W_MM  = 300.0             # drawing area width  (30 cm)
AREA_H_MM  = 400.0             # drawing area height (40 cm)
TOP_MARGIN_MM = 300.0           # motor line -> TOP edge of drawing area

# The motors stick OUT of the board (z direction). Only the DIFFERENCE
# gap = MOTOR_Z_MM - GONDOLA_Z_MM matters: it is the distance between the
# string exits and the string tie point on the pen holder, measured
# perpendicular to the board. calibrate.py fits this too.
MOTOR_Z_MM   = 120.0            # measured: motors jut 50 mm out of the board
GONDOLA_Z_MM = 0.0             # board surface -> string tie point on the holder

STEPS_PER_MM_L = 250.0         # <<< from calibrate.py ruler test (LEFT motor)
STEPS_PER_MM_R = 250.0         # <<< from calibrate.py ruler test (RIGHT motor)
# ^-- NEVER guess these from the driver label: A4988 vs DRV8825 and the
#     spool diameter both change them, and one motor can differ from the
#     other.  A wrong value is exactly what makes drawings oversized and
#     horizontal lines sag into arcs.  Measure it -- 5 minutes, once.

DRAW_SPEED_MM_S   = 25.0       # pen-down speed (mm/s). Lower = safer.
TRAVEL_SPEED_MM_S = 60.0       # pen-up travel speed (mm/s)
MAX_SEG_MM = 1.0               # max segment length after flattening (mm)
CORNER_STOP_DEG = 60.0         # turns sharper than this get a full stop (crisper corners)
PAD_MM     = 8.0               # keep this far from the area border

SWAP_MOTORS = False            # True if the drawing comes out mirrored
MIRROR_X    = False            # alternative fix for a mirrored drawing

BAUD = 115200
# =====================================================================

A_HALF = MOTOR_SEPARATION_MM / 2.0
Z_GAP  = MOTOR_Z_MM - GONDOLA_Z_MM     # z distance motor exit <-> gondola tie
HOME = (0.0, TOP_MARGIN_MM + AREA_H_MM / 2.0)   # centre of drawing area
PEN_DWELL_S = 0.2
ACCEL_STEPS = 25000.0     # must match ACCEL in the firmware (for time est.)


# ---------------------------------------------------------------- geometry
class Geometry:
    """Kinematics for one machine description.

    Motors' string exits sit at (-a, 0, z) and (+a, 0, z) in front of the
    board plane; the pen tip is at (x, y, 0) on the board.  String lengths
    are true 3D distances."""

    def __init__(self, a_half=A_HALF, z_gap=Z_GAP):
        self.a = float(a_half)
        self.z = float(z_gap)

    def ik(self, x, y):
        """(x, y) mm -> (left, right) string lengths in mm."""
        return (math.hypot(math.hypot(x + self.a, y), self.z),
                math.hypot(math.hypot(x - self.a, y), self.z))

    def fk(self, l_len, r_len):
        """(left, right) string lengths mm -> (x, y) mm (y downward)."""
        dl = math.sqrt(max(l_len * l_len - self.z * self.z, 0.0))
        dr = math.sqrt(max(r_len * r_len - self.z * self.z, 0.0))
        x = (dl * dl - dr * dr) / (4.0 * self.a)
        y2 = dl * dl - (x + self.a) ** 2
        return (x, math.sqrt(y2) if y2 > 0 else 0.0)


GEO = Geometry()        # default geometry, from the config block above


def ik(x, y):
    return GEO.ik(x, y)


def fk(l_len, r_len):
    return GEO.fk(l_len, r_len)


class Machine:
    """Converts machine (x, y) mm <-> motor step positions.

    Step 0/0 is wherever the pen was when G92 was issued = HOME.
    Positive steps = string wound IN.
    """
    def __init__(self, spm_l=STEPS_PER_MM_L, spm_r=STEPS_PER_MM_R,
                 a_half=None, z_gap=None,
                 swap=SWAP_MOTORS, mirror=MIRROR_X):
        self.spm_l = float(spm_l)
        self.spm_r = float(spm_r)
        self.swap = swap
        self.mirror = mirror
        self.geom = (Geometry() if a_half is None and z_gap is None
                     else Geometry(A_HALF if a_half is None else a_half,
                                   Z_GAP if z_gap is None else z_gap))
        self.L0, self.R0 = self.geom.ik(*HOME)

    def steps(self, x, y):
        if self.mirror:
            x = -x
        l, r = self.geom.ik(x, y)
        sl = int(round((l - self.L0) * self.spm_l))
        sr = int(round((r - self.R0) * self.spm_r))
        if self.swap:
            sl, sr = sr, sl
        return sl, sr

    def xy_from_steps(self, sl, sr):
        if self.swap:
            sl, sr = sr, sl
        l = self.L0 + sl / self.spm_l
        r = self.R0 + sr / self.spm_r
        return self.geom.fk(l, r)


# ---------------------------------------------------------------- SVG input
def parse_svg(path, max_seg_units=2.0):
    """Return list of strokes; each stroke is a list of complex points
    in SVG user units. Curves are flattened, gaps start new strokes."""
    try:
        from svgelements import SVG, Shape, Line as SELine
    except ImportError:
        sys.exit("Missing package. Run:  pip install svgelements")

    try:
        svg = SVG.parse(path)
    except Exception as e:
        sys.exit(f"Could not read SVG '{path}': {e}")

    def shape_segments(shape):
        # svgelements exposes .segments as a method in some versions,
        # as a property in others.
        try:
            segs = shape.segments() if callable(shape.segments) else shape.segments
            return list(segs)
        except Exception:
            return []

    strokes = []
    for e in svg.elements():
        if not isinstance(e, Shape):
            continue
        cur = None
        for seg in shape_segments(e):
            try:
                seg_len = seg.length()
            except Exception:
                seg_len = None
            if seg_len is not None and seg_len <= 1e-9:
                continue                      # Move / Close / zero length
            try:
                start = complex(seg.start)
                end = complex(seg.end)
            except Exception:
                continue
            if cur is None or abs(start - cur[-1]) > 1e-3:
                if cur and len(cur) > 1:
                    strokes.append(cur)
                cur = [start]
            if isinstance(seg, SELine):
                cur.append(end)
            else:                             # curve: sample it
                if seg_len and math.isfinite(seg_len) and seg_len > 0:
                    n = max(2, int(math.ceil(seg_len / max_seg_units)))
                else:
                    n = 36
                n = min(n, 4000)
                for i in range(1, n + 1):
                    pt = seg.point(i / n)
                    cur.append(complex(pt.real, pt.imag))
        if cur and len(cur) > 1:
            strokes.append(cur)
    return strokes


def to_machine_mm(strokes, scale=None, pad=PAD_MM):
    """Scale + translate strokes from SVG units into machine mm, fitted
    inside the drawing area (centred). Returns (strokes_mm, scale_used)."""
    pts = [p for s in strokes for p in s]
    if not pts:
        sys.exit("SVG contains no drawable points.")
    minx = min(p.real for p in pts); maxx = max(p.real for p in pts)
    miny = min(p.imag for p in pts); maxy = max(p.imag for p in pts)
    w = max(maxx - minx, 1e-9)
    h = max(maxy - miny, 1e-9)
    if scale is None:
        fx = (AREA_W_MM - 2 * pad) / w
        fy = (AREA_H_MM - 2 * pad) / h
        scale = min(fx, fy)
    ox = -(minx + maxx) / 2.0 * scale                       # centre at x=0
    oy = (TOP_MARGIN_MM + (AREA_H_MM - h * scale) / 2.0     # centre in area
          - miny * scale)
    out = [[(p.real * scale + ox, p.imag * scale + oy) for p in s]
           for s in strokes]
    return out, scale


def resample(strokes, max_seg=MAX_SEG_MM, min_seg=0.1):
    """Subdivide segments longer than max_seg (keeps pen-down lines straight
    in Cartesian space) and drop points closer than min_seg."""
    out = []
    for s in strokes:
        ns = []
        prev = None
        for pt in s:
            if prev is None:
                ns.append(pt)
                prev = pt
                continue
            d = math.dist(prev, pt)
            if d < min_seg:
                continue
            n = int(math.ceil(d / max_seg))
            for i in range(1, n + 1):
                f = i / n
                ns.append((prev[0] + (pt[0] - prev[0]) * f,
                           prev[1] + (pt[1] - prev[1]) * f))
            prev = pt
        if len(ns) > 1:
            out.append(ns)
    return out


def order_strokes(strokes, start_pt):
    """Greedy nearest-neighbour ordering (strokes may be reversed) to cut
    pen-up travel time."""
    remaining = list(strokes)
    pos = start_pt
    ordered = []
    while remaining:
        best_i, best_d, best_rev = 0, float("inf"), False
        for i, s in enumerate(remaining):
            d0 = math.dist(pos, s[0])
            d1 = math.dist(pos, s[-1])
            if d0 < best_d:
                best_i, best_d, best_rev = i, d0, False
            if d1 < best_d:
                best_i, best_d, best_rev = i, d1, True
        s = remaining.pop(best_i)
        if best_rev:
            s = s[::-1]
        ordered.append(s)
        pos = s[-1]
    return ordered


def clamp_to_area(x, y):
    x2 = min(max(x, -(AREA_W_MM / 2 - 2)), AREA_W_MM / 2 - 2)
    y2 = min(max(y, TOP_MARGIN_MM + 2), TOP_MARGIN_MM + AREA_H_MM - 2)
    return x2, y2


# ---------------------------------------------------------------- commands
def _turn_deg(a, b, c):
    """Turn angle (deg) at vertex b of polyline a -> b -> c."""
    v1x, v1y = b[0] - a[0], b[1] - a[1]
    v2x, v2y = c[0] - b[0], c[1] - b[1]
    d = (v1x * v2x + v1y * v2y) / (math.hypot(v1x, v1y) * math.hypot(v2x, v2y) + 1e-12)
    return math.degrees(math.acos(max(-1.0, min(1.0, d))))


def build_cmds(strokes, machine, draw_speed, travel_speed):
    """High-level command list. Moves are ('move', x, y, feed, cont, pen_up).
    Pen: ('pen_up',) / ('pen_down',). Feed is in steps/s of the dominant motor.
    Interior vertices are drawn "continuous" (C1) EXCEPT where the polyline
    turns more than CORNER_STOP_DEG -- those get a full stop so the pen
    actually reaches the corner point -> sharper corners."""
    spm = max(machine.spm_l, machine.spm_r)
    feed_draw = max(int(draw_speed * spm), 50)
    feed_trav = max(int(travel_speed * spm), 50)

    ordered = order_strokes(strokes, HOME)
    cmds = [("pen_up",)]
    for s in ordered:
        sx, sy = clamp_to_area(*s[0])
        cmds.append(("move", sx, sy, feed_trav, False, True))
        cmds.append(("pen_down",))
        for j in range(1, len(s)):
            x, y = clamp_to_area(*s[j])
            last = (j == len(s) - 1)
            cont = (not last) and _turn_deg(s[j - 1], s[j], s[j + 1]) < CORNER_STOP_DEG
            cmds.append(("move", x, y, feed_draw, cont, False))
        cmds.append(("pen_up",))
    hx, hy = HOME
    cmds.append(("move", hx, hy, feed_trav, False, True))    # park at home
    return cmds


def compile_cmds(cmds, machine):
    """Turn high-level commands into serial lines + info for stats/sim.
    Output tuples: ('move', line, n_steps, feed, cont, sl, sr)
                   ('pen',  line, 0, 0, False, None, None)"""
    out = []
    curL, curR = 0, 0                     # after G92
    for c in cmds:
        if c[0] == "move":
            _, x, y, feed, cont, _pen_up = c
            sl, sr = machine.steps(x, y)
            n = max(abs(sl - curL), abs(sr - curR))
            line = "G01 L%d R%d F%d%s" % (sl, sr, feed, " C1" if cont else "")
            out.append(("move", line, n, feed, cont, sl, sr))
            curL, curR = sl, sr
        elif c[0] == "pen_down":
            out.append(("pen", "M3", 0, 0, False, None, None))
        elif c[0] == "pen_up":
            out.append(("pen", "M5", 0, 0, False, None, None))
    return out


def path_lengths(compiled, machine):
    """(pen-down length mm, pen-up length mm) of the real machine path."""
    down = up = 0.0
    pen_up = True
    cur = machine.xy_from_steps(0, 0)
    for kind, _line, _n, _f, _c, sl, sr in compiled:
        if kind == "pen":
            pen_up = (_line == "M5")
            continue
        pt = machine.xy_from_steps(sl, sr)
        d = math.dist(cur, pt)
        if pen_up:
            up += d
        else:
            down += d
        cur = pt
    return down, up


def est_seconds(compiled):
    t = 0.0
    for kind, _line, n, feed, _c, *_ in compiled:
        if kind == "pen":
            t += PEN_DWELL_S + 0.01
        elif n > 0:
            v = min(float(feed), math.sqrt(ACCEL_STEPS * n))   # triangular cap
            t += n / max(v, 1.0) + v / ACCEL_STEPS * 0.5
            t += 0.004                                          # handshake
    return t


# ---------------------------------------------------------------- self test
def self_check():
    """Numeric self-test: model consistency (no hardware needed).  If this
    passes but the real machine still distorts, the CONFIG VALUES are off,
    not the code -- run calibrate.py."""
    worst = 0.0
    for x in (-140, -70, 0, 70, 140):
        for y in (50, 150, 250, 350, 430):
            l, r = ik(x, y)
            X, Y = fk(l, r)
            worst = max(worst, abs(X - x), abs(Y - y))
    m = Machine()
    # commanded straight lines must stay straight through steps and back
    bow_h = bow_v = 0.0
    for i in range(21):
        x = -125.0 + 250.0 * i / 20.0
        X, Y = m.xy_from_steps(*m.steps(x, HOME[1]))
        bow_h = max(bow_h, abs(Y - HOME[1]))
    for i in range(21):
        y = HOME[1] - 125.0 + 250.0 * i / 20.0
        X, Y = m.xy_from_steps(*m.steps(0.0, y))
        bow_v = max(bow_v, abs(X - 0.0))
    print("self-check: ik/fk roundtrip max error %.2e mm" % worst)
    print("self-check: straight-line bow through steps: h %.3f mm, v %.3f mm"
          % (bow_h, bow_v))
    ok = worst < 1e-6 and bow_h < 0.05 and bow_v < 0.05
    print("self-check:", "PASS -- the math is exact; any distortion you see"
          " on the board is a calibration issue (run calibrate.py)"
          if ok else "FAIL")
    return ok


# ---------------------------------------------------------------- preview
def simulate(compiled, machine, png_path, src_name, down_mm, up_mm):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed -> cannot save a preview "
              "(pip install matplotlib)")
        return

    fig, ax = plt.subplots(figsize=(7.5, 9.5), constrained_layout=True)
    # drawing area + machine frame
    ax.add_patch(plt.Rectangle((-AREA_W_MM / 2, TOP_MARGIN_MM), AREA_W_MM,
                                AREA_H_MM, fill=False, ls="--", ec="0.4",
                                lw=1.0))
    ax.plot([-A_HALF, A_HALF], [0, 0], "k-", lw=2, solid_capstyle="round")
    ax.plot(-A_HALF, 0, "rx", ms=9); ax.plot(A_HALF, 0, "rx", ms=9)
    ax.annotate("L motor", (-A_HALF, 0), textcoords="offset points",
                xytext=(0, 8), ha="center", color="r", fontsize=9)
    ax.annotate("R motor", (A_HALF, 0), textcoords="offset points",
                xytext=(0, 8), ha="center", color="r", fontsize=9)

    pen_up = True
    cur = machine.xy_from_steps(0, 0)
    ax.plot(*cur, "o", color="green", ms=5)
    ax.annotate("HOME", cur, textcoords="offset points", xytext=(8, -3),
                color="green", fontsize=9)
    up_x, up_y, dn_x, dn_y = [], [], [], []
    for kind, _line, _n, _f, _c, sl, sr in compiled:
        if kind == "pen":
            pen_up = (_line == "M5")
            continue
        pt = machine.xy_from_steps(sl, sr)
        (up_x if pen_up else dn_x).extend([cur[0], pt[0]])
        (up_y if pen_up else dn_y).extend([cur[1], pt[1]])
        cur = pt
    if up_x:
        ax.plot(up_x, up_y, color="0.6", lw=0.6, ls=":",
                label="travel (pen up)")
    if dn_x:
        ax.plot(dn_x, dn_y, color="#0055cc", lw=1.3, label="drawing (pen down)")
    ax.set_aspect("equal")
    ax.grid(True, lw=0.3, alpha=0.5)
    ax.set_xlabel("x (mm)")
    ax.set_ylabel("y (mm, downward)")
    ax.set_title("String plotter simulation - %s\n"
                 "draw %.0f mm | travel %.0f mm | area %.0fx%.0f mm"
                 % (src_name, down_mm, up_mm, AREA_W_MM, AREA_H_MM))
    ax.legend(loc="upper left", bbox_to_anchor=(1.02, 1.0), fontsize=8)
    fig.savefig(png_path)
    plt.close(fig)
    print("Preview saved: %s" % png_path)
    try:
        if sys.platform == "win32":
            os.startfile(png_path)          # open it for the user
    except Exception:
        pass


# ---------------------------------------------------------------- serial
def pick_port(want):
    import serial.tools.list_ports as lp
    ports = list(lp.comports())
    if want:
        return want
    if len(ports) == 1:
        return ports[0].device
    if not ports:
        sys.exit("No serial ports found. Is the Arduino plugged in?")
    print("Available serial ports:")
    for p in ports:
        print("   %-20s %s" % (p.device, p.description))
    sys.exit("Pick one with:  --port COMx")


def send_cmd(ser, line, timeout=30.0, want_id=False):
    """Send one line, wait for OK. Returns True if a STRINGPLOT id line
    was seen along the way."""
    ser.write((line + "\n").encode())
    t0 = time.time()
    saw_ok = saw_id = False
    while time.time() - t0 < timeout:
        raw = ser.readline()
        if not raw:
            continue
        ln = raw.decode(errors="ignore").strip()
        if not ln:
            continue
        if "STRINGPLOT" in ln:
            saw_id = True
        if ln == "OK":
            saw_ok = True
            break
        if ln.startswith("ERR"):
            raise RuntimeError("plotter replied: %s (after '%s')" % (ln, line))
    if not saw_ok:
        raise TimeoutError("No OK from plotter for '%s' " % line)
    return saw_id


def run_serial(compiled, port, est_s):
    import serial
    print("Connecting to %s ..." % port)
    ser = serial.Serial(port, BAUD, timeout=2)
    try:
        time.sleep(2.5)                     # UNO reboots when port opens
        ser.reset_input_buffer()
        if not send_cmd(ser, "M115", want_id=True):
            raise RuntimeError("This does not look like StringPlotter firmware.")
        print("Firmware OK. Motors enabled.")
        send_cmd(ser, "M17")
        print()
        print("SETUP:")
        print("  1. Both strings hooked to the pen holder, marker in place.")
        print("  2. Move the pen so its TIP sits exactly on the HOME dot")
        print("     (centre of the %dx%d mm area, %g mm below the motor line)."
              % (AREA_W_MM, AREA_H_MM, TOP_MARGIN_MM + AREA_H_MM / 2))
        input("  3. Press ENTER to set this as the ZERO position... ")
        send_cmd(ser, "G92")
        n = len(compiled)
        input("\n%d moves, estimated %.1f min.\n"
              "Press ENTER to START drawing (Ctrl+C aborts)... " % (n, est_s / 60))
        t0 = time.time()
        for i, (kind, line, *_rest) in enumerate(compiled):
            send_cmd(ser, line)
            if i % 20 == 0 or i == n - 1:
                print("\r  %d / %d moves  (%.0f s elapsed)"
                      % (i + 1, n, time.time() - t0), end="", flush=True)
        print("\nDONE in %.1f min. Parked at home; motors released."
              % ((time.time() - t0) / 60))
    finally:
        for l in ("M5", "M18"):
            try:
                send_cmd(ser, l, timeout=3)
            except Exception:
                pass
        try:
            ser.close()
        except Exception:
            pass
        print("(pen up + motors released)")


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(
        description="SVG -> string plotter (needs StringPlotter.ino on the Arduino)")
    ap.add_argument("svg", nargs="?", help="input SVG file (vector line art)")
    ap.add_argument("--port", help="serial port, e.g. COM5 (omit to list)")
    ap.add_argument("--simulate", nargs="?", const="AUTO",
                    help="save a preview PNG of the machine path (no hardware)")
    ap.add_argument("--dry-run", action="store_true",
                    help="parse and show stats only")
    ap.add_argument("--check", action="store_true",
                    help="run the numeric self-test and exit")
    ap.add_argument("--speed", type=float, default=DRAW_SPEED_MM_S,
                    help="pen-down speed mm/s (default %g)" % DRAW_SPEED_MM_S)
    ap.add_argument("--travel-speed", type=float, default=TRAVEL_SPEED_MM_S,
                    help="pen-up speed mm/s (default %g)" % TRAVEL_SPEED_MM_S)
    ap.add_argument("--spm", type=float,
                    help="steps/mm for BOTH motors (overrides config)")
    ap.add_argument("--spm-l", type=float, help="steps/mm LEFT motor override")
    ap.add_argument("--spm-r", type=float, help="steps/mm RIGHT motor override")
    ap.add_argument("--separation", type=float,
                    help="motor separation mm override (string exit points)")
    ap.add_argument("--z-gap", type=float,
                    help="motor exit <-> gondola tie z distance mm override")
    ap.add_argument("--scale", type=float, help="force SVG->mm scale (default: fit)")
    ap.add_argument("--pad", type=float, default=PAD_MM,
                    help="border inside the drawing area, mm (default %g)" % PAD_MM)
    args = ap.parse_args()

    if args.check:
        sys.exit(0 if self_check() else 1)

    if not args.svg:
        ap.error("the SVG file argument is required unless --check is used")
    if not os.path.isfile(args.svg):
        sys.exit("File not found: %s" % args.svg)

    print("Parsing %s ..." % args.svg)
    strokes = parse_svg(args.svg)
    if not strokes:
        sys.exit("No drawable strokes found. Note: <text> must be converted "
                 "to paths, embedded raster images are ignored.")
    machine = Machine(spm_l=(args.spm_l or args.spm or STEPS_PER_MM_L),
                      spm_r=(args.spm_r or args.spm or STEPS_PER_MM_R),
                      a_half=None if args.separation is None
                             else args.separation / 2.0,
                      z_gap=args.z_gap)

    mm_strokes, scale = to_machine_mm(strokes, scale=args.scale, pad=args.pad)
    mm_strokes = resample(mm_strokes)
    cmds = build_cmds(mm_strokes, machine, args.speed, args.travel_speed)
    compiled = compile_cmds(cmds, machine)

    down_mm, up_mm = path_lengths(compiled, machine)
    est = est_seconds(compiled)
    n_moves = sum(1 for c in compiled if c[0] == "move")
    print()
    print("Strokes: %d   moves: %d   (SVG->mm scale %.3f)"
          % (len(mm_strokes), n_moves, scale))
    print("Pen-down path: %.0f mm   pen-up travel: %.0f mm" % (down_mm, up_mm))
    print("Estimated time: ~%.1f min   speeds: %.0f / %.0f mm/s"
          % (est / 60, args.speed, args.travel_speed))
    print()

    if args.simulate:
        if args.simulate == "AUTO":
            root, _ = os.path.splitext(args.svg)
            png = root + "_preview.png"
        else:
            png = args.simulate
        simulate(compiled, machine, png, os.path.basename(args.svg),
                 down_mm, up_mm)

    if args.dry_run or not args.port:
        if not args.simulate and not args.dry_run:
            print("No --port given -> not sending anything.")
            print("Try --simulate for a preview, or --port COMx to draw.")
        return

    port = pick_port(args.port)
    run_serial(compiled, port, est)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nAborted by user.")
        sys.exit(130)
