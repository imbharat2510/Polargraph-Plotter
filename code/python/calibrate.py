#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
calibrate.py -- measurement wizard for the string plotter (v2).

The v1 lesson, learned the hard way on a real whiteboard: GUESSING steps/mm
from the driver chip label is what makes drawings oversized, vertically odd
and puts an arc into every horizontal line.  The vertical axis is a lying
witness -- a 25% steps/mm error still draws a vertical line within ~10%,
so "vertical looks right" proves nothing.  The horizontal scale and the
straightness of horizontal lines are the honest witnesses.

This wizard only trusts MEASUREMENTS, in this order:

  STEP 1  motor direction check            (jog test, as before)
  STEP 2  steps/mm per motor with a ruler  (ground truth, catches mixed
          A4988/DRV8825 drivers and different spool sizes)
  STEP 3  closed-loop CROSS calibration: draws a 250 mm cross, you measure
          the four arms and the bow of the horizontal line with a ruler,
          the script FITS your machine (steps/mm, real motor separation,
          real string z-gap) and writes the values into polargraph.py.
          Then it redraws the cross with the fitted values: you re-measure
          -- when the cross comes out 250 x 250 mm and straight, you are
          DONE, with proof on the board.
  STEP 4  summary + what to do next.

Usage:  python calibrate.py [--port COM5]
"""
import argparse
import math
import os
import re
import sys
import time

import polargraph as pg

BAUD = 115200
CROSS_HALF = 125.0          # each arm of the calibration cross, mm
TOL_MM = 1.5                # "calibrated" tolerance for the cross arms
TOL_BOW = 1.0               # ... and for the bow of the horizontal line


# ---------------------------------------------------------------- serial
def send_cmd(ser, line, timeout=30.0):
    ser.write((line + "\n").encode())
    t0 = time.time()
    while time.time() - t0 < timeout:
        raw = ser.readline()
        if not raw:
            continue
        ln = raw.decode(errors="ignore").strip()
        if not ln:
            continue
        if "STRINGPLOT" in ln:
            continue
        if ln == "OK":
            return
        if ln.startswith("ERR"):
            raise RuntimeError("plotter replied: %s" % ln)
    raise TimeoutError("No OK from plotter for '%s'" % line)


def ask_float(prompt, lo=-500.0, hi=500.0):
    while True:
        try:
            v = float(input(prompt).replace(",", "."))
            if lo <= v <= hi:
                return v
        except ValueError:
            pass
        print("  Please enter a number between %g and %g." % (lo, hi))


# ------------------------------------------------- machine simulator (fit)
# Mirrors exactly what the hardware does:
#   G92 at physical HOME  ->  strings hold their TRUE home lengths.
#   PC commands with the CONFIG model -> steps = (L_model - L_model_home)*spm_conf
#   motors execute steps   ->  physical length = true_home_len + model_delta * k
#   pen lands where the TRUE geometry satisfies those lengths.
def _ik(x, y, a, z):
    return (math.hypot(math.hypot(x + a, y), z),
            math.hypot(math.hypot(x - a, y), z))


def _fk(l_len, r_len, a, z):
    dl = math.sqrt(max(l_len * l_len - z * z, 0.0))
    dr = math.sqrt(max(r_len * r_len - z * z, 0.0))
    x = (dl * dl - dr * dr) / (4.0 * a)
    y2 = dl * dl - (x + a) ** 2
    return (x, math.sqrt(y2) if y2 > 0 else 0.0)


def drawn_point(x_cmd, y_cmd, conf, true):
    a_c, z_c, sl_c, sr_c = conf
    a_t, z_t, sl_t, sr_t = true
    Lm, Rm = _ik(x_cmd, y_cmd, a_c, z_c)
    Lm0, Rm0 = _ik(pg.HOME[0], pg.HOME[1], a_c, z_c)
    La = _ik(pg.HOME[0], pg.HOME[1], a_t, z_t)[0] + (Lm - Lm0) * (sl_c / sl_t)
    Ra = _ik(pg.HOME[0], pg.HOME[1], a_t, z_t)[1] + (Rm - Rm0) * (sr_c / sr_t)
    return _fk(La, Ra, a_t, z_t)


def cross_observables(conf, true):
    """The 5 numbers the user measures on the board after the cross is
    drawn with config `conf` on a machine that is really `true`:
    v_top, v_bot, h_left, h_right, bow_h (mm, bow + = sags down)."""
    hy = pg.HOME[1]
    cen = drawn_point(0.0, hy, conf, true)
    ends = {"v_top": drawn_point(0.0, hy - CROSS_HALF, conf, true),
            "v_bot": drawn_point(0.0, hy + CROSS_HALF, conf, true),
            "h_left": drawn_point(-CROSS_HALF, hy, conf, true),
            "h_right": drawn_point(+CROSS_HALF, hy, conf, true)}
    out = {k: math.dist(cen, p) for k, p in ends.items()}
    pts = [drawn_point(-CROSS_HALF + 2.0 * CROSS_HALF * i / 20.0, hy,
                       conf, true)
           for i in range(21)]
    (x0, y0), (x1, y1) = pts[0], pts[-1]
    worst = 0.0
    for (px, py) in pts:
        t = ((px - x0) * (x1 - x0) + (py - y0) * (y1 - y0)) / \
            max((x1 - x0) ** 2 + (y1 - y0) ** 2, 1e-12)
        d = py - (y0 + t * (y1 - y0))
        if abs(d) > abs(worst):
            worst = d
    out["bow_h"] = worst
    return out


# ------------------------------------------------------------- the fitter
def nelder_mead(f, x0, steps, bounds, iters=320):
    """Plain Nelder-Mead with bounds clamping. Returns (best_x, best_f)."""
    n = len(x0)

    def clamp(p):
        return [min(max(p[i], bounds[i][0]), bounds[i][1]) for i in range(n)]

    simp = [clamp(list(x0))]
    for i in range(n):
        p = list(x0)
        p[i] = p[i] + steps[i] if p[i] + steps[i] <= bounds[i][1] \
            else p[i] - steps[i]
        simp.append(clamp(p))
    val = [f(p) for p in simp]

    for _ in range(iters):
        order = sorted(range(n + 1), key=lambda i: val[i])
        simp = [simp[i] for i in order]
        val = [val[i] for i in order]
        cen = [sum(simp[i][j] for i in range(n)) / n for j in range(n)]
        xr = clamp([cen[j] + (cen[j] - simp[-1][j]) for j in range(n)])
        fr = f(xr)
        if fr < val[0]:
            xe = clamp([cen[j] + 2 * (cen[j] - simp[-1][j])
                        for j in range(n)])
            fe = f(xe)
            simp[-1], val[-1] = (xe, fe) if fe < fr else (xr, fr)
        elif fr < val[-2]:
            simp[-1], val[-1] = xr, fr
        else:
            xc = clamp([cen[j] + 0.5 * (simp[-1][j] - cen[j])
                        for j in range(n)])
            fc = f(xc)
            if fc < val[-1]:
                simp[-1], val[-1] = xc, fc
            else:
                for i in range(1, n + 1):
                    simp[i] = clamp([(simp[0][j] + 0.5 * (simp[i][j] - simp[0][j]))
                                      for j in range(n)])
                    val[i] = f(simp[i])
        if val[0] < 1e-9:
            break
    best = min(range(n + 1), key=lambda i: val[i])
    return simp[best], val[best]


def fit_machine(conf, measured):
    """Given what the user measured, find the TRUE machine parameters.

    conf  = (a, z, spm_l, spm_r) that the cross was DRAWN with.
    measured = dict(v_top, v_bot, h_left, h_right, bow_h) in mm.
    Returns (a_t, z_t, spm_l_t, spm_r_t, residual_mm)."""
    keys = ("v_top", "v_bot", "h_left", "h_right", "bow_h")

    def residual(true):
        pred = cross_observables(conf, true)
        return sum(((pred[k] - measured[k]) / 1.0) ** 2 for k in keys)

    bounds = [(150.0, 350.0), (0.0, 120.0), (20.0, 600.0), (20.0, 600.0)]
    a_c, z_c, sl_c, sr_c = conf
    best = None

    # stage A: fit steps/mm + separation, keep z at the configured value
    for s in (0.70, 0.85, 1.0, 1.15, 1.30):
        for sa in (0.92, 1.0, 1.08):
            x0 = [a_c * sa, z_c, min(max(sl_c * s, 20.0), 600.0),
                  min(max(sr_c * s, 20.0), 600.0)]
            p, v = nelder_mead(residual, x0, [8.0, 0.0, sl_c * 0.08 + 1.0,
                                               sr_c * 0.08 + 1.0], bounds)
            if best is None or v < best[1]:
                best = (p, v)
    # stage B: also let z move, starting from stage A's best
    p0 = best[0]
    p, v = nelder_mead(residual, p0, [6.0, 12.0, p0[2] * 0.05 + 1.0,
                                       p0[3] * 0.05 + 1.0], bounds, iters=420)
    if v < best[1]:
        best = (p, v)
    a_t, z_t, sl_t, sr_t = best[0]
    return a_t, z_t, sl_t, sr_t, math.sqrt(best[1])


# ------------------------------------------------- config file auto-apply
CONF_PATTERNS = {
    "MOTOR_SEPARATION_MM": r"^(\s*MOTOR_SEPARATION_MM\s*=\s*)[-\d.]+",
    "MOTOR_Z_MM": r"^(\s*MOTOR_Z_MM\s*=\s*)[-\d.]+",
    "STEPS_PER_MM_L": r"^(\s*STEPS_PER_MM_L\s*=\s*)[-\d.]+",
    "STEPS_PER_MM_R": r"^(\s*STEPS_PER_MM_R\s*=\s*)[-\d.]+",
}


def read_config_value(path, name):
    with open(path, "r", encoding="utf-8") as fh:
        m = re.search(r"^%s\s*=\s*([-\d.]+)" % name, fh.read(), re.M)
    return float(m.group(1)) if m else None


def write_config_values(path, values):
    """values: {'STEPS_PER_MM_L': 160.0, ...} -> rewrite those lines in
    polargraph.py (a .bak copy is kept)."""
    with open(path, "r", encoding="utf-8") as fh:
        txt = fh.read()
    backup = path + ".bak"
    with open(backup, "w", encoding="utf-8") as fh:
        fh.write(txt)
    for name, val in values.items():
        txt, n = re.subn(CONF_PATTERNS[name], r"\g<1>%s" % val, txt,
                         count=1, flags=re.M)
        if n != 1:
            print("  WARNING: could not find '%s = ...' in polargraph.py --"
                  " set it manually to %s" % (name, val))
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(txt)


def apply_and_update(pg_path, conf, new):
    """Write the fitted values into polargraph.py and return the new conf
    tuple (a, z, spm_l, spm_r) for in-memory use."""
    a_t, z_t, sl_t, sr_t = new
    gondola = read_config_value(pg_path, "GONDOLA_Z_MM")
    if gondola is None:
        gondola = 0.0
    values = {"STEPS_PER_MM_L": round(sl_t, 1),
              "STEPS_PER_MM_R": round(sr_t, 1),
              "MOTOR_SEPARATION_MM": round(2 * a_t, 0),
              "MOTOR_Z_MM": round(z_t + gondola, 1)}
    ans = input("  Write these into polargraph.py? [Y/n] ").strip().lower()
    if ans in ("n", "no"):
        print("  OK -- set them by hand, then re-run calibrate.py.")
    else:
        write_config_values(pg_path, values)
        print("  polargraph.py updated (backup saved as polargraph.py.bak).")
    return (a_t, z_t, sl_t, sr_t)


# ---------------------------------------------------------------- drawing
def draw_cross(ser, conf):
    """Draw the calibration cross with the given config values."""
    a_c, z_c, sl_c, sr_c = conf
    m = pg.Machine(spm_l=sl_c, spm_r=sr_c, a_half=a_c, z_gap=z_c)
    feed = max(int(15.0 * max(sl_c, sr_c)), 50)     # slow = clean lines
    hy = pg.HOME[1]
    cmds = [
        ("move", 0.0, hy - CROSS_HALF, feed, False, True),   # top arm
        ("pen_down",),
        ("move", 0.0, hy + CROSS_HALF, feed, False, False),  # vertical
        ("pen_up",),
        ("move", -CROSS_HALF, hy, feed, False, True),        # stray diagonal
        ("pen_down",),
        ("move", +CROSS_HALF, hy, feed, False, False),      # horizontal
        ("pen_up",),
        ("move", 0.0, hy, feed, False, True),                # back home
    ]
    compiled = pg.compile_cmds(cmds, m)
    send_cmd(ser, "G92")
    for kind, line, *_rest in compiled:
        send_cmd(ser, line)


def measure_cross():
    print("  With a ruler ON the whiteboard, measure (the marker never")
    print("  lifts yet, so ignore the thin diagonal stray line):")
    v_top = ask_float("  1. crossing point -> TOP end of vertical line [mm]: ",
                      10, 400)
    v_bot = ask_float("  2. crossing point -> BOTTOM end of vertical line [mm]: ",
                      10, 400)
    h_left = ask_float("  3. crossing point -> LEFT end of horizontal line [mm]: ",
                       10, 400)
    h_right = ask_float("  4. crossing point -> RIGHT end of horizontal line [mm]: ",
                        10, 400)
    print("  5. Hold the ruler so its edge touches BOTH ends of the")
    print("     horizontal line at once. Look at the MIDDLE of the line:")
    while True:
        s = input("     is it BELOW the ruler edge, ABOVE it, or FLAT? "
                  "[below/above/flat]: ").strip().lower()
        if s.startswith("b") or s.startswith("a") or s.startswith("f"):
            break
        print("     please answer below / above / flat")
    if s.startswith("f"):
        bow = 0.0
    else:
        mm = ask_float("     by how many mm? ", 0, 40)
        bow = mm if s.startswith("b") else -mm
    return dict(v_top=v_top, v_bot=v_bot, h_left=h_left, h_right=h_right,
                bow_h=bow)


def report_cross(o):
    errs = {k: o[k] - CROSS_HALF for k in ("v_top", "v_bot", "h_left",
                                           "h_right")}
    ok = (max(abs(e) for e in errs.values()) <= TOL_MM
          and abs(o["bow_h"]) <= TOL_BOW)
    print("  measured: top %.0f  bottom %.0f  left %.0f  right %.0f mm"
          " (target %.0f), bow %+.1f mm (target 0)"
          % (o["v_top"], o["v_bot"], o["h_left"], o["h_right"],
             CROSS_HALF, o["bow_h"]))
    return ok


# ---------------------------------------------------------------- wizard
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="serial port, e.g. COM5")
    ap.add_argument("--skip-steps", action="store_true",
                    help="skip the ruler steps/mm test (not recommended)")
    args = ap.parse_args()

    try:
        import serial
        import serial.tools.list_ports as lp
    except ImportError:
        sys.exit("Missing package. Run:  pip install pyserial")

    here = os.path.dirname(os.path.abspath(__file__))
    pg_path = os.path.join(here, "polargraph.py")

    ports = list(lp.comports())
    port = args.port
    if not port:
        if len(ports) == 1:
            port = ports[0].device
        elif not ports:
            sys.exit("No serial ports found. Is the Arduino plugged in?")
        else:
            print("Available serial ports:")
            for p in ports:
                print("   %-20s %s" % (p.device, p.description))
            port = input("Which port? ").strip()
    print("Connecting to %s ..." % port)
    ser = serial.Serial(port, BAUD, timeout=2)
    try:
        time.sleep(2.5)
        ser.reset_input_buffer()
        send_cmd(ser, "M115")
        print("Firmware found. Motors enabled.\n")
        send_cmd(ser, "M17")

        # live machine state the wizard keeps in memory:
        conf = (pg.A_HALF, pg.Z_GAP, pg.STEPS_PER_MM_L, pg.STEPS_PER_MM_R)
        print("Current config in polargraph.py: separation %.0f mm, gap %.0f"
              " mm, steps/mm %.0f / %.0f\n" % (2 * conf[0], conf[1],
                                                conf[2], conf[3]))

        # ------------------------------------------- step 1: direction
        print("=" * 64)
        print("STEP 1: motor direction check")
        print("=" * 64)
        print("Strings attached to the pen holder, pen roughly centred,")
        print("hands clear of the strings. Each test winds 600 steps and")
        print("then unwinds them again, so the pen returns where it was.\n")
        invert_needed = []
        for ax, name in (("L", "LEFT"), ("R", "RIGHT")):
            input("Press ENTER to jog the %s motor IN by 600 steps..." % name)
            send_cmd(ser, "JOG %s 600" % ax)
            ans = input("Did the %s string get SHORTER (wound onto the "
                        "spool)? [y/n] " % name).strip().lower()
            send_cmd(ser, "JOG %s -600" % ax)
            if ans in ("n", "no"):
                invert_needed.append(ax)
                print("  -> Noted. It must be fixed before measuring:")
                print("     set  #define INVERT_%s 1  in StringPlotter.ino," % ax)
                print("     re-upload, then run this wizard again.")
            else:
                print("  -> OK.")
        if invert_needed:
            sys.exit("\nFix INVERT_%s first, re-upload, re-run."
                     % "/".join(invert_needed))

        # ------------------------------------------- step 2: steps/mm
        print()
        print("=" * 64)
        print("STEP 2: steps per millimetre  (ruler + marker, 5 min)")
        print("=" * 64)
        print("This is the GROUND TRUTH measurement. Do NOT skip it and do")
        print("not compute steps/mm from driver labels: the spool diameter")
        print("sets it, one motor can differ from the other, and 'the")
        print("vertical axis looks right' is NOT evidence it's correct.")
        ruler = {}
        if not args.skip_steps:
            print("\nWhich driver boards + jumper setting?")
            print("  [1] A4988   with all 3 jumpers  ->  3200 steps/rev")
            print("  [2] DRV8825 with all 3 jumpers  ->  6400 steps/rev")
            print("  [3] something else")
            choice = input("Choice [1/2/3]: ").strip()
            if choice == "1":
                spr = 3200
            elif choice == "2":
                spr = 6400
            else:
                spr = int(ask_float("Steps per motor revolution: ", 200, 25600))
            test_steps = 2 * spr     # two revolutions -> clearly measurable
            for ax, name in (("L", "LEFT"), ("R", "RIGHT")):
                print()
                print("--- %s motor ---" % name)
                print("1. Unhook the %s string from the pen holder." % name)
                print("2. Tie something light to its end (nut / paperclip)")
                print("   so it hangs straight and stays taut.")
                print("3. Tape a ruler vertically beside the thread. Put a")
                print("   tiny marker dot / tape flag on the thread right")
                print("   where it leaves the motor spool.")
                input("4. Press ENTER -> the motor winds IN %d steps "
                      "(2 revolutions)..." % test_steps)
                send_cmd(ser, "JOG %s %d" % (ax, test_steps))
                mm = ask_float("5. How many mm did the flag RISE? "
                               "(read the ruler): ", 5, 300)
                send_cmd(ser, "JOG %s -%d" % (ax, test_steps))
                ruler[ax] = test_steps / mm
                print("   -> %s motor: %.1f steps/mm  (spool diameter "
                      "~%.1f mm)" % (name, ruler[ax],
                                     spr / (math.pi * ruler[ax])))
            if abs(ruler["L"] / ruler["R"] - 1.0) > 0.05:
                print("\n  NOTE: the two motors differ by >5%% -- mixed")
                print("  driver types or different spools. That is fine,")
                print("  the two values are set separately.")
            ans = input("\nWrite these two values into polargraph.py now? "
                        "[Y/n] ").strip().lower()
            if ans not in ("n", "no"):
                write_config_values(pg_path,
                                   {"STEPS_PER_MM_L": round(ruler["L"], 1),
                                    "STEPS_PER_MM_R": round(ruler["R"], 1)})
                print("  polargraph.py updated (backup: polargraph.py.bak).")
            conf = (conf[0], conf[1], ruler["L"], ruler["R"])
        else:
            print("  (skipped with --skip-steps: fitting step 3 will have")
            print("   to find steps/mm from the drawing instead)")

        # ------------------------------------------- step 3: the cross
        print()
        print("=" * 64)
        print("STEP 3: closed-loop cross calibration")
        print("=" * 64)
        print("Re-hook both strings. Put the marker back in the holder.")
        print("Place the pen TIP exactly on the HOME dot (centre of the")
        print("%dx%d mm area, %g mm below the motor line)."
              % (pg.AREA_W_MM, pg.AREA_H_MM,
                 pg.TOP_MARGIN_MM + pg.AREA_H_MM / 2))
        print("This draws a 250 mm cross, you measure 5 numbers, the")
        print("script computes your machine's true geometry, rewrites the")
        print("config and redraws to PROVE it. Two to three rounds.\n")

        rounds = 0
        while True:
            rounds += 1
            input("Press ENTER to draw the calibration cross... ")
            draw_cross(ser, conf)
            print("  Cross drawn.")
            meas = measure_cross()
            if report_cross(meas):
                print("\n  *** CALIBRATED -- cross is square and straight ***")
                break
            if rounds > 4:
                print("\n  Not converging after 4 rounds. Physical suspects:")
                print("  thread slipping on a spool, a motor losing steps,")
                print("  spool wound in bulging layers, strings snagging.")
                break
            a_t, z_t, sl_t, sr_t, resid = fit_machine(conf, meas)
            print("\n  fit residual %.2f mm -> your real machine is:" % resid)
            print("    steps/mm  L %.1f   R %.1f   (config had %.1f / %.1f)"
                  % (sl_t, sr_t, conf[2], conf[3]))
            print("    separation %.0f mm (string exit points; config %.0f)"
                  % (2 * a_t, 2 * conf[0]))
            print("    string z-gap %.0f mm (config %.0f)"
                  % (z_t, conf[1]))
            if sl_t / conf[2] - 1 > 0.10 or conf[2] / sl_t - 1 > 0.10:
                print("    -> steps/mm were off >10%: THAT is what made the")
                print("       drawing oversized and arched the lines.")
            conf = apply_and_update(pg_path, conf, (a_t, z_t, sl_t, sr_t))
            print("  Redrawing with the new values to verify...")

        # ------------------------------------------- summary
        print()
        print("=" * 64)
        print("DONE.")
        print("=" * 64)
        print("Config in polargraph.py: separation %.0f mm, gap %.0f mm,"
              " steps/mm %.1f / %.1f" % (2 * conf[0], conf[1],
                                         conf[2], conf[3]))
        print()
        print("Final check: draw the verification pattern at 1:1 and measure:")
        print("    python polargraph.py ../svg/calibration_pattern.svg"
              " --scale 1 --port %s" % port)
        print("The box MUST come out 240 x 320 mm, diagonals and the")
        print("midline cross dead straight, ticks 50 mm apart. Then draw")
        print("anything you like.")
    finally:
        try:
            send_cmd(ser, "M18", timeout=3)
        except Exception:
            pass
        ser.close()
        print("\n(motors released, port closed)")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nAborted by user.")
        sys.exit(130)
