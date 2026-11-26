# Polargraph Plotter

A wall-mounted, string-driven 2-axis Polargraph (V-plotter) built with an Arduino UNO, CNC Shield V3, DRV8825 stepper drivers, and NEMA 17 motors. Drop in any SVG drawing and let the machine plot it directly onto a whiteboard.

---

## Demo & Results

<!-- Drag and drop your .mp4 video clip here using GitHub's web editor -->

| Build Setup | Final Output |
| :---: | :---: |
| ![Build Setup](media/build.jpg) | ![Final Result](media/result.jpg) |

---

## Hardware & Wiring

![Polargraph Plotter Wiring Schematic](docs/wiring_schematic.jpg)

### What's Inside
* **Controller:** Arduino UNO + CNC Shield V3
* **Motors:** 2x NEMA 17 Stepper Motors (`X` = Left motor, `Y` = Right motor)
* **Drivers:** 2x DRV8825 Stepper Drivers with heatsinks (all 3 microstep jumpers installed)
* **Power Supply:** 12 V 10 A SMPS (motors) + USB cable (Arduino & laptop)
* **Pen Holder:** 3D-printed gondola suspended on two strings (continuous drawing, no pen-lift servo)

### Board Layout
* **Motor Spacing:** 480 mm apart (string exit points)
* **Drawing Area:** 300 x 400 mm (starting 40 mm below the motor line)
* **HOME Dot:** Marked at the exact centre of the drawing area (240 mm below the motor line)

---

## First-Time Setup

1. **Upload the Firmware:** Open `firmware/StringPlotter/StringPlotter.ino` in the Arduino IDE, select **Arduino UNO**, and click **Upload**.
2. **Install Python Requirements:** Make sure Python 3.8+ is installed, then run:
   ```bash
   cd python
   pip install -r requirements.txt
   ```
   *(On Windows, install the CH340 driver if your COM port doesn't show up, and close any other serial apps before starting.)*
3. **Run Calibration (Once):** Measure and tune the machine with a ruler (~10 minutes):
   ```bash
   python calibrate.py --port COM5
   ```
   Follow the on-screen prompts to check motor directions, measure string travel, and draw a 250 mm test cross.

---

## How to Use

### 1. Prepare Your SVG
Use any vector SVG file (shapes, lines, paths). If your design has text, convert it to outlines first (in Inkscape: **Path → Object to Path**). Embedded photos are ignored.

### 2. Preview Before Drawing
Always run a quick simulation first to see the exact path the pen will take:
```bash
python polargraph.py yourfile.svg --simulate
```
* **Blue lines** = drawing path
* **Dotted grey lines** = travel moves

### 3. Draw on the Board
1. Power on the 12 V supply and plug the Arduino into your laptop via USB.
2. Run the plotter command (replace `COM5` with your port):
   ```bash
   python polargraph.py yourfile.svg --port COM5
   ```
3. Place the marker tip directly on your **HOME** dot and press **Enter** to set zero.
4. Press **Enter** again to start drawing.
5. Press **`Ctrl + C`** at any time to stop safely.

