# upgraded-blot

> **Heads up.** This project was built in an afternoon for RMRRF. Changes from the stock blot were made with the help of AI (Claude Opus). It's not my best work, but it's what I could do with limited time.

![](https://cdn.hackclub.com/019dac2d-420c-7c59-98bc-546af7972c20/frame_3.jpg)

A fork of [Blot](https://blot.hackclub.com) (Hack Club's 2-axis pen plotter) for GRBL-compatibility and photo-to-plotter desktop app. Originally built for [RMRRF](https://rmrrf.com) 2026 as an interactive exhibit. See the [original Blot repo](https://github.com/hackclub/blot) for hardware design, assembly instructions, and the stock firmware.

## Firmware for GRBL support

Stock Blot's firmware uses a hand-rolled serial protocol that is inefficient and stuttery. This is okay for the kind of drawings Blot ships with, but it leaves a lot on the table. There's no acceleration planning and no compatibility with existing G-code tools.

**`firmware/` is a drop-in replacement that teaches Blot to accept reasonably advanced G-code:**

- **Motion planning.** Trapezoidal velocity profiles with junction deviation at corners, planned in Cartesian tip-space so CoreXY diagonals don't get clipped. Buffered streaming at 115200. The host keeps the next 10–50 moves queued so motion overlaps parsing.
- **Native arcs.** `G2`/`G3` with `I/J` or `R`. No more pre-tessellating circles into a hundred line segments.
- **Runtime settings.** `$110` max feedrate, `$120` max acceleration, `$151` servo pen-down µs, etc. Tuned once with `$$` and persisted, not baked into firmware.
- **GRBL-compatible protocol.** Any standard G-code sender (Universal G-code Sender, bCNC, OctoPrint+GRBL plugin, or a dumb serial script) can drive it. The pen maps onto GRBL's spindle commands: `M3 S<microseconds>` = pen down, `M5` = pen up.

> This is a **subset** of GRBL 1.1f, not a full reimplementation. Coolant, spindle PWM, probing, tool change, and hard homing are deliberately skipped because a pen plotter doesn't need them. See [`firmware/SUPPORTED_COMMANDS.md`](firmware/SUPPORTED_COMMANDS.md) for the exact supported set.

## Optional UI to draw portraits

**`ui/`** is a Mac desktop app I made on top of the firmware: it takes a photo (webcam or file), runs it through a stipple / scribble / long-line algorithm, previews the output, and streams the resulting G-code over USB.

I built it for [RMRRF](https://rmrrf.com) 2026 as an interactive exhibit, the UI is just a convenient bundle of _photo → pen strokes → send_ for the event. **Any GRBL-speaking sender will drive the plotter just fine once the firmware is flashed**

Everything else (hardware, wiring, assembly, the Blot bill of materials) is unchanged. Use Hack Club's docs in the top level folder for that.

> The UI is Mac-only for now as it uses Apple Vision for local background removal and AVFoundation for camera capture. The firmware, however, is cross-platform and can be driven by any G-code sender on any OS.

## Folder layout

```
firmware/   Arduino firmware for the Seeed XIAO RP2040 on Blot
├── firmware.ino + *.cpp/*.h   ← flash this to the board
├── TUNING.md                  ← calibration after assembly
├── SUPPORTED_COMMANDS.md      ← full G-code reference
├── tools/                     ← stream.py, term.py, tune.py
├── test/                      ← host-side test harness (see test/README.md)
└── test_gcode/                ← sample G-code programs for integration tests

ui/         Desktop app (Mac-only for now)
├── blot.sh                    ← launcher
├── blot_ui.py                 ← the Qt app itself
├── *_lineart.py               ← stipple / scribble / long-line algos
├── mask_*.py, remove_bg.swift ← Apple Vision bg-removal helpers
├── combing.py                 ← keeps pen-up travel inside foreground
└── blot_vpype.toml, blot.vpy  ← vpype profile + CLI recipe
```

## Usage

### Install prerequisites (one time)

You need macOS 12+, and an assembled Blot plotter.

```bash
# 1. Homebrew (skip if you already have it)
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# 2. Python + pipx
brew install python@3.13 pipx
pipx ensurepath
# close and reopen Terminal so PATH picks up

# 3. vpype + the UI's Python deps
pipx install --python python3.13 "vpype[all]"
pipx inject vpype pyserial scikit-image pyobjc-framework-AVFoundation
```

### Flash the firmware (one time, or whenever you change it)

Install the [Arduino IDE](https://www.arduino.cc/en/software), then:

1. Arduino → Settings → _Additional boards manager URLs_, paste `https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json`.
2. Tools → Board → Boards Manager → search "pico" → install **Raspberry Pi Pico/RP2040** by Earle F. Philhower.
3. Tools → Board → Raspberry Pi Pico/RP2040 → **Seeed XIAO RP2040**.
4. File → Open → `firmware/firmware.ino`. The IDE picks up the other `.cpp`/`.h` files in that folder automatically.
5. Plug in your Blot. Tools → Port → pick the `/dev/cu.usbmodem…` entry.
6. Click the upload arrow (top-left). Wait for "Done uploading."

After flashing, [`tools/tune.py`](firmware/tools/tune.py) walks you through motor calibration. See [`firmware/TUNING.md`](firmware/TUNING.md).

### Run the desktop app (optional, for photo → plot)

From the repo root:

```bash
chmod +x ui/blot.sh       # first run only
./ui/blot.sh
```

The first launch triggers the macOS camera-permission prompt. Click Allow. If you miss it, toggle it back on under **System Settings → Privacy & Security → Camera**.

You'll see a window with three tabs.

**Camera**: pick your webcam, point it at something, **Capture**, drag the square overlay to crop, then **Use cropped photo →**.

**Generate**: pick an algorithm (Stipple / Scribble / Long lines), tweak parameters, click **Generate**. After a few seconds you'll see a plotter preview with each pen layer in its own color. Toggle **Branding** off if you don't want the `HACKCLUB.COM` watermark; adjust its **Scale** slider separately.

**Control**: plug in the plotter, pick the USB port, **Connect**. Quick-command buttons handle jog, pen up/down, and setting origin. When ready, click **Send G-code** to stream the drawing. Pause/Cancel work while it runs.

### Typical first-plot sequence

1. Tape a piece of paper to the bed.
2. Control tab → **Connect**.
3. **Home the machine.**
   - Click **Disable motors (M18)**. The carriage now moves freely by hand.
   - Hand-move the carriage to where you want the top-left of the drawing.
   - Click **Enable motors (M17)** to lock the carriage in place.
   - Click **Set origin (G92)** to call that spot (0, 0).
4. **Adjust the pen.**
   - Click **Pen down (M3)**. The holder drops to its down position.
   - Slide the pen into the holder so that when the pen tip is resting on the paper, the holder sits _slightly_ above its mechanical bottom. You want gravity pulling the pen down onto the paper, that way the tip self-levels.
   - Tighten the holder around the pen.
   - Click **Pen up (M5)** and verify the tip clears the paper with room to spare.
5. Click **Send G-code**. Watch the first few strokes. If anything looks wrong, hit **Cancel** or press the physical reset on the board.

## Talking to the firmware without the UI

Because the protocol is GRBL-compatible, any serial terminal or G-code sender can drive it at 115200 baud. Two helpers ship in [`firmware/tools/`](firmware/tools/):

- `term.py`: raw serial REPL
- `stream.py`: file-based G-code streamer with flow control
- `tune.py`: guided motor calibration (uses the `$$` settings)

## Credits

Blot itself, and the hardware design, are by [Hack Club](https://hackclub.com). This fork just swaps the firmware protocol and bolts a nicer UI on top.
