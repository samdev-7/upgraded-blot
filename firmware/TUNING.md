# Blot Firmware — Max Speed / Accel Tuning

Adapted from Ellis's [Determining Max Speeds & Accels](https://ellis3dp.com/Print-Tuning-Guide/articles/determining_max_speeds_accels.html) with CoreXY adjustments.

## Why this is more involved than on a Cartesian printer

On Cartesian machines there's one max speed and one max accel per axis. Blot is CoreXY, so tip motion and motor rotation don't line up the same way on all angles:

| Direction | Motor activity (at tip speed `v`) | Cap that bites first |
|-----------|-----------------------------------|----------------------|
| Pure X or Y (axis-aligned) | Both motors turn at rate `v` | `$110` Cartesian feedrate = `$112` motor rate (load = 1) |
| ±45° diagonal | Only one motor turns, at rate `v·√2` | `$112` motor rate (at `v = $112/√2`) |

Same split applies to accel: axis-aligned stresses `$120`, diagonal stresses `$122`.

**So you have to tune both directions.** An axis-aligned test tells you nothing about how hard the single motor works on a diagonal, and vice versa.

## Settings recap

| `$N` | Meaning                                     | Tuned by |
|------|---------------------------------------------|----------|
| `$110` | Cartesian feedrate cap (mm/min)           | Axis-aligned speed test |
| `$112` | Motor-space rate cap (mm/min)             | **Diagonal** speed test |
| `$120` | Cartesian accel cap (mm/s²)               | Axis-aligned accel test |
| `$122` | Motor-space accel cap (mm/s²)             | **Diagonal** accel test |

The planner internally takes `min($110, $112 / load)` for speed and `min($120, $122 / load)` for accel. If you set `$110 = $112` and `$120 = $122`, diagonals auto-reduce; axis moves get the full cap. That's the recommended configuration.

## What you're actually measuring

**Step conservation.** After a sequence of moves that *should* return to (0, 0), ask for status with `?`. If `MPos:` shows anything other than `0.000,0.000`, the motor skipped steps during the run — that's your failure signal.

The firmware's own position counter (`motor_pos_steps`) is updated on every pulse emitted, so it reflects what the **firmware commanded**, not what the motor actually did. To measure real step loss you need to compare commanded vs mechanical — which you do by watching if the pen lands back on its starting mark. So:

1. Manually park the carriage, send `G92 X0 Y0`.
2. Drop a pen or mark the starting position (a bit of tape on paper under the pen).
3. Run the test.
4. After it finishes, pen should return to the mark. Misalignment = step loss.

## Procedure

### Baseline (do this first, every time)

```
G21 G90 G92 X0 Y0
M17
M5
```

Confirm:
- `$100` is calibrated (100 mm commanded = 100 mm on paper)
- Belt is tight, pulley grubscrews torqued
- Carriage isn't binding

Start from known-safe values — no point tuning around a mechanical issue.

```
$110=3000
$112=3000
$120=500
$122=500
```

(50 mm/s everywhere, 500 mm/s² accel. These are safe baselines the default would hit.)

### Test A — Axis-aligned speed (tunes `$110`)

Paste this to UGS with `<SPEED>` replaced by the value you're testing:

```
G21 G90 G92 X0 Y0
G1 X100 F<SPEED>
G1 X0
G1 X100 F<SPEED>
G1 X0
G1 X100 F<SPEED>
G1 X0
```

Then send `?` and read `MPos`. Should be `0.000,0.000`. If it is, bump:

```
SPEED: 3000 → 6000 → 9000 → 12000 → 15000 → ...
```

Raise `$110` to match each test:

```
$110=<SPEED>
```

When `MPos` drifts (or pen misses the mark), **back off to 80% of the last working speed** and that's your `$110`. Repeat with `Y` instead of `X` and take the smaller of the two.

### Test B — Diagonal speed (tunes `$112`)

Diagonals are how you find the motor-rate ceiling. Paste:

```
G21 G90 G92 X0 Y0
G1 X50 Y50 F<SPEED>
G1 X0 Y0
G1 X50 Y50 F<SPEED>
G1 X0 Y0
G1 X50 Y50 F<SPEED>
G1 X0 Y0
```

`F<SPEED>` here is the commanded Cartesian feedrate. On a 45° diagonal the firmware internally clamps to `$112 / √2` tip speed, so:

- To make the motor actually hit its cap, set `$112 = SPEED × √2`. E.g. to test motor at 10000 mm/min, command `F7071` on the diagonal.
- Or just raise `$112` to a large number (e.g. `30000`) and keep pushing `F` up until diagonal fails; the actual motor rate is `F × √2`.

Same back-off rule: 80% of the last working, record as `$112`.

### Test C — Axis-aligned accel (tunes `$120`)

Same pattern but we need **short moves** — acceleration only shows up when there's not enough distance to cruise, so the machine spends most of the time accelerating/decelerating. Use 10-20 mm strokes:

```
G21 G90 G92 X0 Y0
G1 X20 F6000
G1 X0 F6000
G1 X20 F6000
G1 X0 F6000
G1 X20 F6000
G1 X0 F6000
```

Raise `$120` between runs:

```
$120=500 → 1000 → 1500 → 2500 → 4000 → ...
```

At each step, confirm `MPos` stays at `0.000,0.000` after the test. You'll hear the motors struggle (louder, grindy stall sound) before you see position drift — either symptom is "too much," back off to 80%.

### Test D — Diagonal accel (tunes `$122`)

Same as C but on the diagonal:

```
G21 G90 G92 X0 Y0
G1 X14 Y14 F6000
G1 X0 Y0
G1 X14 Y14 F6000
G1 X0 Y0
G1 X14 Y14 F6000
G1 X0 Y0
```

Raise `$122` between runs. When you find the limit, set `$122` to 80% of it.

## Save and verify

The firmware auto-persists every `$N=` to flash, so your tuned values survive power cycles. Confirm with `$$`.

After tuning, run a small real art piece (not a test pattern) at your new settings and look for any visible inconsistency — that catches anything that test patterns miss.

## Values I've seen work on a tuned Blot

These are rough — yours may differ. Start below and measure:

| Setting | Conservative | Aggressive |
|---------|--------------|------------|
| `$110`  | 3000         | 9000       |
| `$112`  | 3000         | 12000      |
| `$120`  | 500          | 2000       |
| `$122`  | 500          | 2500       |

`$112` and `$122` are typically higher than `$110` and `$120` because axis-aligned Cartesian moves don't need to match the motor's peak — the planner clamps Cartesian first.

## Quick one-shot test macro (for iterating)

When you find your settings, sanity-check with a combined pattern:

```
G21 G90 G92 X0 Y0
F<FEED>
G1 X100 Y0
G1 X100 Y100
G1 X0 Y100
G1 X0 Y0
G1 X100 Y100
G1 X0 Y0
G1 X50 Y100
G1 X0 Y0
```

Run it 2-3 times in a row, then `?`. `MPos:0.000,0.000` means the step budget balanced across axis moves, axis returns, diagonal moves, and arbitrary-angle moves all at once.
