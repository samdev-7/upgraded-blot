#!/usr/bin/env python3
"""Blot firmware tuning helper — automates the procedure in TUNING.md.

Walks you through each of the four tests (C/D/A/B, in that order),
runs the test pattern at successively higher values, and asks whether
the pen returned cleanly to its starting mark. When you say no (or you
hear stalls), the script backs off to 80% of the last passing value
and saves it via the matching `$N=` so it persists to flash.

Tests run in **accel-first** order (C, D, A, B) on purpose — speed
tests need high accel to actually reach their commanded F. Running a
speed test with low accel would mis-report the ceiling because the
move would be acceleration-limited (v_peak = √(a·L)), not
feedrate-limited.

Any setting the script mutates *temporarily* (to keep a test from
clamping itself) is restored from the pre-run snapshot if that
setting's tuning test is skipped or fails to converge.

Usage:
    pip3 install pyserial                  # once
    python3 tune.py                        # coarse tune, 1.5× steps, 80% backoff
    python3 tune.py --fine                 # fine tune, 1.1× steps, 90% backoff,
                                           #   starts just below current stored value
    python3 tune.py --port /dev/cu.usbmodemXXXX
    python3 tune.py --tests CA             # axis accel + axis speed only

Between tests:
- The pen stays on paper the whole time a test is running.
- Check the endpoint against the first dot the script made at origin.
- Listen for stepper stalls (grindy/clacky noise) — those count as a fail.
"""

import argparse
import re
import sys
import time

try:
    import serial
except ImportError:
    sys.stderr.write("pyserial is not installed. Run:\n  pip3 install pyserial\n")
    sys.exit(1)


DEFAULT_PORT = "/dev/cu.usbmodem1101"
BAUD = 115200


# ───────── serial plumbing ─────────

def open_port(port):
    try:
        s = serial.Serial(port, BAUD, timeout=1)
    except serial.SerialException as e:
        sys.stderr.write(
            f"Can't open {port}: {e}\nIs UGS connected? Close it and try again.\n"
        )
        sys.exit(1)
    time.sleep(0.6)
    s.reset_input_buffer()
    return s


def read_until(s, pattern, timeout):
    """Read from serial until `pattern` (regex) appears or timeout."""
    buf = ""
    deadline = time.time() + timeout
    rx = re.compile(pattern)
    while time.time() < deadline:
        chunk = s.read(s.in_waiting or 1).decode(errors="replace")
        if chunk:
            buf += chunk
            if rx.search(buf):
                return buf
        else:
            time.sleep(0.01)
    return buf


def send_line(s, line, timeout=10.0):
    """Send a line + \\n, wait for ok/error. Returns 'ok' or 'error:N' or 'timeout'."""
    s.write((line + "\n").encode())
    buf = read_until(s, r"ok\r\n|error:\d+", timeout)
    if "ok\r\n" in buf: return "ok"
    m = re.search(r"error:(\d+)", buf)
    if m: return f"error:{m.group(1)}"
    return "timeout"


def status(s, timeout=0.6):
    """`?` realtime → (state, mx, my)."""
    s.reset_input_buffer()
    s.write(b"?")
    buf = read_until(s, r"<\w+\|MPos:[-\d.]+,[-\d.]+", timeout)
    m = re.search(r"<(\w+)\|MPos:([-\d.]+),([-\d.]+)", buf)
    if m:
        return m.group(1), float(m.group(2)), float(m.group(3))
    return "Unknown", 0.0, 0.0


def wait_idle(s, timeout=120.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        st, _, _ = status(s)
        if st == "Idle":
            return
        time.sleep(0.15)
    raise RuntimeError("Timed out waiting for Idle")


def stream(s, lines, per_line_timeout=15.0):
    for line in lines:
        line = line.strip()
        if not line: continue
        r = send_line(s, line, timeout=per_line_timeout)
        if r.startswith("error"):
            print(f"    ! [{line}] → {r}", file=sys.stderr)


def parse_all_settings(s):
    """Send $$, parse response, return dict like {'$110': 1500.0, ...}."""
    s.reset_input_buffer()
    s.write(b"$$\n")
    buf = read_until(s, r"ok\r\n", timeout=3.0)
    return {m.group(1): float(m.group(2))
            for m in re.finditer(r"(\$\d+)=([-\d.]+)", buf)}


def prompt(msg, valid=("y", "n", "r", "q")):
    while True:
        r = input(msg).strip().lower()
        if r in valid: return r
        print(f"  (choose one of: {', '.join(valid)})")


# ───────── test patterns ─────────
# Each pattern fn takes the value under test (info only — the test_block
# wrapper handles the actual $N= set) and returns motion lines. Uses plain
# M3 (no S) so we don't overwrite the user's servo_pen_down calibration.

def pattern_a(speed_mm_min):
    return [
        f"G1 X100 F{int(speed_mm_min)}", "G1 X0",
        f"G1 X100 F{int(speed_mm_min)}", "G1 X0",
        f"G1 X100 F{int(speed_mm_min)}", "G1 X0",
    ]


def pattern_b(speed_mm_min):
    return [
        f"G1 X50 Y50 F{int(speed_mm_min)}", "G1 X0 Y0",
        f"G1 X50 Y50 F{int(speed_mm_min)}", "G1 X0 Y0",
        f"G1 X50 Y50 F{int(speed_mm_min)}", "G1 X0 Y0",
    ]


def pattern_c(_accel):
    # F is pinned at the ISR ceiling so the cruise cap never binds — v_peak
    # ends up as √(a·L) (pure triangle profile), so every $120 step produces
    # an audibly different peak pitch. At $120=10000, v_peak=√(10000·20)=447
    # mm/s which is still under the 500 mm/s ISR ceiling.
    return [
        "G1 X20 F30000", "G1 X0",
        "G1 X20 F30000", "G1 X0",
        "G1 X20 F30000", "G1 X0",
    ]


def pattern_d(_accel):
    # Same reasoning as pattern_c — keep the move accel-limited, not cruise-limited.
    return [
        "G1 X14 Y14 F30000", "G1 X0 Y0",
        "G1 X14 Y14 F30000", "G1 X0 Y0",
        "G1 X14 Y14 F30000", "G1 X0 Y0",
    ]


def test_block(pattern_lines):
    """Uniform wrapper: zero origin, pen down, trace, pen up. Plain M3 (no S)
    so we don't rewrite the user's calibrated servo_pen_down on every run."""
    return (
        ["G21 G90", "M17", "G92 X0 Y0", "M3", "G4 P100"]
        + pattern_lines
        + ["M5"]
    )


# ───────── correctness guards ─────────

def max_meaningful_speed_mm_min(accel_mm_s2, length_mm):
    """Above this commanded F, the motion is accel-limited and never actually
    reaches the requested speed (triangle profile v_peak = √(a·L)).
    Returning it in mm/min for direct comparison to F values."""
    import math
    return math.sqrt(accel_mm_s2 * length_mm) * 60.0


# ───────── the iterative search ─────────

def run_tuning(s, name, setting, pattern_fn, start, growth, min_step,
               accel_mm_s2=None, effective_length_mm=None,
               hard_cap=None, backoff=0.8):
    """Run the test pattern at increasing values. Returns the tuned value
    (after `backoff` fraction applied) or None if nothing passed.

    If `accel_mm_s2` and `effective_length_mm` are given, warns once when
    the commanded speed crosses the accel-limited ceiling (so the user
    knows later 'y' answers aren't measuring the motor anymore).

    If `hard_cap` is given, the ramp stops once the value would exceed it
    (used for the ISR step-rate ceiling, above which the firmware clamps
    internally and y-answers would be false positives)."""
    print(f"\n=== {name} ({setting}) ===")
    print(f"Start {start}, step ×{growth}.")

    meaningful_ceiling = None
    if accel_mm_s2 and effective_length_mm:
        meaningful_ceiling = max_meaningful_speed_mm_min(
            accel_mm_s2, effective_length_mm
        )
        print(f"(Accel-limited ceiling for this test geometry: "
              f"~F{int(meaningful_ceiling)}. Above that, commanded F is "
              f"not the actual speed the motor sees.)")
    print()

    value = float(start)
    last_good = None
    warned_ceiling = False

    while True:
        if meaningful_ceiling and value > meaningful_ceiling and not warned_ceiling:
            print(f"  ! F{int(value)} exceeds the accel-limited ceiling "
                  f"(~F{int(meaningful_ceiling)}). Further y-answers are "
                  f"NOT measuring the motor — raise $120/$122 if you want "
                  f"to go higher.")
            warned_ceiling = True

        print(f"Setting {setting} = {int(value)}...")
        if send_line(s, f"{setting}={int(value)}") != "ok":
            print(f"  ! could not set {setting}, aborting test")
            break

        stream(s, test_block(pattern_fn(value)))
        try:
            wait_idle(s)
        except RuntimeError as e:
            print(f"  ! {e}")
            break

        _, mx, my = status(s)
        print(f"  MPos after test: ({mx:.3f}, {my:.3f})")
        r = prompt(
            "  Pen returned cleanly, no stalls? [y/n/r=retry/q=quit tuning]: "
        )
        if r == "q":
            print("  quitting this test")
            break
        if r == "r":
            continue
        if r == "y":
            last_good = value
            value *= growth
            if hard_cap and value > hard_cap:
                print(f"  → Reached firmware ISR ceiling ({int(hard_cap)}); "
                      f"stopping ramp. Motor literally cannot step faster "
                      f"without a MIN_STEP_INTERVAL_US change.")
                break
            if meaningful_ceiling and value > meaningful_ceiling:
                print(f"  → Next F{int(value)} exceeds the accel-limited "
                      f"ceiling (~F{int(meaningful_ceiling)}) — higher "
                      f"commanded F would be clipped by the accel ramp and "
                      f"the test can't distinguish it. Stopping ramp.")
                break
        else:  # "n"
            break

    if last_good is None:
        print(f"  ! No passing value found for {setting}.")
        return None
    final = max(int(min_step), int(last_good * backoff))
    print(f"  → Last passing {setting} = {int(last_good)}. "
          f"Saving {int(backoff * 100)}% → {final}.")
    send_line(s, f"{setting}={final}")
    return final


# ───────── main ─────────

def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default=DEFAULT_PORT,
                    help=f"Serial port (default: {DEFAULT_PORT})")
    ap.add_argument("--tests", default="CDAB",
                    help="Which tests to run (any subset of ABCD; "
                         "default CDAB runs accel before speed)")
    ap.add_argument("--fine", action="store_true",
                    help="Fine-tune mode: 1.1× growth, 90%% backoff, starts "
                         "at 0.95× the currently-stored value. Use after a "
                         "coarse tune has landed you in the right ballpark.")
    args = ap.parse_args()

    # Coarse defaults (original behavior) vs fine-tune parameters.
    if args.fine:
        growth  = 1.1
        backoff = 0.9
        start_frac = 0.95   # first iteration sits just below current stored value
        start_floor_accel = 0    # no floor — use stored value
        start_floor_speed = 0
    else:
        growth  = 1.5
        backoff = 0.8
        start_frac = 1.0
        start_floor_accel = 5000
        start_floor_speed = 5000

    s = open_port(args.port)

    # Probe firmware version
    send_line(s, "$I")
    probe_buf = read_until(s, r"\]\r\n", timeout=0.5)
    if "Blot" not in probe_buf:
        print(f"  ! version probe didn't see 'Blot' — is this the right firmware?")

    # Snapshot original settings so we can restore anything we touch but
    # don't end up successfully tuning.
    orig = parse_all_settings(s)
    print(f"\nCurrent persisted settings:")
    for k in ("$110", "$111", "$112", "$120", "$122"):
        if k in orig:
            print(f"  {k} = {orig[k]:.0f}")

    print("""
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 Blot tuning — follow firmware/TUNING.md

 PHYSICAL SETUP (do this before continuing):
   1. Put a pen in the holder with tip touching paper.
   2. Tape down paper so it doesn't slide.
   3. Move the carriage BY HAND to the origin you want. Leave at
      least 120 mm of room in +X and +Y (100 mm for axis tests,
      ~71 mm for diagonals).
   4. Close UGS if it's open (port can only have one owner).
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
""")
    input("Press ENTER when you're ready → ")

    print("\nMarking origin...")
    # Plain M3 (no S) — don't overwrite the user's calibrated $151 pen-down.
    stream(s, ["M17", "G21 G90", "G92 X0 Y0", "M3", "G4 P400", "M5"])
    wait_idle(s)
    print("Origin dot made. All subsequent tests should return to this point.\n")

    results = {}          # {"$110": int_value, ...} for tests that found a value
    touched = set()       # settings we mutated via prereq; may need restore

    # Pick starting values from the stored settings so re-tuning picks up
    # where we left off. Coarse mode applies a floor (5000) so first-run
    # iterations aren't wasted on obviously-passing low values; fine mode
    # starts at start_frac × stored so we're right near the previous
    # ceiling and step in small increments from there.
    def start_for(setting_key, accel):
        stored = int(orig.get(setting_key, 5000))
        floor = start_floor_accel if accel else start_floor_speed
        return max(floor, int(stored * start_frac))

    c_start = start_for("$120", accel=True)
    d_start = start_for("$122", accel=True)
    a_start = start_for("$110", accel=False)
    b_start = start_for("$112", accel=False)

    print("Tuning will begin at:")
    print(f"  $120 (axis accel)   = {c_start}")
    print(f"  $122 (diag accel)   = {d_start}")
    print(f"  $110 (axis speed)   = {a_start}")
    print(f"  $112 (diag speed)   = {b_start}")
    print(f"First iteration confirms the start value holds; "
          f"subsequent iterations ramp up ×{growth}. "
          f"Backoff on the last-passing value is {int(backoff * 100)}%.")

    # Push-out-of-the-way caps for cross-constraints. These must be higher
    # than ANY value the tuning loop might reach, otherwise they'd silently
    # bind and we'd measure the push-cap instead of the variable under test.
    PUSH_SPEED = 3_000_000    # mm/min — astronomical on purpose
    PUSH_ACCEL = 1_000_000    # mm/s²

    # Firmware ISR has MIN_STEP_INTERVAL_US=25 → 40 kHz step events →
    # 500 mm/s motor rate → 30000 mm/min. Commanding higher is pointless —
    # the ISR clamps and we'd get false passes.
    ISR_MAX_SPEED = 30000     # mm/min — passed as ceiling to speed tests

    print(f"\nNote: firmware ISR caps step rate at ~{ISR_MAX_SPEED} mm/min. "
          f"Tests stop walking up once they reach this ceiling — the motor "
          f"literally can't move faster without a MIN_STEP_INTERVAL_US change.")

    try:
        tests = args.tests.upper()

        # Test C — tune $120 (axis accel).
        # On pure X, motor_load = 1, so:
        #   effective tip speed = min($110, $112)  ← push BOTH out of the way
        #   effective tip accel = min($120, $122)  ← push $122, tune $120
        # Missing either speed push silently clamps cruise velocity, which
        # makes higher $120 values stop producing audibly faster motion.
        if "C" in tests:
            touched.update({"$110", "$112", "$122"})
            send_line(s, f"$110={max(PUSH_SPEED, int(orig.get('$110', 1500)))}")
            send_line(s, f"$112={max(PUSH_SPEED, int(orig.get('$112', 1500)))}")
            send_line(s, f"$122={max(PUSH_ACCEL, int(orig.get('$122', 500)))}")
            results["$120"] = run_tuning(
                s, "Test C: axis-aligned accel", "$120",
                pattern_c, start=c_start, growth=growth, min_step=200,
                backoff=backoff,
            )

        # Test D — tune $122 (diagonal/motor accel).
        # On 45° diagonal, motor_load = √2, so:
        #   effective tip speed = min($110, $112/√2)  ← push BOTH
        #   effective tip accel = min($120, $122/√2)  ← push $120, tune $122
        if "D" in tests:
            touched.update({"$110", "$112", "$120"})
            send_line(s, f"$110={max(PUSH_SPEED, int(orig.get('$110', 1500)))}")
            send_line(s, f"$112={max(PUSH_SPEED, int(orig.get('$112', 1500)))}")
            send_line(s, f"$120={max(PUSH_ACCEL, int(orig.get('$120', 200)))}")
            results["$122"] = run_tuning(
                s, "Test D: diagonal accel", "$122",
                pattern_d, start=d_start, growth=growth, min_step=200,
                backoff=backoff,
            )

        # Test A — tune $110 (axis speed).
        # Effective axis speed = min($110, $112). Push $112 out of the way.
        # Accel caps stay at the TUNED/SAFE values — we don't want to
        # stress the motor with both unknown-high speed AND high accel.
        if "A" in tests:
            touched.update({"$120", "$122", "$112"})
            a_accel  = results.get("$120") or int(orig.get("$120", 500))
            d_accel  = results.get("$122") or int(orig.get("$122", 500))
            send_line(s, f"$120={a_accel}")
            send_line(s, f"$122={d_accel}")
            send_line(s, f"$112={max(PUSH_SPEED, int(orig.get('$112', 1500)))}")
            results["$110"] = run_tuning(
                s, "Test A: axis-aligned speed", "$110",
                pattern_a, start=a_start, growth=growth, min_step=1500,
                accel_mm_s2=a_accel, effective_length_mm=100.0,
                hard_cap=ISR_MAX_SPEED,
                backoff=backoff,
            )

        # Test B — tune $112 (diagonal/motor speed).
        # Effective diag tip speed = min($110, $112/√2). Push $110 out.
        if "B" in tests:
            touched.update({"$120", "$122", "$110"})
            a_accel  = results.get("$120") or int(orig.get("$120", 500))
            d_accel  = results.get("$122") or int(orig.get("$122", 500))
            send_line(s, f"$120={a_accel}")
            send_line(s, f"$122={d_accel}")
            send_line(s, f"$110={max(PUSH_SPEED, int(orig.get('$110', 1500)))}")
            # On 45° diagonal, Cartesian accel = $122/√2. Cartesian length
            # on (50,50) = √(50²+50²) ≈ 70.7 mm.
            import math
            a_eff = d_accel / math.sqrt(2.0)
            results["$112"] = run_tuning(
                s, "Test B: diagonal speed", "$112",
                pattern_b, start=b_start, growth=growth, min_step=1500,
                accel_mm_s2=a_eff, effective_length_mm=70.7,
                hard_cap=ISR_MAX_SPEED,
                backoff=backoff,
            )

    finally:
        # Re-write every touched setting to its "correct" final value:
        #   - tuned (results[k] set)  → tuned value
        #   - not tuned               → pre-run snapshot value
        # Blindly re-writing catches the case where a later test's prereq
        # push (e.g. Test B setting $110=PUSH_SPEED) overwrote a value that
        # an earlier test had already tuned. The old "only restore if not
        # tuned" logic left those pushed sentinels stuck in flash.
        for k in ("$110", "$112", "$120", "$122"):
            if k not in touched and k not in results:
                continue
            desired = results.get(k) if results.get(k) is not None else orig.get(k)
            if desired is None:
                continue
            print(f"  writing {k} = {int(desired)}")
            send_line(s, f"{k}={int(desired)}")

        # $111 (rapid) wasn't tested — keep it at max($110, orig $111) so
        # pen-up traversal is at least as fast as feed moves. Cap at $112
        # if we have one, since the motor-rate cap is the real ceiling.
        final_110 = results.get("$110") or orig.get("$110", 1500)
        final_112 = results.get("$112") or orig.get("$112", 1500)
        target_111 = min(final_112, max(final_110, orig.get("$111", 1500)))
        send_line(s, f"$111={int(target_111)}")

        # Pen up + motors off, regardless of what happened.
        send_line(s, "M5")
        send_line(s, "M18")

    # Summary
    print("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
    print("Final persisted values:")
    final = parse_all_settings(s)
    for k in ("$110", "$111", "$112", "$120", "$122"):
        new = final.get(k)
        was = orig.get(k)
        if new is None: continue
        changed = "" if was == new else f"  (was {int(was) if was else '?'})"
        tuned = " [tuned]" if results.get(k) is not None else ""
        print(f"  {k:5} = {int(new)}{tuned}{changed}")
    print("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
    print("\nDone. Tuning values persist across power cycles.")
    s.close()


if __name__ == "__main__":
    main()
