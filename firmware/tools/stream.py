#!/usr/bin/env python3
"""Minimal gcode streamer — bypasses UGS for debugging.

Usage:
    python3 stream.py patterns/spiral_square.gcode
    python3 stream.py patterns/spiral_square.gcode --port /dev/cu.usbmodem1101
"""
import argparse, sys, time, re
import serial

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("file")
    ap.add_argument("--port", default="/dev/cu.usbmodem1101")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    s = serial.Serial(args.port, args.baud, timeout=1)
    time.sleep(0.6)
    s.reset_input_buffer()

    with open(args.file) as f:
        lines = [ln.strip() for ln in f if ln.strip() and not ln.strip().startswith(';')]

    print(f"Streaming {len(lines)} lines from {args.file}...\n")
    for i, line in enumerate(lines, 1):
        s.write((line + "\n").encode())
        buf = ""
        start = time.time()
        last_poll = start
        done = False
        first_slow_print = False
        while time.time() - start < 60:
            chunk = s.read(s.in_waiting or 1).decode(errors="replace")
            if chunk:
                buf += chunk
                if first_slow_print:
                    # Raw byte-level printout so we can't miss anything.
                    sys.stdout.write(f"         <fw raw: {chunk!r}>\n")
                    sys.stdout.flush()
                if re.search(r"ok\r\n|error:\d+", buf):
                    done = True
                    break
            now = time.time()
            if now - last_poll > 0.5 and now - start > 1.5:
                if not first_slow_print:
                    print(f"   .. [{i:3d}] {line:<30}  waiting... (firmware state below, `?` polled every 0.5s)")
                    first_slow_print = True
                s.write(b"?")
                last_poll = now
        status = "ok" if "ok\r\n" in buf else ("timeout" if not done else buf.strip())
        if not first_slow_print:
            mark = "  " if status == "ok" else "!!"
            print(f"{mark} [{i:3d}] {line:<30}  →  {status}")
        else:
            mark = "  " if status == "ok" else "!!"
            print(f"{mark} [{i:3d}] {line:<30}  →  {status}  (took {time.time()-start:.1f}s)")
        if "error" in status:
            print("\n  Stopping on error.")
            sys.exit(1)
        if status == "timeout":
            print("\n  Timed out waiting for ok. Firmware is likely hung.")
            sys.exit(1)

    print("\nDone. All lines accepted by firmware.")
    s.close()

if __name__ == "__main__":
    main()
