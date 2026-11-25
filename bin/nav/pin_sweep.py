#!/usr/bin/env python3
"""Open-loop PWM sweep per motor and direction, printing the unloaded speed reached at each level.
Run on the robot with the client launch stopped. Usage: pin_sweep.py [MOTORS e.g. 2,3] [PORT]
A direction that is proportionally slower than the others across the whole range points at a weak
driver half or its wiring rather than the motor (the same motor runs the other way)."""
import struct, sys, time
import serial

MOTORS = [int(m) for m in (sys.argv[1] if len(sys.argv) > 1 else "2,3").split(",")]
PORT = sys.argv[2] if len(sys.argv) > 2 else "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5734047665-if00"
LEVELS = [15, 30, 45, 60, 76]                # rpm targets; open loop maps rpm/76 -> PWM 50..255
NAMES = ["L1 front-left", "L2 rear-left", "R1 front-right", "R2 rear-right"]
PINS = [15, 16, 17, 18, 4, 5, 6, 7]
REVERSED = [1, 1, 0, 1]

ser = serial.Serial(PORT, 115200, timeout=0.05)
time.sleep(2.5); ser.reset_input_buffer()
rpms = [0.0] * 4; buf = bytearray()

def pump(secs):
    end = time.monotonic() + secs
    while time.monotonic() < end:
        buf.extend(ser.read(256))
        while buf:
            if buf[0] == ord("M"):
                if len(buf) < 27: break
                if buf[1] == 24:
                    payload, chk = buf[2:26], buf[26]; x = 0
                    for b in payload: x ^= b
                    if x == chk:
                        vals = struct.unpack("<" + "hi" * 4, bytes(payload))
                        for i in range(4): rpms[i] = vals[2 * i] / 100.0
                        del buf[:27]; continue
                del buf[:1]; continue
            nl = buf.find(b"\n")
            if nl == -1:
                if len(buf) > 200: del buf[:1]
                break
            del buf[:nl + 1]

def cmd(s, wait=0.15):
    ser.write((s + "\n").encode()); pump(wait)

cmd("stop"); cmd("set open_loop 1")
print(f"{'motor':16s} {'dir':>4s} {'pin':>4s} " + " ".join(f"PWM{round(l / 76 * 255):>4d}" for l in LEVELS) + "   (unloaded rpm, mean of last 0.4 s)")
for m in MOTORS:
    for sign in (+1, -1):
        value = -sign if REVERSED[m] else sign
        pin = PINS[2 * m] if value > 0 else PINS[2 * m + 1]
        row = []
        for lvl in LEVELS:
            cmd(f"S {m} {sign * lvl}", 0.05)
            pump(0.4); samples = []
            end = time.monotonic() + 0.4
            while time.monotonic() < end:
                pump(0.05); samples.append(abs(rpms[m]))
            row.append(sum(samples) / len(samples))
        cmd("stop", 0.7)
        print(f"{NAMES[m]:16s} {'fwd' if sign > 0 else 'rev':>4s} {pin:4d} " + " ".join(f"{v:7.1f}" for v in row), flush=True)
cmd("set open_loop 0"); cmd("stop"); ser.close()
