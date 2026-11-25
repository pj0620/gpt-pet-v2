#!/usr/bin/env python3
"""Alternate forward/backward open-loop pulses on ONE motor without a break, so a hand braking the
wheel applies matched pressure to both directions; prints counts per pulse and the rev/fwd ratio.
Run on the robot with the client launch stopped. Usage: pin_alternate.py MOTOR [RPM] [SECONDS] [PAIRS]"""
import struct, sys, time
import serial

PORT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5734047665-if00"
M = int(sys.argv[1]); RPM = float(sys.argv[2]) if len(sys.argv) > 2 else 45.0
HOLD = float(sys.argv[3]) if len(sys.argv) > 3 else 1.5; PAIRS = int(sys.argv[4]) if len(sys.argv) > 4 else 3
NAMES = ["L1 front-left", "L2 rear-left", "R1 front-right", "R2 rear-right"]
ser = serial.Serial(PORT, 115200, timeout=0.05); time.sleep(2.5); ser.reset_input_buffer()
counts = [0] * 4; rpms = [0.0] * 4; buf = bytearray()
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
                        v = struct.unpack("<" + "hi" * 4, bytes(payload))
                        for i in range(4): rpms[i] = v[2 * i] / 100.0; counts[i] = v[2 * i + 1]
                        del buf[:27]; continue
                del buf[:1]; continue
            nl = buf.find(b"\n")
            if nl == -1:
                if len(buf) > 200: del buf[:1]
                break
            del buf[:nl + 1]
def cmd(s, wait=0.15): ser.write((s + "\n").encode()); pump(wait)
cmd("stop"); cmd("set open_loop 1"); pump(0.3)
print(f"{NAMES[M]}: {PAIRS} x (fwd, rev) pulses of {HOLD:.1f} s at {RPM:.0f} rpm open-loop (PWM {round(RPM / 76 * 255)})")
fwd, rev = [], []
for k in range(PAIRS):
    for sign, store in ((+1, fwd), (-1, rev)):
        before = counts[M]; cmd(f"S {M} {sign * RPM:.0f}", 0.05)
        end = time.monotonic() + HOLD
        while time.monotonic() < end: pump(0.05)
        cmd("stop", 0.4); store.append(abs(counts[M] - before))
        print(f"  pulse {2 * k + 1 + (sign < 0)}: {'fwd' if sign > 0 else 'rev'} {store[-1]:6d} counts", flush=True)
cmd("set open_loop 0"); cmd("stop"); ser.close()
mf, mr = sum(fwd) / len(fwd), sum(rev) / len(rev)
print(f"mean fwd {mf:.0f}, mean rev {mr:.0f}, rev/fwd = {mr / mf if mf else float('nan'):.2f}")
