#!/usr/bin/env python3
"""Pulse each motor in each direction through the ESP32's open-loop mode and report which pin moved
which wheel. Run on the robot with the client launch stopped (it owns the serial port):
  tmux send-keys -t client C-c   # wait for ros2_control_node to exit
  python3 ~/gpt-pet-v2/bin/nav/pin_test.py [PORT] [RPM] [SECONDS]
Decodes the firmware's binary 'M' reports (int16 rpm*100 + int32 counts per motor, little-endian,
XOR checksum) between text replies. Counts are in the command frame, so + means forward."""
import struct, sys, time
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5734047665-if00"
RPM = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
HOLD = float(sys.argv[3]) if len(sys.argv) > 3 else 0.6
MOTORS = [int(m) for m in sys.argv[4].split(",")] if len(sys.argv) > 4 else [0, 1, 2, 3]
PAUSE = float(sys.argv[5]) if len(sys.argv) > 5 else 0.6
NAMES = ["L1 front-left", "L2 rear-left", "R1 front-right", "R2 rear-right"]
PINS = [15, 16, 17, 18, 4, 5, 6, 7]          # motorPins in arduino_driver.ino: (positive, negative) per motor
REVERSED = [1, 1, 0, 1]                     # motor_tuning.yaml / firmware default

ser = serial.Serial(PORT, 115200, timeout=0.05)
time.sleep(2.5)                              # opening the port resets the ESP32 (DTR/RTS)
ser.reset_input_buffer()
counts = [0, 0, 0, 0]; rpms = [0.0] * 4; text = []; buf = bytearray()

def pump(secs):
    """Read for secs, decoding 'M' frames and text lines."""
    end = time.monotonic() + secs
    while time.monotonic() < end:
        buf.extend(ser.read(256))
        while buf:
            if buf[0] == ord("M"):
                if len(buf) < 27:
                    break                    # wait for the rest of a frame
                if buf[1] == 24:
                    payload, chk = buf[2:26], buf[26]
                    x = 0
                    for b in payload:
                        x ^= b
                    if x == chk:
                        vals = struct.unpack("<" + "hi" * 4, bytes(payload))
                        for i in range(4):
                            rpms[i] = vals[2 * i] / 100.0
                            counts[i] = vals[2 * i + 1]
                        del buf[:27]
                        continue
                del buf[:1]
                continue
            nl = buf.find(b"\n")
            if nl == -1:
                if len(buf) > 200:
                    del buf[:1]
                break
            line = bytes(buf[:nl]).decode("ascii", "replace").strip()
            del buf[:nl + 1]
            if line:
                text.append(line)

def cmd(s, wait=0.15):
    ser.write((s + "\n").encode())
    pump(wait)

cmd("stop"); cmd("set open_loop 1"); cmd("params", 0.3)
print("\n".join([t for t in text if t.startswith(("PARAMS", "OK", "ERR"))][-3:]))
text.clear(); pump(0.5)
print(f"\npulses of {RPM:.0f} rpm open-loop for {HOLD:.1f} s each")
print(f"{'motor':16s} {'cmd':>5s} {'pin':>4s} {'counts moved':>13s} {'peak rpm':>9s}  note")
for m in MOTORS:
    for sign in (+1, -1):
        value = -sign if REVERSED[m] else sign          # sign of the PWM the firmware applies
        pin = PINS[2 * m] if value > 0 else PINS[2 * m + 1]
        before = list(counts); peak = 0.0; text.clear()
        cmd(f"S {m} {sign * RPM:.0f}", 0.05)
        end = time.monotonic() + HOLD
        while time.monotonic() < end:
            pump(0.05); peak = max(peak, abs(rpms[m]))
        cmd("stop", PAUSE)
        moved = counts[m] - before[m]
        others = [counts[j] - before[j] for j in range(4) if j != m]
        note = "" if abs(moved) > 50 else "NO MOTION"
        if any(abs(o) > 50 for o in others):
            note += f" others moved {others}"
        print(f"{NAMES[m]:16s} {sign * RPM:+5.0f} {pin:4d} {moved:13d} {peak:9.1f}  {note}", flush=True)
cmd("set open_loop 0"); cmd("stop"); text.clear(); cmd("status", 0.3)
print("\n" + "\n".join(t for t in text if t.startswith("STATUS"))[:300])
ser.close()
