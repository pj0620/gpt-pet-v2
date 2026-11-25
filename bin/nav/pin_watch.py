#!/usr/bin/env python3
"""Keep one motor running open-loop through the watchdog-covered host channel and flag dropouts,
for shaking wires and connectors. Run on the robot with the client launch stopped.
Usage: pin_watch.py MOTOR SIGNED_RPM [SECONDS]      e.g. pin_watch.py 2 -30 120"""
import statistics, struct, sys, time
import serial

PORT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5734047665-if00"
M = int(sys.argv[1]); RPM = float(sys.argv[2]); SECS = float(sys.argv[3]) if len(sys.argv) > 3 else 120.0
NAMES = ["L1 front-left", "L2 rear-left", "R1 front-right", "R2 rear-right"]
ser = serial.Serial(PORT, 115200, timeout=0.02); time.sleep(2.5); ser.reset_input_buffer()
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
                        v = struct.unpack("<" + "hi" * 4, bytes(payload))
                        for i in range(4): rpms[i] = v[2 * i] / 100.0
                        del buf[:27]; continue
                del buf[:1]; continue
            nl = buf.find(b"\n")
            if nl == -1:
                if len(buf) > 200: del buf[:1]
                break
            del buf[:nl + 1]

def w_frame(targets):
    payload = struct.pack("<4h", *[int(round(t * 100)) for t in targets]); x = 0
    for b in payload: x ^= b
    return b"W" + payload + bytes([x])

ser.write(b"set open_loop 1\n"); pump(0.3)
targets = [0.0] * 4; targets[M] = RPM
print(f"{NAMES[M]} at {RPM:+.0f} rpm open-loop for {SECS:.0f} s; watchdog 500 ms. Dips below half speed for >100 ms are flagged.", flush=True)
t0 = time.monotonic(); last_send = 0; last_line = 0; window = []; recent = []
dropped_since = None; dropouts = 0; steady = None
while time.monotonic() - t0 < SECS:
    now = time.monotonic()
    if now - last_send >= 0.1:
        ser.write(w_frame(targets)); last_send = now
    pump(0.02)
    r = abs(rpms[M]); window.append(r); recent.append(r)
    if now - t0 > 2.0:
        if steady is None or now - last_line >= 1.0:
            steady = statistics.median(window[-50:]) if len(window) >= 10 else steady
        if steady and steady > 3:
            low = r < 0.5 * steady
            if low and dropped_since is None: dropped_since = now
            if low and dropped_since and now - dropped_since > 0.1 and dropouts == dropouts:
                pass
            if not low and dropped_since is not None:
                if now - dropped_since > 0.1:
                    dropouts += 1
                    print(f"  RECOVERED at {now - t0:6.1f} s after {now - dropped_since:.2f} s dropout", flush=True)
                dropped_since = None
            if low and dropped_since is not None and 0.1 <= now - dropped_since < 0.13:
                print(f"  DROPOUT  at {now - t0:6.1f} s (rpm {r:.1f}, steady {steady:.1f})", flush=True)
    if now - last_line >= 1.0:
        last_line = now
        if recent:
            print(f"t={now - t0:5.1f}s rpm mean {statistics.mean(recent):5.1f} min {min(recent):5.1f}", flush=True)
        recent = []
ser.write(w_frame([0.0] * 4)); pump(0.2); ser.write(b"stop\n"); pump(0.2); ser.write(b"set open_loop 0\n"); pump(0.2); ser.close()
print(f"done: {dropouts} dropout(s) longer than 100 ms" + (" (one still in progress at the end)" if dropped_since else ""))
