#!/usr/bin/env python3
"""Run all four motors at once, open loop through the watchdog-covered host channel, and flag per-wheel
dropouts and serial-link gaps while the harness is flexed. Phases: all forward, all backward, then
direction flips every 5 s. Run on the robot with the client launch stopped.
Usage: motor_soak.py [RPM] [PHASE_SECONDS] [FLIP_SECONDS]"""
import statistics, struct, sys, time
import serial

PORT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5734047665-if00"
RPM = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
PHASE = float(sys.argv[2]) if len(sys.argv) > 2 else 90.0
FLIP = float(sys.argv[3]) if len(sys.argv) > 3 else 60.0
NAMES = ["L1 front-left", "L2 rear-left", "R1 front-right", "R2 rear-right"]
PINS = [15, 16, 17, 18, 4, 5, 6, 7]; REVERSED = [1, 1, 0, 1]
def pin(m, sign):
    v = -sign if REVERSED[m] else sign
    return PINS[2 * m] if v > 0 else PINS[2 * m + 1]

ser = serial.Serial(PORT, 115200, timeout=0.02); time.sleep(2.5); ser.reset_input_buffer()
rpms = [0.0] * 4; buf = bytearray(); last_frame = time.monotonic(); bad = 0; frames = 0

def pump(secs):
    global last_frame, bad, frames
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
                        last_frame = time.monotonic(); frames += 1
                        del buf[:27]; continue
                    bad += 1
                del buf[:1]; continue
            nl = buf.find(b"\n")
            if nl == -1:
                if len(buf) > 200: del buf[:1]
                break
            del buf[:nl + 1]

def w_frame(t):
    p = struct.pack("<4h", *[int(round(x * 100)) for x in t]); x = 0
    for b in p: x ^= b
    return b"W" + p + bytes([x])

ser.write(b"set open_loop 1\n"); pump(0.3)
stats = {}   # (motor, sign) -> dict
def run_phase(title, sign_of, secs):
    t0 = time.monotonic(); last_send = 0; last_status = 0
    down = [None] * 4; med = [[] for _ in range(4)]; steady = [None] * 4; link_down = None
    for m in range(4):
        stats.setdefault((m, sign_of(m, 0)), {"drops": 0, "down_s": 0.0, "min": 99.0, "mean": []})
    print(f"== {title}: {secs:.0f} s", flush=True)
    while time.monotonic() - t0 < secs:
        now = time.monotonic(); el = now - t0
        targets = [sign_of(m, el) * RPM for m in range(4)]
        if now - last_send >= 0.1:
            ser.write(w_frame(targets)); last_send = now
        pump(0.02)
        if now - last_frame > 0.3 and link_down is None:
            link_down = now; print(f"  SERIAL: no report for 0.3 s at {el:5.1f} s", flush=True)
        if link_down and now - last_frame < 0.1:
            print(f"  SERIAL: back after {now - link_down:.1f} s", flush=True); link_down = None
        if el < 1.5: continue
        flipped_recently = any(sign_of(m, el) != sign_of(m, el - 0.4) for m in range(4))
        if flipped_recently:
            for m in range(4): down[m] = None   # motors pass through zero at a flip: not a fault
            continue
        healthy = [statistics.median(x) for x in med if len(x) >= 20 and statistics.median(x) >= 3]
        for m in range(4):
            s = stats[(m, sign_of(m, el))]; r = abs(rpms[m]); med[m].append(r); med[m] = med[m][-100:]
            s["mean"].append(r)
            if len(med[m]) >= 20: steady[m] = statistics.median(med[m])
            if not steady[m] or steady[m] < 3:
                # dead from the start of the phase: judge it against the other wheels (or the target)
                if el < 3.0: continue
                steady[m] = statistics.median(healthy) if healthy else RPM * 0.7
                if down[m] is None:
                    down[m] = t0 + 1.5; print(f"  DEAD {NAMES[m]} ({'fwd' if targets[m] > 0 else 'rev'}, GPIO {pin(m, 1 if targets[m] > 0 else -1)}) since the start of the phase", flush=True)
            low = r < 0.5 * steady[m]
            if low and down[m] is None: down[m] = now
            if low and down[m] and 0.1 <= now - down[m] < 0.125:
                print(f"  DROP {NAMES[m]} ({'fwd' if targets[m] > 0 else 'rev'}, GPIO {pin(m, 1 if targets[m] > 0 else -1)}) at {el:5.1f} s", flush=True)
            if not low and down[m] is not None:
                d = now - down[m]
                if d >= 0.1:
                    s["drops"] += 1; s["down_s"] += d
                    print(f"       {NAMES[m]} back after {d:.2f} s", flush=True)
                down[m] = None
                med[m] = [r] * 20   # a wheel that was dead from the start now has a real speed; stop re-declaring it dead
            s["min"] = min(s["min"], r)
        if now - last_status >= 15:
            last_status = now
            print(f"  t={el:5.1f}s rpm " + " ".join(f"{NAMES[m][:2]} {abs(rpms[m]):4.1f}" for m in range(4)) + f"  frames {frames} bad {bad}", flush=True)
    for m in range(4):   # close out a dropout still open at the end of the phase
        if down[m] is not None:
            d = time.monotonic() - down[m]; s = stats[(m, sign_of(m, secs))]; s["drops"] += 1; s["down_s"] += d
            print(f"       {NAMES[m]} still down at the end of the phase ({d:.1f} s)", flush=True)
    ser.write(w_frame([0.0] * 4)); pump(0.8)

run_phase("all forward", lambda m, el: +1, PHASE)
run_phase("all backward", lambda m, el: -1, PHASE)
run_phase("direction flip every 5 s", lambda m, el: +1 if int(el // 5) % 2 == 0 else -1, FLIP)
ser.write(b"stop\n"); pump(0.3); ser.write(b"set open_loop 0\n"); pump(0.3); ser.close()
print("\n== summary (open loop, unloaded)")
print(f"{'wheel':16s} {'dir':>4s} {'GPIO':>5s} {'dropouts':>9s} {'down s':>7s} {'min rpm':>8s} {'mean rpm':>9s}")
for (m, sign), s in sorted(stats.items()):
    mean = statistics.mean(s["mean"]) if s["mean"] else float("nan")
    print(f"{NAMES[m]:16s} {'fwd' if sign > 0 else 'rev':>4s} {pin(m, sign):5d} {s['drops']:9d} {s['down_s']:7.1f} {s['min']:8.1f} {mean:9.1f}")
print(f"serial: {frames} reports, {bad} bad checksums")
