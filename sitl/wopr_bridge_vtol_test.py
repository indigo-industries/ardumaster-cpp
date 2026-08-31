"""VTOL bridge bench test: QHOVER climb -> hold -> forward transition -> QLAND."""
import socket
import struct
import subprocess
import threading
import time

from pymavlink import mavutil

EXE = r"C:\Docs\Code\ArduMaster\ardumaster-cpp\build\sitl\wopr_bridge.exe"
PORT = 9171
MAVPORT = 10171
MAGIC = 0x57534231
HDR = struct.Struct("<IB3xI")
INIT = struct.Struct("<iiffB3x9f260s")
STEP = struct.Struct("<H4H")
ARM = struct.Struct("<B")
REPLY = struct.Struct("<I4BI20fHHIBB2x")

proc = subprocess.Popen([EXE, str(PORT), "--mavlink", str(MAVPORT)],
                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
time.sleep(0.4)
wopr = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
wopr.settimeout(3.0)
seq = [0]
rc = [0, 0, 0, 0]


def rpc(t, payload):
    seq[0] += 1
    wopr.sendto(HDR.pack(MAGIC, t, seq[0]) + payload, ("127.0.0.1", PORT))
    d, _ = wopr.recvfrom(2048)
    r = REPLY.unpack(d)
    return {"status": r[2], "mode": r[3], "gnd": r[4], "pos": r[6:9], "as": r[18],
            "hagl": r[19], "lift": r[24], "hover_cmd": r[25], "sim_ms": r[28]}


model = rb"C:\Docs\Code\Unreal\WOPR5000\Config\SitlModels\octavian-vtol.json"
st = rpc(1, INIT.pack(348997800, -1178864000, 700.0, 0.0, 0, *([0.0] * 9), model))
print("INIT status:", st["status"])
assert st["status"] == 0
rpc(5, ARM.pack(1))

gcs = mavutil.mavlink_connection(f"udpout:127.0.0.1:{MAVPORT}", source_system=255)
gcs.mav.heartbeat_send(6, 8, 0, 0, 0)
hb = gcs.recv_match(type="HEARTBEAT", blocking=True, timeout=6)
print("vehicle HB:", "ok" if hb else "MISSING")
assert hb is not None


def set_mode(custom):
    gcs.mav.command_long_send(1, 1, 176, 0, 1, custom, 0, 0, 0, 0, 0)
    ack = gcs.recv_match(type="COMMAND_ACK", blocking=True, timeout=4)
    print(f"mode {custom} ack:", ack.result if ack else None)


def fly(seconds, label, rc_throttle):
    for i in range(seconds):
        st = rpc(2, STEP.pack(50, 0, 0, rc_throttle, 0))
        if i % max(1, seconds // 6) == 0 or i == seconds - 1:
            n, e, d = st["pos"]
            print(f"[{label}] t={st['sim_ms']/1000:6.1f}s alt={-d:6.2f} as={st['as']:5.1f} "
                  f"gnd={st['gnd']} mode={st['mode']} N={n:7.1f} E={e:6.1f}")
    return st


# Octavian acceptance profile (published numbers: hover climb 6, cruise 35,
# stall 18): climb ~80 m, drift-free hold, transition with <15 m sag to >30
# m/s cruise, back-transition, 2 m/s qland to touchdown.
set_mode(18)  # QHOVER
st = fly(16, "QHOVER climb", 1900)   # full stick = published 6 m/s
climb_alt = -st["pos"][2]
assert climb_alt > 60, f"hover climb too slow: {climb_alt:.1f} m in 16 s (want ~90)"
st = fly(10, "QHOVER hold", 1500)
hold_alt = -st["pos"][2]
assert abs(hold_alt - climb_alt) < 5.0, f"hold drifted {hold_alt - climb_alt:+.1f} m"

set_mode(7)   # CRUISE -> forward transition
min_alt = [hold_alt]
for i in range(100):
    st = rpc(2, STEP.pack(50, 0, 0, 1650, 0))
    min_alt[0] = min(min_alt[0], -st["pos"][2])
    if i % 12 == 0 or i == 99:
        n, e, d = st["pos"]
        print(f"[TRANSITION] t={st['sim_ms']/1000:6.1f}s alt={-d:6.2f} as={st['as']:5.1f} mode={st['mode']}")
sag = hold_alt - min_alt[0]
print(f"transition sag: {sag:.1f} m")
assert st["as"] > 30, f"cruise too slow: {st['as']:.1f} (want >30 toward 35)"
assert sag < 15, f"transition sagged {sag:.1f} m"

set_mode(18)  # back-transition
st = fly(25, "BACK-TRANSITION", 1500)
set_mode(20)  # QLAND at 2 m/s from ~hold_alt
st = fly(100, "QLAND", 1500)
print("final: alt=%.2f gnd=%d" % (-st["pos"][2], st["gnd"]))
proc.kill()
print("PASS" if st["gnd"] == 1 else "PARTIAL: airborne at end")


