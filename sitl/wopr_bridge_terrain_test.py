"""Terrain-feed bench test (ticket c0faaf38): STEP tail moves the plant ground plane.

A: hover at ~90 m above start, feed terrain HAE start+50 -> hagl drops to ~40.
B: qland with that hill under it -> touchdown at ~50 m above start, on real ground.
C: tail-less STEP still accepted (wire compat) and keeps the last surface.
"""
import socket
import struct
import subprocess
import time

from pymavlink import mavutil

EXE = r"C:\Docs\Code\ArduMaster\ardumaster-cpp\build\sitl\wopr_bridge.exe"
PORT = 9173
MAVPORT = 10173
MAGIC = 0x57534231
HDR = struct.Struct("<IB3xI")
INIT = struct.Struct("<iiffB3x9f260s")
STEP = struct.Struct("<H4H")
STEP_T = struct.Struct("<H4HfB")  # + terrain tail: float HAE m, uint8 valid
ARM = struct.Struct("<B")
REPLY = struct.Struct("<I4BI20fHHIBB2x")

START_HAE = 700.0

proc = subprocess.Popen([EXE, str(PORT), "--mavlink", str(MAVPORT)],
                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
time.sleep(0.4)
wopr = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
wopr.settimeout(3.0)
seq = [0]


def rpc(t, payload):
    seq[0] += 1
    wopr.sendto(HDR.pack(MAGIC, t, seq[0]) + payload, ("127.0.0.1", PORT))
    d, _ = wopr.recvfrom(2048)
    r = REPLY.unpack(d)
    return {"status": r[2], "mode": r[3], "gnd": r[4], "pos": r[6:9], "as": r[18],
            "hagl": r[19], "lift": r[24], "sim_ms": r[28]}


def fly(seconds, label, rc_throttle, terrain=None):
    st = None
    for i in range(seconds):
        if terrain is None:
            st = rpc(2, STEP.pack(50, 0, 0, rc_throttle, 0))
        else:
            st = rpc(2, STEP_T.pack(50, 0, 0, rc_throttle, 0, terrain, 1))
        if i % max(1, seconds // 4) == 0 or i == seconds - 1:
            n, e, d = st["pos"]
            print(f"[{label}] t={st['sim_ms']/1000:6.1f}s altAboveStart={-d:6.2f} "
                  f"hagl={st['hagl']:6.2f} gnd={st['gnd']} as={st['as']:4.1f}")
    return st


model = rb"C:\Docs\Code\Unreal\WOPR5000\Config\SitlModels\octavian-vtol.json"
st = rpc(1, INIT.pack(348997800, -1178864000, START_HAE, 0.0, 0, *([0.0] * 9), model))
assert st["status"] == 0, f"INIT failed: {st['status']}"
rpc(5, ARM.pack(1))

gcs = mavutil.mavlink_connection(f"udpout:127.0.0.1:{MAVPORT}", source_system=255)
gcs.mav.heartbeat_send(6, 8, 0, 0, 0)
assert gcs.recv_match(type="HEARTBEAT", blocking=True, timeout=6) is not None
gcs.mav.command_long_send(1, 1, 176, 0, 1, 18, 0, 0, 0, 0, 0)  # QHOVER
gcs.recv_match(type="COMMAND_ACK", blocking=True, timeout=4)

st = fly(16, "climb (flat)", 1900)
alt = -st["pos"][2]
assert alt > 60, f"hover climb too slow: {alt:.1f}"
flat_hagl = st["hagl"]
assert abs(flat_hagl - alt) < 3.0, f"flat-earth hagl {flat_hagl:.1f} vs alt {alt:.1f}"

# --- A: 50 m hill slides in under the aircraft ---------------------------------
st = fly(4, "hold + hill", 1500, terrain=START_HAE + 50.0)
alt = -st["pos"][2]
expect = alt - 50.0
assert abs(st["hagl"] - expect) < 3.0, \
    f"terrain hagl {st['hagl']:.1f}, expected ~{expect:.1f} (alt {alt:.1f} over +50 hill)"
print(f"A PASS: hagl {flat_hagl:.1f} -> {st['hagl']:.1f} with a +50 m hill fed in")

# --- C: tail-less STEP keeps stepping and keeps the last surface ----------------
st = fly(3, "hold (no tail)", 1500)
alt = -st["pos"][2]
assert st["status"] == 0
assert abs(st["hagl"] - (alt - 50.0)) < 3.0, \
    f"no-tail hagl {st['hagl']:.1f} should keep last surface (~{alt - 50.0:.1f})"
print("C PASS: tail-less STEP accepted, last surface kept")

# --- B: qland lands ON the hill -------------------------------------------------
gcs.mav.command_long_send(1, 1, 176, 0, 1, 20, 0, 0, 0, 0, 0)  # QLAND
gcs.recv_match(type="COMMAND_ACK", blocking=True, timeout=4)
touchdown = None
for i in range(60):
    st = fly(1, "qland", 1500, terrain=START_HAE + 50.0)
    if st["gnd"] == 1:
        touchdown = -st["pos"][2]
        break
assert touchdown is not None, "never touched down"
assert 45.0 < touchdown < 55.0, f"touched down at {touchdown:.1f} m above start, want ~50 (the hill top)"
print(f"B PASS: qland touchdown at {touchdown:.1f} m above start — landed on the fed terrain")

print("ALL PASS")
proc.kill()
