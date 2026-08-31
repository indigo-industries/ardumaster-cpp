"""End-to-end MAVLink GCS test against wopr_bridge:
pymavlink is the ground station; the WOPR wire drives the sim clock."""
import socket
import struct
import subprocess
import threading
import time

from pymavlink import mavutil

EXE = r"C:\Docs\Code\ArduMaster\ardumaster-cpp\build\sitl\wopr_bridge.exe"
PORT = 9161
MAVPORT = 10161
MAGIC = 0x57534231

HDR = struct.Struct("<IB3xI")
INIT = struct.Struct("<iiffB3x9f260s")
STEP = struct.Struct("<H4H")
REPLY = struct.Struct("<I4BI18fHHIBB2x")

proc = subprocess.Popen([EXE, str(PORT), "--mavlink", str(MAVPORT), "--sysid", "7"],
                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
lines = []
threading.Thread(target=lambda: lines.extend(proc.stdout), daemon=True).start()
time.sleep(0.4)

wopr = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
wopr.settimeout(3.0)
seq = [0]


def wopr_rpc(t, payload):
    seq[0] += 1
    wopr.sendto(HDR.pack(MAGIC, t, seq[0]) + payload, ("127.0.0.1", PORT))
    d, _ = wopr.recvfrom(2048)
    return REPLY.unpack(d)


# Keep the sim clock running in the background (50 ticks/s wall-ish).
stop = [False]


def stepper():
    while not stop[0]:
        try:
            wopr_rpc(2, STEP.pack(10, 0, 0, 0, 0))
        except Exception:
            pass
        time.sleep(0.15)


model = rb"C:\Docs\Code\Unreal\WOPR5000\Config\SitlModels\ga-light.json"
r = wopr_rpc(1, INIT.pack(348997800, -1178864000, 700.0, 0.0, 0, *([0.0] * 9), model))
assert r[2] == 0, "INIT failed"
threading.Thread(target=stepper, daemon=True).start()

# ---- GCS side (pymavlink) ----
gcs = mavutil.mavlink_connection(f"udpout:127.0.0.1:{MAVPORT}", source_system=255)
gcs.mav.heartbeat_send(6, 8, 0, 0, 0)  # GCS heartbeat -> teaches the bridge its peer

hb = gcs.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
print("HEARTBEAT:", hb, "srcSystem=", hb.get_srcSystem() if hb else None)
assert hb is not None, "no heartbeat"
assert hb.get_srcSystem() == 7, "sysid not honored"
att = gcs.recv_match(type="ATTITUDE", blocking=True, timeout=5)
print("ATTITUDE:", att)
gpi = gcs.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
print("GLOBAL_POSITION_INT:", gpi)
hud = gcs.recv_match(type="VFR_HUD", blocking=True, timeout=5)
print("VFR_HUD:", hud)
assert att and gpi and hud, "telemetry stream incomplete"
assert abs(gpi.lat / 1e7 - 34.8998) < 0.01, "position wrong"

# ---- Mission upload (INT protocol): home + takeoff + 2 waypoints ----
wp = mavutil.mavlink.MAVLink_mission_item_int_message
items = [
    (0, 16, 0, 0.0, 348998000, -1178864000, 700.0),
    (1, 22, 3, 10.0, 349048000, -1178864000, 150.0),
    (2, 16, 3, 0.0, 349538000, -1178864000, 400.0),
    (3, 16, 3, 0.0, 349538000, -1178324000, 400.0),
]
gcs.mav.mission_count_send(7, 1, len(items))
for _ in range(len(items)):
    req = gcs.recv_match(type=["MISSION_REQUEST_INT", "MISSION_REQUEST"], blocking=True, timeout=5)
    assert req is not None, "no mission request"
    s, cmd, frame, p1, lat, lon, z = items[req.seq]
    gcs.mav.mission_item_int_send(7, 1, s, frame, cmd, 1 if s == 0 else 0, 1,
                                  p1, 0, 0, 0, lat, lon, z, 0)
ack = gcs.recv_match(type="MISSION_ACK", blocking=True, timeout=5)
print("MISSION_ACK:", ack)
assert ack is not None and ack.type == 0, "mission upload not accepted"

# ---- Arm + AUTO via COMMAND_LONG ----
gcs.mav.command_long_send(7, 1, 400, 0, 1, 0, 0, 0, 0, 0, 0)
cack = gcs.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
print("ARM ACK:", cack)
assert cack is not None and cack.result == 0
gcs.mav.command_long_send(7, 1, 176, 0, 1, 10, 0, 0, 0, 0, 0)  # custom mode 10 = AUTO
cack = gcs.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
print("MODE ACK:", cack)
assert cack is not None and cack.result == 0

# ---- Watch it fly via MAVLink only ----
t0 = time.time()
max_alt = -1
while time.time() - t0 < 45:
    m = gcs.recv_match(type=["GLOBAL_POSITION_INT", "HEARTBEAT"], blocking=True, timeout=5)
    if m is None:
        break
    if m.get_type() == "HEARTBEAT" and m.custom_mode != 10:
        print("mode changed:", m.custom_mode)
    if m.get_type() == "GLOBAL_POSITION_INT":
        max_alt = max(max_alt, m.relative_alt / 1000.0)

print(f"max relative alt seen over 45s: {max_alt:.1f} m")
stop[0] = True
time.sleep(0.3)
proc.kill()
print("PASS" if max_alt > 30 else "FAIL: did not climb")

