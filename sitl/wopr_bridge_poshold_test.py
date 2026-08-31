"""Phase 3 bench: qhover position hold + mission NAV_LAND precision landing.

A: hover hold — horizontal drift < 2 m over 20 s at mid-stick.
B: AUTO mission with a Land item at (n=600, e=100): transition, fly WP,
   auto-qland takeover, brake/transit, land ON the item (< 5 m).
"""
import socket
import struct
import subprocess
import time

from pymavlink import mavutil

EXE = r"C:\Docs\Code\ArduMaster\ardumaster-cpp\build\sitl\wopr_bridge.exe"
PORT = 9179
MAVPORT = 10179
MAGIC = 0x57534231
HDR = struct.Struct("<IB3xI")
INIT = struct.Struct("<iiffB3x9f260s")
STEP = struct.Struct("<H4H")
ARM = struct.Struct("<B")
ITEM = struct.Struct("<B3x5f")
REPLY = struct.Struct("<I4BI20fHHIBB2x")

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
    return {"status": r[2], "mode": r[3], "gnd": r[4], "n": r[6], "e": r[7],
            "alt": -r[8], "as": r[18], "lift": r[24], "sim_ms": r[28],
            "mi": r[26], "mc": r[27]}


def fly(seconds, rc_throttle, label="", every=0):
    st = None
    for i in range(seconds):
        st = rpc(2, STEP.pack(50, 0, 0, rc_throttle, 0))
        if every and (i % every == 0 or i == seconds - 1):
            print(f"  [{label}] t={st['sim_ms']/1000:6.1f} n={st['n']:7.1f} e={st['e']:6.1f} "
                  f"alt={st['alt']:6.1f} as={st['as']:4.1f} mode={st['mode']} mi={st['mi']} gnd={st['gnd']}")
        if st["gnd"] == 1 and label == "auto-land" and i > 5:
            break
    return st


model = rb"C:\Docs\Code\Unreal\WOPR5000\Config\SitlModels\octavian-vtol.json"
st = rpc(1, INIT.pack(348997800, -1178864000, 700.0, 0.0, 0, *([0.0] * 9), model))
assert st["status"] == 0
rpc(5, ARM.pack(1))
gcs = mavutil.mavlink_connection(f"udpout:127.0.0.1:{MAVPORT}", source_system=255)
gcs.mav.heartbeat_send(6, 8, 0, 0, 0)
assert gcs.recv_match(type="HEARTBEAT", blocking=True, timeout=6) is not None


def mode(custom):
    gcs.mav.command_long_send(1, 1, 176, 0, 1, custom, 0, 0, 0, 0, 0)
    gcs.recv_match(type="COMMAND_ACK", blocking=True, timeout=4)


# --- A: hover hold ------------------------------------------------------------
mode(18)  # QHOVER
fly(12, 1900)                      # climb ~65 m
st0 = fly(12, 1500)                # settle back onto the entry latch point
n0, e0 = st0["n"], st0["e"]
st = fly(20, 1500)
drift = ((st["n"] - n0) ** 2 + (st["e"] - e0) ** 2) ** 0.5
print(f"A hold: start ({n0:.1f},{e0:.1f}) end ({st['n']:.1f},{st['e']:.1f}) drift {drift:.2f} m over 20 s")
assert drift < 2.0, f"hover drifted {drift:.1f} m"
print("A PASS")

# --- B: mission with Land item ------------------------------------------------
items = [(0, 300.0, 0.0, 60.0, 40.0, 0.0),      # WP
         (2, 600.0, 100.0, 0.0, 0.0, 0.0)]      # LAND at (600,100)
payload = struct.pack("<H2x", len(items))
for cmd, n, e, up, acc, tp in items:
    payload += ITEM.pack(cmd, n, e, up, acc, tp)
st = rpc(4, payload)
assert st["status"] == 0, f"mission upload failed: {st['status']}"
mode(10)  # AUTO
st = fly(240, 1500, "auto-land", every=10)
err = ((st["n"] - 600.0) ** 2 + (st["e"] - 100.0) ** 2) ** 0.5
print(f"B land: touchdown ({st['n']:.1f},{st['e']:.1f}) target (600,100) error {err:.2f} m gnd={st['gnd']}")
assert st["gnd"] == 1, "never touched down"
assert err < 5.0, f"landed {err:.1f} m off target"
print("B PASS")
print("ALL PASS")
proc.kill()
