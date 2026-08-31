"""qhover/qland rate-tracking bench: commanded vs achieved climb/sink + collective."""
import socket
import struct
import subprocess
import time

from pymavlink import mavutil

EXE = r"C:\Docs\Code\ArduMaster\ardumaster-cpp\build\sitl\wopr_bridge.exe"
PORT = 9177
MAVPORT = 10177
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


def rpc(t, payload):
    seq[0] += 1
    wopr.sendto(HDR.pack(MAGIC, t, seq[0]) + payload, ("127.0.0.1", PORT))
    d, _ = wopr.recvfrom(2048)
    r = REPLY.unpack(d)
    return {"status": r[2], "gnd": r[4], "alt": -r[8], "vz": -r[11], "as": r[18],
            "lift": r[24], "sim_ms": r[28]}


def phase(seconds, label, rc_throttle, want=None):
    rows = []
    st = None
    for i in range(seconds):
        st = rpc(2, STEP.pack(50, 0, 0, rc_throttle, 0))
        rows.append((st["sim_ms"] / 1000.0, st["alt"], st["vz"], st["lift"]))
        if st["gnd"] == 1 and label.startswith("qland"):
            break
    print(f"--- {label} (want {want}) ---")
    for t, alt, vz, lift in rows:
        print(f"  t={t:7.1f} alt={alt:7.2f} vz={vz:+6.2f} lift={lift:5.3f}")
    if len(rows) > 6 and not label.startswith("qland"):
        tail = rows[len(rows)//2:]
        avg = sum(r[2] for r in tail) / len(tail)
        print(f"  steady vz: {avg:+.2f} (want {want})")
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


for cycle in (1, 2):
    print(f"===== CYCLE {cycle} =====")
    mode(18)  # QHOVER
    phase(12, "qhover full-stick climb", 1900, want="+6.0")
    phase(8, "qhover mid-stick hold", 1500, want="0.0")
    mode(20)  # QLAND
    st = phase(90, "qland descent", 1500, want="-2.0")
    print(f"  touchdown: gnd={st['gnd']} alt={st['alt']:.2f}")
    # settle a couple of seconds on the ground before the next cycle
    for _ in range(2):
        rpc(2, STEP.pack(50, 0, 0, 1500, 0))

print("BENCH DONE")
proc.kill()
