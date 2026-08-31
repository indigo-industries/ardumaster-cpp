"""Model-file test for wopr_bridge.exe: fly a mission with a per-class airframe JSON."""
import math
import socket
import struct
import subprocess
import sys
import time

EXE = r"C:\Docs\Code\ArduMaster\ardumaster-cpp\build\sitl\wopr_bridge.exe"
MAGIC = 0x57534231

HDR = struct.Struct("<IB3xI")
INIT = struct.Struct("<iiffB3x9f260s")
STEP = struct.Struct("<H4H")
MODE = struct.Struct("<B")
ARM = struct.Struct("<B")
MHDR = struct.Struct("<HH")
MITEM = struct.Struct("<B3x5f")
REPLY = struct.Struct("<I4BI18fHHIBB2x")


def run_case(port, model_path, mission, sim_seconds, label):
    proc = subprocess.Popen([EXE, str(port)], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    time.sleep(0.4)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(3.0)
    addr = ("127.0.0.1", port)
    seq = [0]

    def rpc(msg_type, payload):
        seq[0] += 1
        sock.sendto(HDR.pack(MAGIC, msg_type, seq[0]) + payload, addr)
        data, _ = sock.recvfrom(2048)
        r = REPLY.unpack(data)
        assert r[0] == MAGIC and r[5] == seq[0]
        return {
            "status": r[2], "mode": r[3], "on_ground": r[4],
            "pos": r[6:9], "rpy": r[12:15], "airspeed": r[18], "hagl": r[19],
            "servo": r[20:24], "wp": r[24], "wpn": r[25], "sim_ms": r[26],
            "armed": r[27], "init": r[28],
        }

    try:
        model_bytes = model_path.encode("utf-8") if model_path else b""
        st = rpc(1, INIT.pack(348997800, -1178864000, 700.0, 0.0, 0, *([0.0] * 9), model_bytes))
        print(f"[{label}] INIT status={st['status']} init={st['init']}")
        if st["status"] != 0:
            print(f"[{label}] MODEL REFUSED — aborting case")
            return False

        st = rpc(4, MHDR.pack(len(mission), 0) + b"".join(mission))
        print(f"[{label}] MISSION status={st['status']} count={st['wpn']}")
        rpc(5, ARM.pack(1))
        st = rpc(3, MODE.pack(4))
        print(f"[{label}] AUTO status={st['status']}")

        max_roll = 0.0
        reqs = sim_seconds  # 50 ticks (1 sim-sec) per request
        for i in range(reqs):
            st = rpc(2, STEP.pack(50, 0, 0, 0, 0))
            max_roll = max(max_roll, abs(math.degrees(st["rpy"][0])))
            if i % max(1, reqs // 12) == 0 or i == reqs - 1:
                n, e, d = st["pos"]
                print(
                    f"[{label}] t={st['sim_ms']/1000.0:6.1f}s N={n:8.0f} E={e:7.0f} alt={-d:6.1f} "
                    f"as={st['airspeed']:6.1f} gnd={st['on_ground']} wp={st['wp']}/{st['wpn']} "
                    f"thr={st['servo'][3]:.2f} roll={math.degrees(st['rpy'][0]):6.1f} pitch={math.degrees(st['rpy'][1]):6.1f}"
                )
        print(f"[{label}] max|roll| seen: {max_roll:.1f} deg")
        return True
    finally:
        proc.kill()
        sock.close()


GA_MISSION = [
    MITEM.pack(1, 0.0, 0.0, 150.0, 0.0, 10.0),
    MITEM.pack(0, 8000.0, 0.0, 400.0, 0.0, 0.0),
    MITEM.pack(0, 8000.0, 5000.0, 400.0, 0.0, 0.0),
]
JET_MISSION = [
    MITEM.pack(1, 0.0, 0.0, 400.0, 0.0, 8.0),
    MITEM.pack(0, 25000.0, 0.0, 900.0, 0.0, 0.0),
    MITEM.pack(0, 25000.0, 15000.0, 900.0, 0.0, 0.0),
]

ok1 = run_case(9141, r"C:\Docs\Code\Unreal\WOPR5000\Config\SitlModels\ga-light.json", GA_MISSION, 240, "ga-light")
print()
ok2 = run_case(9142, r"C:\Docs\Code\Unreal\WOPR5000\Config\SitlModels\jet-transport.json", JET_MISSION, 360, "jet")
print()
ok3 = run_case(9143, r"C:\does\not\exist.json", GA_MISSION, 1, "bad-model")
print()
print("ga-light:", ok1, " jet:", ok2, " bad-model refused:", not ok3)
