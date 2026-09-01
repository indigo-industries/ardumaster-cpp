"""TECS internals trace for the jet-transport model mission."""
import math
import socket
import struct
import subprocess
import time

EXE = r"C:\Docs\Code\ArduMaster\ardumaster-cpp\build\sitl\wopr_bridge.exe"
MAGIC = 0x57534231
PORT = 9151

HDR = struct.Struct("<IB3xI")
INIT = struct.Struct("<iiffB3x9f260s")
STEP = struct.Struct("<H4H")
MODE = struct.Struct("<B")
ARM = struct.Struct("<B")
MHDR = struct.Struct("<HH")
MITEM = struct.Struct("<B3x5f")
REPLY = struct.Struct("<I4BI20fHHIBB2x")

import threading

proc = subprocess.Popen([EXE, str(PORT), "--debug"], stdout=subprocess.PIPE,
                        stderr=subprocess.STDOUT, text=True)
dbg_lines = []
threading.Thread(target=lambda: dbg_lines.extend(proc.stdout), daemon=True).start()
time.sleep(0.4)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(3.0)
addr = ("127.0.0.1", PORT)
seq = 0


def rpc(t, payload):
    global seq
    seq += 1
    sock.sendto(HDR.pack(MAGIC, t, seq) + payload, addr)
    data, _ = sock.recvfrom(2048)
    r = REPLY.unpack(data)
    return {"status": r[2], "pos": r[6:9], "airspeed": r[18], "wp": r[26], "sim_ms": r[28]}


try:
    model = rb"C:\Docs\Code\Unreal\WOPR5000\Config\SitlModels\jet-transport.json"
    st = rpc(1, INIT.pack(348997800, -1178864000, 700.0, 0.0, 0, *([0.0] * 9), model))
    assert st["status"] == 0, st
    items = [
        MITEM.pack(1, 0.0, 0.0, 400.0, 0.0, 8.0),
        MITEM.pack(0, 25000.0, 0.0, 900.0, 0.0, 0.0),
        MITEM.pack(0, 25000.0, 15000.0, 900.0, 0.0, 0.0),
    ]
    rpc(4, MHDR.pack(len(items), 0) + b"".join(items))
    rpc(5, ARM.pack(1))
    rpc(3, MODE.pack(4))
    for i in range(300):  # 300 sim-seconds
        st = rpc(2, STEP.pack(50, 0, 0, 0, 0))
        if i % 25 == 0:
            n, e, d = st["pos"]
            print(f"HOST t={st['sim_ms']/1000:6.1f} alt={-d:6.1f} as={st['airspeed']:5.1f} wp={st['wp']}")
finally:
    proc.kill()
    time.sleep(0.3)
    print("".join(dbg_lines))
    sock.close()
