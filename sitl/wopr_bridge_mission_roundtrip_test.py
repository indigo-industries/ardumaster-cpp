"""Real MAVLink upload/download preserves optional NAV metadata.

Requires pymavlink. These fields round-trip; this does not implement pass-by
or yaw flight behavior. Run with --exe build/sitl/wopr_bridge.exe --model PATH.
"""
import argparse
import math
from pathlib import Path
import socket
import struct
import subprocess
import time
import os
os.environ['MAVLINK20'] = '1'
from pymavlink import mavutil

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--exe', type=Path, required=True)
parser.add_argument('--model', type=Path, required=True)
parser.add_argument('--port', type=int, default=9317)
parser.add_argument('--mavport', type=int, default=10317)
a = parser.parse_args()
process = subprocess.Popen([str(a.exe.resolve()), str(a.port), '--mavlink', str(a.mavport), '--sysid', '7'], stdout=subprocess.DEVNULL)
wire = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
wire.settimeout(3)
gcs = mavutil.mavlink_connection('udpout:127.0.0.1:0', source_system=255, source_component=190)
gcs.destination_addr = ('127.0.0.1', a.mavport)
def recv(types):
    deadline = time.monotonic()+5
    while time.monotonic()<deadline:
        gcs.mav.heartbeat_send(6, 8, 0, 0, 0)
        m = gcs.recv_match(blocking=True, timeout=.1)
        if m and m.get_srcSystem()==7 and m.get_type() in types:
            return m
    raise TimeoutError(types)
def upload(rows, expected):
    gcs.mav.mission_count_send(7, 1, len(rows), 0)
    while True:
        m = recv(['MISSION_REQUEST_INT', 'MISSION_ACK'])
        if m.get_type()=='MISSION_ACK':
            assert m.type==expected, m
            return
        r=rows[m.seq]
        gcs.mav.mission_item_int_send(7,1,m.seq,r['frame'],r['command'],int(m.seq==0),1,r['param1'],r['param2'],r['param3'],r['param4'],r['x'],r['y'],r['z'],0)
def verify(rows):
    gcs.mav.mission_request_list_send(7,1,0)
    assert recv(['MISSION_COUNT']).count==len(rows)
    for i,row in enumerate(rows):
        gcs.mav.mission_request_int_send(7,1,i,0)
        m=recv(['MISSION_ITEM_INT'])
        assert m.seq==i, m
        for key,wanted in row.items():
            got=getattr(m,key)
            assert (math.isnan(got) if isinstance(wanted,float) and math.isnan(wanted) else abs(got-wanted)<.001), (i,key,wanted,got)
    gcs.mav.mission_ack_send(7,1,0,0)
try:
    time.sleep(.4)
    model=str(a.model.resolve()).encode()
    assert len(model)<260
    payload=struct.pack('<iiffB3x9f260s',333000000,-1122000000,3500,90,1,0,0,-500,0,160,0,0,0,math.pi/2,model)
    wire.sendto(struct.pack('<IB3xI',0x57534231,1,1)+payload,('127.0.0.1',a.port))
    reply=struct.unpack('<I4BI20fHHIBB2x',wire.recvfrom(2048)[0]);assert reply[2]==0
    recv(['HEARTBEAT'])
    rows=[]
    for i in range(4):
        rows.append(dict(seq=i,frame=0,command=16,param1=0.,param2=250.,param3=42.5 if i==1 else -7.25 if i==2 else 0.,param4=135. if i==1 else float('nan') if i==2 else -90.,x=333000000,y=-1122000000+i*100000,z=4000.))
    upload(rows,0);verify(rows)
    rejected=[dict(r) for r in rows];rejected[1]['command']=17
    upload(rejected,3);verify(rows)
    print('PASS optional NAV param3/param4 including NaN round-trip and rejected-upload cache preservation')
finally:
    gcs.close();wire.close();process.terminate();process.wait(timeout=5)
