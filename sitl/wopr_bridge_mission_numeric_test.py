"""Real MAVLink mission metadata and active-upload target isolation.

Requires pymavlink. These fields round-trip; this does not implement pass-by
or yaw flight behavior. Run with --exe build/sitl/wopr_bridge.exe --model PATH.
"""
import argparse
import math
import json
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
parser.add_argument('--port', type=int, default=29328)
parser.add_argument('--mavport', type=int, default=30328)
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
        if m and m.get_srcSystem()==7 and m.get_srcComponent()==1 and m.get_type() in types:
            return m
    raise TimeoutError(types)
def navigation_values(keys):
    values={};deadline=time.monotonic()+4
    while not keys.issubset(values) and time.monotonic()<deadline:
        message=recv(['NAMED_VALUE_FLOAT'])
        values[message.name]=message.value
    assert keys.issubset(values), values
    return values
def upload(rows, expected, inject_wrong_target=False):
    gcs.mav.mission_count_send(7, 1, len(rows), 0)
    while True:
        m = recv(['MISSION_REQUEST_INT', 'MISSION_ACK'])
        if m.get_type()=='MISSION_ACK':
            assert m.type==expected, m
            return
        r=rows[m.seq]
        if inject_wrong_target and m.seq==1:
            gcs.mav.mission_item_int_send(8,1,m.seq,r['frame'],r['command'],0,1,r['param1'],r['param2'],r['param3'],r['param4'],r['x'],r['y'],r['z']+100,0)
            unexpected=gcs.recv_match(type=['MISSION_REQUEST_INT','MISSION_ACK'],blocking=True,timeout=1)
            assert unexpected is None, ('wrong target advanced active upload', unexpected)
        gcs.mav.mission_item_int_send(0 if inject_wrong_target else 7,1,m.seq,r['frame'],r['command'],int(m.seq==0),1,r['param1'],r['param2'],r['param3'],r['param4'],r['x'],r['y'],r['z'],0)
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
    time.sleep(.3)
    model=str(a.model.resolve()).encode()
    wire.sendto(struct.pack('<IB3xI',0x57534231,1,1)+struct.pack('<iiffB3x9f260s',398260990,-840483020,3500,90,1,0,0,-500,0,160,0,0,0,math.pi/2,model),('127.0.0.1',a.port))
    assert struct.unpack('<I4BI20fHHIBB2x',wire.recvfrom(2048)[0])[2]==0
    recv(['HEARTBEAT'])
    rows=[dict(seq=i,frame=0,command=16,param1=0.,param2=250.,param3=42.5,param4=float('nan'),x=398260990,y=-840483020+i*100000,z=4000.) for i in range(4)]
    upload(rows,0);verify(rows)
    cases=[('x',910000000,10),('x',-910000000,10),('y',1810000000,11),('y',-1810000000,11),('z',float('nan'),12),('z',float('inf'),12),('z',-float('inf'),12),('z',1e9,12),('z',-1e9,12)]
    for frame in (0,3,5,6):
        for field,value,expected in cases:
            bad=[dict(r) for r in rows];bad[1]['frame']=frame;bad[1][field]=value
            upload(bad,expected);verify(rows)
    # Guard float multiplication rounding at the int32 centimetre boundary.
    for value,expected in [(21474836.,12),(21474834.,0),(-21474836.,0),(-21474838.,12)]:
        boundary=[dict(r) for r in rows];boundary[1]['frame']=3;boundary[1]['z']=value
        upload(boundary,expected);verify(boundary if expected==0 else rows)
        upload(rows,0);verify(rows)
    # Finite negative and zero heights and valid coordinate endpoints remain accepted.
    for field,value in [('z',-100.),('z',0.),('x',900000000),('x',-900000000),('y',1800000000),('y',-1800000000)]:
        valid=[dict(r) for r in rows];valid[1][field]=value
        upload(valid,0);verify(valid)
    upload(rows,0);verify(rows)
    print('PASS numericmissionvalidation/rejectedcache/optionalNaNyaw/validboundaries')
finally:
    gcs.close();wire.close();process.terminate();process.wait(timeout=5)
