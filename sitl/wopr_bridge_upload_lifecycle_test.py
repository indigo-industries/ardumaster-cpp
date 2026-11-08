"""Wire regression for upload capacity, rejected plans and staged replacement."""
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
    upload([rows[0]],5);verify(rows)
    gcs.mav.mission_count_send(7,1,65,0);m=recv(['MISSION_ACK']);assert m.type==4 and m.mission_type==0,m;verify(rows)
    def sendrow(row):gcs.mav.mission_item_int_send(7,1,row['seq'],row['frame'],row['command'],int(row['seq']==0),1,row['param1'],row['param2'],row['param3'],row['param4'],row['x'],row['y'],row['z'],0)
    gcs.mav.mission_count_send(7,1,len(rows),0);assert recv(['MISSION_REQUEST_INT']).seq==0;sendrow(rows[0]);assert recv(['MISSION_REQUEST_INT']).seq==1
    changed=dict(rows[1],z=4100);sendrow(changed);assert recv(['MISSION_REQUEST_INT']).seq==2
    verify(rows)
    sendrow(rows[3]);m=recv(['MISSION_REQUEST_INT']);assert m.seq==2,m
    upload(rows,0);verify(rows)
    print('PASS home-only rejection/oversized count/partial cache/out-of-order request/restart original')

finally:
    gcs.close();wire.close();process.terminate();process.wait(timeout=5)
