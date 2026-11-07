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
    time.sleep(.4)
    model=str(a.model.resolve()).encode()
    assert len(model)<260
    payload=struct.pack('<iiffB3x9f260s',333000000,-1122000000,3500,90,1,0,0,-500,0,160,0,0,0,math.pi/2,model)
    wire.sendto(struct.pack('<IB3xI',0x57534231,1,1)+payload,('127.0.0.1',a.port))
    reply=struct.unpack('<I4BI20fHHIBB2x',wire.recvfrom(2048)[0]);assert reply[2]==0
    recv(['HEARTBEAT'])
    model=json.loads(a.model.read_text(encoding='utf-8'))
    nav=navigation_values({'LOIT_RAD','RTL_RAD'})
    assert nav['LOIT_RAD']==model.get('loiter_radius',60),nav
    assert nav['RTL_RAD']==model.get('rtl_radius',0),nav
    rows=[]
    for i in range(4):
        rows.append(dict(seq=i,frame=0,command=16,param1=0.,param2=250.,param3=42.5 if i==1 else -7.25 if i==2 else 0.,param4=135. if i==1 else float('nan') if i==2 else -90.,x=333000000,y=-1122000000+i*100000,z=4000.))
    upload(rows,0);verify(rows)
    upload(rows,0,inject_wrong_target=True);verify(rows)
    rejected=[dict(r) for r in rows];rejected[1]['command']=17
    upload(rejected,3);verify(rows)
    # Independent plan types cannot mutate a flight upload/cache.
    for mission_type in (1,2):
        for count in (0,len(rows)):
            gcs.mav.mission_count_send(7,1,count,mission_type)
            ack=recv(['MISSION_ACK'])
            assert ack.type==3 and ack.mission_type==mission_type
            verify(rows)
    # Also reject a different plan type without aborting a partial flight upload.
    gcs.mav.mission_count_send(7,1,len(rows),0)
    assert recv(['MISSION_REQUEST_INT']).seq==0
    gcs.mav.mission_count_send(7,1,0,1)
    ack=recv(['MISSION_ACK']);assert ack.type==3 and ack.mission_type==1
    for i,row in enumerate(rows):
        gcs.mav.mission_item_int_send(7,1,i,row['frame'],row['command'],int(i==0),1,row['param1'],row['param2'],row['param3'],row['param4'],row['x'],row['y'],row['z'],0)
        m=recv(['MISSION_REQUEST_INT','MISSION_ACK'])
        if i+1<len(rows):assert m.get_type()=='MISSION_REQUEST_INT' and m.seq==i+1
        else:assert m.get_type()=='MISSION_ACK' and m.type==0
    verify(rows)
    # Empty upload must clear both the download cache and executable plan.
    gcs.mav.command_long_send(7,1,176,0,1,10,0,0,0,0,0)
    assert recv(['COMMAND_ACK']).result==0
    gcs.mav.mission_count_send(7,1,0,0)
    assert recv(['MISSION_ACK']).type==0
    gcs.mav.mission_request_list_send(7,1,0)
    assert recv(['MISSION_COUNT']).count==0, 'empty upload ACKed but download cache retained'
    wire.sendto(struct.pack('<IB3xI',0x57534231,2,2)+struct.pack('<H4H',0,0,0,0,0),('127.0.0.1',a.port))
    cleared=struct.unpack('<I4BI20fHHIBB2x',wire.recvfrom(2048)[0])
    assert cleared[27]==0, 'empty upload left executable mission loaded'
    wire.sendto(struct.pack('<IB3xI',0x57534231,2,3)+struct.pack('<H4H',1,0,0,0,0),('127.0.0.1',a.port))
    assert struct.unpack('<I4BI20fHHIBB2x',wire.recvfrom(2048)[0])[2]==0
    deadline=time.monotonic()+3
    while True:
        hb=recv(['HEARTBEAT'])
        if hb.custom_mode==11:break
        assert time.monotonic()<deadline, 'cleared AUTO did not exit to RTL'
    nav=navigation_values({'RAD_BASE','RAD_EFF','EAS2TAS','LOIT_DIR','HOME_UP','TGT_UP'})
    assert nav['HOME_UP']==0 and nav['TGT_UP']==100,nav
    expected=int(abs(model.get('rtl_radius',0)))
    if expected<=1:
        loiter=model.get('loiter_radius',60)
        expected=60 if abs(loiter)<=1 else int(abs(loiter))
    assert nav['RAD_BASE']==expected,nav
    assert abs(nav['RAD_EFF']-expected*nav['EAS2TAS']**2)<.1,nav
    direction_radius=model.get('rtl_radius',0) if int(abs(model.get('rtl_radius',0)))>1 else model.get('loiter_radius',60)
    assert nav['LOIT_DIR']==(-1 if direction_radius<0 else 1),nav
    # Non-AUTO clear must not switch pilot control mode.
    upload(rows,0)
    gcs.mav.command_long_send(7,1,176,0,1,0,0,0,0,0,0)
    assert recv(['COMMAND_ACK']).result==0
    gcs.mav.mission_count_send(7,1,0,0)
    assert recv(['MISSION_ACK']).type==0
    gcs.mav.mission_request_list_send(7,1,0)
    assert recv(['MISSION_COUNT']).count==0
    deadline=time.monotonic()+3
    while True:
        hb=recv(['HEARTBEAT'])
        if hb.custom_mode==0:break
        assert time.monotonic()<deadline, 'clear changed MANUAL mode'
    print('PASS mission metadata, target isolation, rejection and empty upload clearing')
finally:
    gcs.close();wire.close();process.terminate();process.wait(timeout=5)
