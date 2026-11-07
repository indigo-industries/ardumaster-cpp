"""MISSION_COUNT routing protects active plans from other recipients."""
import argparse,os,socket,subprocess,struct,time,math
os.environ['MAVLINK20']='1'
from pymavlink import mavutil
p=argparse.ArgumentParser();p.add_argument('--exe',required=True);p.add_argument('--model',required=True);a=p.parse_args()
for port in (9321,10321):
 s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);s.bind(('127.0.0.1',port));s.close()
proc=subprocess.Popen([a.exe,'9321','--mavlink','10321','--sysid','7'],stdout=subprocess.DEVNULL);wire=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);wire.settimeout(3);seq=0
g=mavutil.mavlink_connection('udpout:127.0.0.1:0',source_system=255,source_component=190);g.destination_addr=('127.0.0.1',10321)
def rpc(kind,payload):
 global seq
 seq+=1;wire.sendto(struct.pack('<IB3xI',0x57534231,kind,seq)+payload,('127.0.0.1',9321));r=wire.recvfrom(2048)[0];assert r[5]==0;return r
def command(value):
 g.mav.command_long_send(7,1,176,0,1,value,0,0,0,0,0);end=time.monotonic()+2
 while time.monotonic()<end:
  m=g.recv_match(blocking=True,timeout=.05)
  if m and m.get_srcSystem()==7 and m.get_srcComponent()==1 and m.get_type()=='COMMAND_ACK' and m.command==176:return m.result
 raise TimeoutError('mode ACK')
try:
 time.sleep(.25);rpc(1,struct.pack('<iiffB3x9f260s',398260990,-840483020,3500,90,1,0,0,-500,0,160,0,0,0,math.pi/2,a.model.encode()));rpc(5,bytes([1]));rpc(4,struct.pack('<HHB3x5f',1,0,0,0,100000,500,250,0));rpc(3,bytes([4]))
 def count_request(system,component,count):
  g.mav.mission_count_send(system,component,count,0);end=time.monotonic()+.35;replies=[]
  while time.monotonic()<end:
   m=g.recv_match(blocking=True,timeout=.03)
   if m and m.get_srcSystem()==7 and m.get_srcComponent()==1 and m.get_type() in ('MISSION_ACK','MISSION_REQUEST_INT'):replies.append(m)
  return replies
 def state():return struct.unpack('<I4BI20fHHIBB2x',rpc(2,struct.pack('<H4HfB',20,1500,1500,1500,1500,200,1)))
 for system,component in [(7,2),(8,1)]:
  for count in (0,2):
   replies=count_request(system,component,count);assert not replies,(system,component,count,[m.to_dict() for m in replies])
   r=state();assert r[3]==4 and r[27]==1,(r[3],r[27])
 for system,component in [(7,1),(7,0),(0,0),(0,2)]:
  rpc(4,struct.pack('<HHB3x5f',1,0,0,0,100000,500,250,0));rpc(3,bytes([4]));replies=count_request(system,component,0)
  assert len(replies)==1 and replies[0].get_type()=='MISSION_ACK' and replies[0].type==0
  r=state();assert r[3]==5 and r[27]==0,(r[3],r[27])
 print('MISSION_COUNT recipient isolation and documented broadcasts PASS')
finally:g.close();wire.close();proc.terminate();proc.wait(timeout=5)
