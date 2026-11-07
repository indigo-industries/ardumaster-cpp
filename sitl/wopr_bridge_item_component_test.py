"""MISSION_ITEM_INT recipient checks preserve an active upload."""
import argparse,os,socket,subprocess,struct,time,math
os.environ['MAVLINK20']='1'
from pymavlink import mavutil
p=argparse.ArgumentParser();p.add_argument('--exe',required=True);p.add_argument('--model',required=True);a=p.parse_args()
for port in (9322,10322):
 s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);s.bind(('127.0.0.1',port));s.close()
proc=subprocess.Popen([a.exe,'9322','--mavlink','10322','--sysid','7'],stdout=subprocess.DEVNULL);wire=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);wire.settimeout(3);seq=0
g=mavutil.mavlink_connection('udpout:127.0.0.1:0',source_system=255,source_component=190);g.destination_addr=('127.0.0.1',10322)
def rpc(kind,payload):
 global seq
 seq+=1;wire.sendto(struct.pack('<IB3xI',0x57534231,kind,seq)+payload,('127.0.0.1',9322));r=wire.recvfrom(2048)[0];assert r[5]==0;return r
def command(value):
 g.mav.command_long_send(7,1,176,0,1,value,0,0,0,0,0);end=time.monotonic()+2
 while time.monotonic()<end:
  m=g.recv_match(blocking=True,timeout=.05)
  if m and m.get_srcSystem()==7 and m.get_srcComponent()==1 and m.get_type()=='COMMAND_ACK' and m.command==176:return m.result
 raise TimeoutError('mode ACK')
try:
 time.sleep(.25);rpc(1,struct.pack('<iiffB3x9f260s',398260990,-840483020,3500,90,1,0,0,-500,0,160,0,0,0,math.pi/2,a.model.encode()));rpc(5,bytes([1]));rpc(4,struct.pack('<HHB3x5f',1,0,0,0,100000,500,250,0));rpc(3,bytes([4]))
 def receive(kind,timeout=2):
  end=time.monotonic()+timeout
  while time.monotonic()<end:
   m=g.recv_match(blocking=True,timeout=.03)
   if m and m.get_srcSystem()==7 and m.get_srcComponent()==1 and m.get_type() in kind:return m
  return None
 def item(system,component,seq,z):g.mav.mission_item_int_send(system,component,seq,0,16,int(seq==0),1,0,250,0,0,398260990+seq*1000000,-840483020,z,0)
 for system,component in [(7,1),(7,0),(0,0),(0,2)]:
  g.mav.mission_count_send(7,1,3,0);assert receive(['MISSION_REQUEST_INT']).seq==0
  item(7,1,0,4000);assert receive(['MISSION_REQUEST_INT']).seq==1
  item(7,2,1,4100);unexpected=receive(['MISSION_REQUEST_INT','MISSION_ACK'],.35);assert unexpected is None,unexpected
  item(system,component,1,4000);assert receive(['MISSION_REQUEST_INT']).seq==2
  item(7,1,2,4000);assert receive(['MISSION_ACK']).type==0
  g.mav.mission_request_list_send(7,1,0);assert receive(['MISSION_COUNT']).count==3
  g.mav.mission_request_int_send(7,1,1,0);row=receive(['MISSION_ITEM_INT']);assert row.z==4000,row
  g.mav.mission_ack_send(7,1,0,0)
 print('MISSION_ITEM_INT component isolation and valid broadcast rows PASS')
finally:g.close();wire.close();proc.terminate();proc.wait(timeout=5)
