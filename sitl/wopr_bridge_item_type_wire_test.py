"""MISSION_ITEM_INT recipient checks preserve an active upload."""
import argparse,os,socket,subprocess,struct,time,math
os.environ['MAVLINK20']='1'
from pymavlink import mavutil
p=argparse.ArgumentParser();p.add_argument('--exe',required=True);p.add_argument('--model',required=True);a=p.parse_args()
for port in (9337,10337):
 s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);s.bind(('127.0.0.1',port));s.close()
proc=subprocess.Popen([a.exe,'9337','--mavlink','10337','--sysid','7'],stdout=subprocess.DEVNULL);wire=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);wire.settimeout(3);seq=0
g=mavutil.mavlink_connection('udpout:127.0.0.1:0',source_system=255,source_component=190);g.destination_addr=('127.0.0.1',10337)
def rpc(kind,payload):
 global seq
 seq+=1;wire.sendto(struct.pack('<IB3xI',0x57534231,kind,seq)+payload,('127.0.0.1',9337));r=wire.recvfrom(2048)[0];assert r[5]==0;return r
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
 def item(seq,z=4000,mission_type=0,system=7,component=1):
  m=g.mav.mission_item_int_encode(system,component,seq,0,16,int(seq==0),1,0,250,0,0,398260990+seq*1000000,-840483020,z,mission_type)
  assert m.pack(g.mav)[1]==(38 if mission_type else 37), 'type0 extension must be truncated on wire'
  g.mav.send(m)
 for bad in (1,2,255):
  for expected in (1,2):
   g.mav.mission_count_send(7,1,3,0);assert receive(['MISSION_REQUEST_INT']).seq==0
   for seq in range(expected):
    item(seq);assert receive(['MISSION_REQUEST_INT']).seq==seq+1
   for injected in (expected,7):
    item(injected,4100,bad);m=receive(['MISSION_ACK','MISSION_REQUEST_INT']);assert m and m.get_type()=='MISSION_ACK' and m.type==3 and m.mission_type==bad,m
   item(expected,4100,bad,7,2);assert receive(['MISSION_ACK','MISSION_REQUEST_INT'],.15) is None
   item(expected,4100,bad,8,1);assert receive(['MISSION_ACK','MISSION_REQUEST_INT'],.15) is None
   for seq in range(expected,3):
    item(seq);m=receive(['MISSION_ACK','MISSION_REQUEST_INT'])
    if seq<2:assert m.get_type()=='MISSION_REQUEST_INT' and m.seq==seq+1,m
    else:assert m.get_type()=='MISSION_ACK' and m.type==0 and m.mission_type==0,m
   g.mav.mission_request_list_send(7,1,0);assert receive(['MISSION_COUNT']).count==3
   for seq in range(3):
    g.mav.mission_request_int_send(7,1,seq,0);m=receive(['MISSION_ITEM_INT']);assert m.seq==seq and m.z==4000 and m.mission_type==0,m
   g.mav.mission_ack_send(7,1,0,0)
 print('MISSION_ITEM_INT types1/2/255 ACK echo, partial continuation, wrong recipient and truncated type0 PASS')

finally:g.close();wire.close();proc.terminate();proc.wait(timeout=5)
