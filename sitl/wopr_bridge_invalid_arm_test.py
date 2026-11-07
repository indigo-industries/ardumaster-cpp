"""ARM_DISARM accepts exact boolean values before armed-state mutation."""
import argparse,os,socket,subprocess,struct,time,math
os.environ['MAVLINK20']='1'
from pymavlink import mavutil
p=argparse.ArgumentParser();p.add_argument('--exe',required=True);p.add_argument('--model',required=True);a=p.parse_args()
for port in (9323,10323):
 s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);s.bind(('127.0.0.1',port));s.close()
proc=subprocess.Popen([a.exe,'9323','--mavlink','10323','--sysid','7'],stdout=subprocess.DEVNULL);wire=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);wire.settimeout(3);seq=0
g=mavutil.mavlink_connection('udpout:127.0.0.1:0',source_system=255,source_component=190);g.destination_addr=('127.0.0.1',10323)
def rpc(kind,payload):
 global seq
 seq+=1;wire.sendto(struct.pack('<IB3xI',0x57534231,kind,seq)+payload,('127.0.0.1',9323));r=wire.recvfrom(2048)[0];assert r[5]==0;return r
def command(value):
 g.mav.command_long_send(7,1,176,0,1,value,0,0,0,0,0);end=time.monotonic()+2
 while time.monotonic()<end:
  m=g.recv_match(blocking=True,timeout=.05)
  if m and m.get_srcSystem()==7 and m.get_srcComponent()==1 and m.get_type()=='COMMAND_ACK' and m.command==176:return m.result
 raise TimeoutError('mode ACK')
try:
 time.sleep(.25);rpc(1,struct.pack('<iiffB3x9f260s',398260990,-840483020,3500,90,1,0,0,-500,0,160,0,0,0,math.pi/2,a.model.encode()));rpc(5,bytes([1]));rpc(4,struct.pack('<HHB3x5f',1,0,0,0,100000,500,250,0));rpc(3,bytes([4]))
 def arm_command(value):
  g.mav.command_long_send(7,1,400,0,value,0,0,0,0,0,0);end=time.monotonic()+2
  while time.monotonic()<end:
   m=g.recv_match(blocking=True,timeout=.03)
   if m and m.get_srcSystem()==7 and m.get_srcComponent()==1 and m.get_type()=='COMMAND_ACK' and m.command==400:return m.result
  raise TimeoutError('arm ACK')
 def state():return struct.unpack('<I4BI20fHHIBB2x',rpc(2,struct.pack('<H4HfB',20,1500,1500,1500,1500,200,1)))
 for initial in (1,0):
  rpc(5,bytes([initial]))
  for value in (float('nan'),float('inf'),float('-inf'),-1,.5,2):
   result=arm_command(value);after=state();assert result==4 and after[-2]==initial and after[3]==4,(initial,str(value),result,after[-2],after[3])
 for value in (0,1,1,0):
  assert arm_command(value)==0;assert state()[-2]==value
 print('invalid ARM values rejected without armed-state mutation; valid0/1 preserved')
finally:g.close();wire.close();proc.terminate();proc.wait(timeout=5)
