"""COMMAND_INT terminal replies, routing and non-execution on the bridge wire."""
import argparse,os,socket,struct,subprocess,time,math
from pathlib import Path
os.environ['MAVLINK20']='1'
from pymavlink import mavutil
p=argparse.ArgumentParser(description=__doc__);p.add_argument('--exe',type=Path,required=True);p.add_argument('--model',type=Path,required=True);p.add_argument('--port',type=int,default=29330);p.add_argument('--mavport',type=int,default=30330);a=p.parse_args()
proc=subprocess.Popen([str(a.exe.resolve()),str(a.port),'--mavlink',str(a.mavport),'--sysid','7'],stdout=subprocess.DEVNULL)
w=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);w.settimeout(3);seq=0
g=mavutil.mavlink_connection('udpout:127.0.0.1:0',source_system=255,source_component=190);g.destination_addr=('127.0.0.1',a.mavport)
def rpc(k,payload):
 global seq
 seq+=1;w.sendto(struct.pack('<IB3xI',0x57534231,k,seq)+payload,('127.0.0.1',a.port));r=w.recvfrom(2048)[0];assert r[5]==0;return r
def state():return struct.unpack('<I4BI20fHHIBB2x',rpc(2,struct.pack('<H4HfB',0,1500,1500,1500,1500,213.4527,1)))
def preserved():
 s=state();assert s[3]==4 and s[-2]==1 and s[27]==1,('INT changed AUTO/arming/plan',s)
def collect(timeout=2,count=1):
 out=[];end=time.monotonic()+timeout
 while time.monotonic()<end:
  m=g.recv_match(blocking=True,timeout=.05)
  if m and m.get_srcSystem()==7 and m.get_srcComponent()==1 and m.get_type()=='COMMAND_ACK':
   out.append((m.command,m.result))
   if len(out)>=count:return out
 return out
def packet(cmd,ts=7,tc=1):return g.mav.command_int_encode(ts,tc,6,cmd,0,0,0 if cmd==400 else 1,6,0,0,0,0,0)
def packed(msg):
 b=msg.pack(g.mav);g.mav.seq=(g.mav.seq+1)&255;return bytes(b)
try:
 time.sleep(.3);rpc(1,struct.pack('<iiffB3x9f260s',398260990,-840483020,3500,90,1,0,0,-500,0,160,0,0,0,math.pi/2,str(a.model.resolve()).encode()));rpc(5,bytes([1]));rpc(4,struct.pack('<HHB3x5f',1,0,0,0,100000,500,250,0));rpc(3,bytes([4]));preserved()
 for cmd,result in [(176,7),(400,7),(20,7),(22,7),(178,3),(65535,3)]:
  g.mav.send(packet(cmd));assert collect()==[(cmd,result)];preserved()
 for ts,tc in [(7,0),(0,1),(0,2),(0,0)]:
  g.mav.send(packet(400,ts,tc));assert collect()==[(400,7)];preserved()
 for ts,tc in [(8,1),(8,0),(7,2)]:
  g.mav.send(packet(400,ts,tc));assert collect(.3)==[];preserved()
 # A corrupted frame must not receive a terminal ACK or execute.
 bad=bytearray(packed(packet(176)));bad[-1]^=1;g.write(bytes(bad));assert collect(.3)==[];preserved()
 # Valid INT must no longer make framing discard the following LONG in one datagram.
 long=g.mav.command_long_encode(7,1,176,0,1,10,0,0,0,0,0)
 g.write(packed(packet(65535))+packed(long));assert collect(count=2)==[(65535,3),(176,0)];preserved()
 # Routing applies per frame; ignoring one recipient must retain subsequent frames.
 g.write(packed(packet(400,7,2))+packed(long));assert collect()==[(176,0)];preserved()
 print('PASS COMMAND_INT format results/routing/broadcast/CRC/nonexecution/combined frames')
finally:g.close();w.close();proc.terminate();proc.wait(timeout=5)
