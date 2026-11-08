"""Read-only TECS reference telemetry: local units, slew, and held FBWA values."""
import argparse,json,math,os,pathlib,socket,struct,subprocess,time
os.environ['MAVLINK20']='1'
from pymavlink import mavutil
p=argparse.ArgumentParser();p.add_argument('--exe',required=True);p.add_argument('--port',type=int,default=29359);p.add_argument('--mavport',type=int,default=30359);p.add_argument('--out');a=p.parse_args()
log=pathlib.Path(a.out).open('w',encoding='utf-8') if a.out else None
proc=subprocess.Popen([a.exe,str(a.port),'--mavlink',str(a.mavport),'--sysid','7'],stdout=subprocess.DEVNULL)
w=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);w.settimeout(3);g=mavutil.mavlink_connection('udpout:127.0.0.1:0',source_system=255,source_component=190);g.destination_addr=('127.0.0.1',a.mavport)
R=struct.Struct('<I4BI20fHHIBB2x');seq=0

def record(kind,data):
 if log:log.write(json.dumps({'wall':time.monotonic(),'kind':kind,'data':data})+'\n');log.flush()
def rpc(kind,payload):
 global seq
 seq+=1;record('udp_tx',{'kind':kind,'seq':seq,'hex':payload.hex()});w.sendto(struct.pack('<IB3xI',0x57534231,kind,seq)+payload,('127.0.0.1',a.port));b=w.recvfrom(2048)[0];v=R.unpack(b);record('udp_rx',{'hex':b.hex(),'simMs':v[28]});assert v[0]==0x57534231 and v[2]==0 and v[5]==seq;return v

def step(n):return rpc(2,struct.pack('<H4HfB',n,1500,1500,1500,1500,213.4527,1))
def advance(n):
 while n:take=min(64,n);v=step(take);n-=take
 return v

def load(up):return rpc(4,struct.pack('<HHB3x5f',1,0,0,0,100000,up,250,0))
def collect(target,min_ms):
 groups={};raw=None;end=time.monotonic()+4;next_step=0;latest=0
 while time.monotonic()<end:
  if time.monotonic()>=next_step:latest=step(1)[28];next_step=time.monotonic()+.12;g.mav.heartbeat_send(6,8,0,0,0)
  m=g.recv_match(blocking=True,timeout=.02)
  if not m:continue
  record('mav_rx',{'srcSystem':m.get_srcSystem(),'srcComponent':m.get_srcComponent(),'payload':m.to_dict(),'hex':m.get_msgbuf().hex()})
  if m.get_srcSystem()!=7 or m.get_srcComponent()!=1 or m.get_type()!='NAMED_VALUE_FLOAT':continue
  if m.name=='TGT_UP' and abs(m.value-target)<.02:raw=m.value
  if m.name in ('HDM_IN','HDM_LTD','HDM_ACT'):
   groups.setdefault(m.time_boot_ms,{})[m.name]=m.value
   for boot,values in groups.items():
    if boot>=min_ms and boot==latest and len(values)==3 and raw is not None:
     assert all(math.isfinite(x) for x in values.values());return {'simMs':boot,'target':raw,**values}
 raise AssertionError({'missingReferenceGroup':True,'groups':groups,'target':raw,'minMs':min_ms})
try:
 time.sleep(.25);rpc(1,struct.pack('<iiffB3x9f260s',398260990,-840483020,217.044,90,1,0,0,-500,0,17,0,0,0,math.pi/2,b''));rpc(5,bytes([1]));load(500);rpc(3,bytes([4]));v=advance(100);before=collect(500,v[28]);assert all(abs(before[x]-500)<.1 for x in ('HDM_IN','HDM_LTD','HDM_ACT')),before
 load(600);v=advance(50);after=collect(600,v[28]);elapsed=(after['simMs']-before['simMs'])/1000;assert after['HDM_LTD']<=before['HDM_LTD']+5*elapsed+.15,(before,after);assert after['HDM_ACT']<575,after
 rpc(3,bytes([1]));v=advance(50);held=collect(600,v[28]);assert held['simMs']-after['simMs']>=1000;assert all(struct.pack('<f',held[k])==struct.pack('<f',after[k]) for k in ('HDM_IN','HDM_LTD','HDM_ACT')),(after,held)
 print(json.dumps({'passed':True,'before':before,'after':after,'heldFbwa':held,'scope':'default plant, configured defaultCLMB5; local metres, timestamps observationtime notTECSfreshness'}))
finally:
 g.close();w.close();proc.terminate();proc.wait(timeout=5)
 if log:log.close()
