"""Read-only parameter truth and system/component routing wire regression."""
import argparse,os,socket,subprocess,time
os.environ['MAVLINK20']='1'
from pymavlink import mavutil
p=argparse.ArgumentParser();p.add_argument('--exe',required=True);a=p.parse_args()
for port in (9320,10320):
 s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);s.bind(('127.0.0.1',port));s.close()
proc=subprocess.Popen([a.exe,'9320','--mavlink','10320','--sysid','7'],stdout=subprocess.DEVNULL)
g=mavutil.mavlink_connection('udpout:127.0.0.1:0',source_system=255,source_component=190);g.destination_addr=('127.0.0.1',10320)
def replies(send):
 send();rows=[];end=time.monotonic()+.35
 while time.monotonic()<end:
  m=g.recv_match(blocking=True,timeout=.03)
  if m and m.get_srcSystem()==7 and m.get_srcComponent()==1 and m.get_type()=='PARAM_VALUE':rows.append(m)
 return rows
def current(rows):
 assert len(rows)==1,rows
 m=rows[0];assert m.param_id=='SYSID_THISMAV' and m.param_value==7 and m.param_count==1 and m.param_index==0 and m.param_type==9,m
try:
 time.sleep(.25)
 current(replies(lambda:g.mav.param_request_list_send(7,1)))
 current(replies(lambda:g.mav.param_set_send(7,1,b'SYSID_THISMAV',42,9)))
 current(replies(lambda:g.mav.param_request_list_send(7,1)))
 for sys,comp in [(8,1),(7,2)]:
  assert not replies(lambda:g.mav.param_set_send(sys,comp,b'SYSID_THISMAV',43,9))
  assert not replies(lambda:g.mav.param_request_list_send(sys,comp))
 assert not replies(lambda:g.mav.param_set_send(7,1,b'UNSUPPORTED_QA',12.5,9))
 for value in [float('nan'),float('inf'),-1,0]:current(replies(lambda:g.mav.param_set_send(7,1,b'SYSID_THISMAV',value,9)))
 current(replies(lambda:g.mav.param_request_list_send(0,0)))
 current(replies(lambda:g.mav.param_set_send(0,0,b'SYSID_THISMAV',99,9)))
 print('read-only parameter truth/routing PASS')
finally:g.close();proc.terminate();proc.wait(timeout=5)
