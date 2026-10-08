"""Offline regression checks: real callbacks and packets; never opens a serial port."""
import ast,sys,struct,types,json
from pathlib import Path
from unittest.mock import patch
import numpy as np
root=Path('/home/moonshot/capstone_final_ws/src/serial_test')
sys.path.insert(0,str(root))
from serial_test.motor_driver import MotorDriver
source=(root/'serial_test/test_node_main.py').read_text()
tree=ast.parse(source)
node=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Nodelet')
methods={m.name:m for m in node.body if isinstance(m,ast.FunctionDef)}
# Execute the production joystick callback and production joystick timer branch.
joy_timer=next(n for n in methods['timer_callback'].body if isinstance(n,ast.If) and ast.unparse(n.test)=='self.JOY_CONTROL')
fn=ast.FunctionDef(name='joystick_tick',args=methods['timer_callback'].args,body=joy_timer.body,decorator_list=[])
mod=ast.fix_missing_locations(ast.Module(body=[methods['joy_callback'],methods['Lowpass_filter'],fn],type_ignores=[]))
ns={'np':np};exec(compile(mod,'<production callbacks>','exec'),ns)

def defaults(name):
 for st in methods['__init__'].body:
  if isinstance(st,ast.Assign) and any(ast.unparse(t)=='self.'+name for t in st.targets):return ast.literal_eval(st.value)
 raise AssertionError(name)

class FakeSerial:
 def __init__(self):self.packets=[];self.response=b''
 def write(self,p):self.packets.append(p)
 def read(self,size):return self.response[:size]

ser=FakeSerial()
with patch('serial_test.motor_driver.serial.Serial',return_value=ser):driver=MotorDriver()
assert defaults('w_gain')==20
for gain_idx,gain in enumerate(defaults('gain_list')):
 for name,fb,lr in [('forward',1,0),('backward',-1,0),('left',0,1),('right',0,-1),('mixed',.5,.5),('neutral',0,0)]:
  obj=types.SimpleNamespace(gain_count=gain_idx,gain_list=defaults('gain_list'),joy_lift_up_old=0,joy_lift_down_old=0,joy_speed_up_old=0,joy_speed_down_old=0,change_mode=0,v_gain=defaults('v_gain'),w_gain=defaults('w_gain'),vel_input1_old=0.,vel_input2_old=0.,md=driver,msg_wheelmotor=types.SimpleNamespace())
  obj.Lowpass_filter=types.MethodType(ns['Lowpass_filter'],obj)
  ns['joy_callback'](obj,types.SimpleNamespace(axes=[0,fb,lr,0,0,0],buttons=[0]*16))
  for _ in range(150):ns['joystick_tick'](obj)
  p=ser.packets[-1]
  assert p[:5]==bytes([183,184,1,207,7]) and sum(p)%256==0
  raw_left=struct.unpack('<h',p[6:8])[0];raw_right=struct.unpack('<h',p[9:11])[0]
  # Decode the unchanged transport convention, independent of physical polarity.
  actual_left,actual_right=-raw_left,raw_right
  actual_v=(actual_left+actual_right)/2;actual_w=(actual_right-actual_left)/2
  assert abs(actual_v-100*gain*fb)<1.1,(name,gain,actual_v)
  assert abs(actual_w-20*lr)<1.1,(name,gain,actual_w)
# Position encoding and feedback are inverses, preserving servo-lock targets.
driver.send_position_cmd(123,-456,60,60)
p=ser.packets[-1];assert p[3:5]==bytes([206,15]) and sum(p)%256==0
assert struct.unpack('<i',p[6:10])[0]==-123
assert struct.unpack('<i',p[13:17])[0]==-456
payload=struct.pack('<hhBi',-30,5,0,-123)+struct.pack('<hhBi',40,6,0,-456)
p=bytes([184,183,1,210,18])+payload;p+=bytes([(-sum(p))&255]);ser.response=p
driver.recv_motor_state()
assert (driver.rpm1,driver.rpm2,driver.pos1,driver.pos2)==(30,40,123,-456)
# Torque direction flips consistently relative to its existing convention.
driver.send_torque_cmd(12,34);p=ser.packets[-1]
assert struct.unpack('<h',p[9:11])[0]==-34 and sum(p)%256==0
assert (root/'serial_test/motor_driver.py').read_bytes()==(Path(__file__).parent/'joystick_backup_20261006/motor_driver.py').read_bytes()
print('PASS: 60 joystick/gain cases; rotation fixed at 20 for all gains; straight gain preserved; original direction, position, torque and feedback unchanged')
