"""Exercise actual command integration without opening a serial device."""
import ast
import time
from pathlib import Path
from types import SimpleNamespace as NS, MethodType
from unittest.mock import Mock
import numpy as np
from geometry_msgs.msg import Twist


def driver():
    tree=ast.parse((Path(__file__).parents[1]/'serial_test/test_node_main.py').read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Nodelet')
    method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='cmd_vel_callback')
    scope={'time':time,'np':np}
    exec(compile(ast.Module(body=[method],type_ignores=[]),'driver','exec'),scope)
    n=NS(last_velocity_command=time.monotonic()-.05,JOY_CONTROL=False,command_timeout=.3,
         reset_on_stop=True,md=NS(pos1=100,pos2=200,encoder_gain=1000),
         target_pos1=500.,target_pos2=600.,last_wheel_commands=(.2,.2),
         wheel_separation=.438,wheel_diameter=.17,publish_drive_debug=Mock())
    n.callback=MethodType(scope['cmd_vel_callback'],n)
    return n


def test_zero_clears_accumulated_position_demand():
    n=driver();n.callback(Twist())
    assert (n.target_pos1,n.target_pos2)==(100,200)
    assert n.last_wheel_commands==(0.,0.)
    assert n.publish_drive_debug.call_args.args[-1]=='stop_reset'


def test_wheel_reversal_discards_previous_target_once():
    n=driver();m=Twist();m.angular.z=.25
    n.callback(m)
    assert n.target_pos1<100 and n.target_pos2>200
    assert n.publish_drive_debug.call_args.args[-1]=='wheel_reversal_reset'
    left=n.target_pos1
    n.last_velocity_command=time.monotonic()-.05;n.callback(m)
    assert n.target_pos1<left
    assert n.publish_drive_debug.call_args.args[-1]=='tracking'


def test_manual_command_does_not_change_motor_targets():
    n=driver();n.JOY_CONTROL=True;n.callback(Twist())
    assert (n.target_pos1,n.target_pos2)==(500,600)
