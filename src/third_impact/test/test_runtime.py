"""Opt-in ROS graph smoke test, isolated from robot domain 3.

Run with THIRD_IMPACT_ROS_TESTS=1; never starts a hardware node.
"""
import os
from pathlib import Path
import time

import numpy as np
import pytest

pytestmark = pytest.mark.skipif(os.environ.get('THIRD_IMPACT_ROS_TESTS') != '1', reason='Opt-in isolated ROS graph test')


def test_teaching_and_arrival(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '193')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_HOME', str(tmp_path/'ros'))
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path/'log'))
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    monkeypatch.delenv('CYCLONEDDS_URI', raising=False)
    import rclpy
    from std_msgs.msg import Empty
    from nav_msgs.msg import Odometry
    from third_impact.node import ImpactNode
    from test_core import scene
    (tmp_path/'map.pgm').write_bytes(b'P5\n1 1\n255\n\xff')
    (tmp_path/'map.yaml').write_text('image: map.pgm\nresolution: 0.03\norigin: [0, 0, 0]\n')
    rclpy.init(args=['--ros-args', '-p', 'mode:=teach', '-p', f'map_file:={tmp_path}/map.yaml',
                    '-p', f'state_file:={tmp_path}/goal.json'])
    node = None
    try:
        node = ImpactNode()
        node.healthy = lambda: True
        node.stopped = lambda: True
        node.odom = Odometry()
        node.pose = lambda: np.zeros(3)
        node.cloud = scene
        sequence = [0]
        def key():
            sequence[0] += 1
            return (sequence[0], sequence[0])
        node.scan_key = key
        node.on_save(Empty())
        assert node.state == 'CAPTURE'
        for _ in range(9):
            node.capture()
        node.capture_start -= 3.
        node.capture()
        assert node.data is not None
        assert node.data['rectangle'] is None
        node.editor.rect = [1., 2., .5, 3., 4.]
        node.on_region(Empty())
        assert node.data['rectangle'] == [1., 2., .5, 3., 4.]
        assert (tmp_path/'goal.json').is_file()
        node.publish_markers(); node.publish_status()
        arrivals = []
        subscription = node.create_subscription(Empty, '/arrival_at_goal', lambda msg: arrivals.append(msg), 10)
        node.mode = 'mission'; node.manual = False; node.intent = 'return'; node.state = 'ALIGN'
        node.p['settle_seconds'] = .01
        node.align(); time.sleep(.02); node.align()
        assert node.state == 'HOLD'
        deadline = time.monotonic()+2.
        while not arrivals and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        assert len(arrivals) == 1
        node.on_return(Empty()); node.tick()
        assert node.state == 'HOLD'
        node.on_patrol(Empty())
        assert node.intent == 'patrol'
        node.destroy_subscription(subscription)
    finally:
        if node:
            node.owner_lock.close()
            node.destroy_node()
        rclpy.shutdown()


def test_localization_publishes_seed_but_waits_for_verified_amcl(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID','193')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY','1')
    monkeypatch.setenv('ROS_HOME',str(tmp_path/'ros'))
    monkeypatch.setenv('ROS_LOG_DIR',str(tmp_path/'log'))
    monkeypatch.setenv('RMW_IMPLEMENTATION','rmw_cyclonedds_cpp')
    monkeypatch.delenv('CYCLONEDDS_URI',raising=False)
    import rclpy
    from concurrent.futures import Future
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from third_impact.localization import LocalizationBootstrap
    from test_localization_search import room,scan
    rclpy.init(args=['--ros-args','-p',f'pose_file:={tmp_path}/last_pose.json'])
    node=None
    try:
        node=LocalizationBootstrap()
        model=room(); local=[1.4,2.1,1.1]; world=model.world_pose(local)
        cloud=scan(model,local)
        node.model=model; node.cloud=lambda:cloud; node.odom=Odometry()
        node.odom_time=time.monotonic(); node.amcl_active=True; node.pose=lambda:None
        node.scans={'front':(0,1,None),'rear':(0,1,None)}
        future=Future(); future.set_result({'confident':True,'reason':'unique_match','pose':world})
        node.future=future; node.job_epoch=node.epoch; node.job_odom=np.zeros(3)
        messages=[]
        node.create_subscription(PoseWithCovarianceStamped,'/initialpose',lambda msg:messages.append(msg),10)
        node.tick()
        assert node.state=='INITIALIZING' and not node.ready
        deadline=time.monotonic()+2.
        while not messages and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
        assert len(messages)==1 and messages[0].header.frame_id=='map'
        assert not node.get_publishers_info_by_topic('/cmd_vel')
        # Emulated fresh AMCL result: readiness still needs repeated scan verification.
        node.pose=lambda:world
        for i in range(4):
            node.odom_time=time.monotonic()
            node.scans={'front':(0,i+2,None),'rear':(0,i+2,None)}
            if node.verify_since is not None: node.verify_since-=1.1
            node.tick()
        assert node.ready and node.state=='READY'
        assert (tmp_path/'last_pose.json').is_file()
    finally:
        if node:
            node.close(); node.destroy_node()
        rclpy.shutdown()
