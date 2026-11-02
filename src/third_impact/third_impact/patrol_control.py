"""Bounded patrol recovery: cancel barrier, measured stop, safe heading alignment."""
import math
import time
import numpy as np
from geometry_msgs.msg import Twist
from .core import wrap, swept_clear, travel_allowed, transform


def escape_heading(points, pose, boundary, exclusion, half_length, half_width):
    """Use a safe near-path direction, never a straight shortcut to the goal."""
    if points is None or len(points) < 2:
        return None
    nearest = int(np.argmin(np.linalg.norm(points-pose[:2], axis=1)))
    for p in points[nearest:]:
        distance = np.linalg.norm(p-pose[:2])
        if distance < .15:
            continue
        if distance > .65:
            break
        angle = math.atan2(p[1]-pose[1], p[0]-pose[0])
        if travel_allowed([*pose[:2], angle], boundary, exclusion,
                          min(distance, .3), 0., half_length, half_width):
            return angle
    return None


class PatrolControl:
    def init_patrol(self):
        self.patrol_near = None
        self.patrol_reversing_since = None
        self.patrol_wait_reason = ''
        self.planning_anchor = None
        self.patrol_path = None
        self.patrol_path_stamp = 0.
        self.patrol_recovery = None
        self.patrol_recovery_count = 0
        self.patrol_extra_clearance = 0.
        self.patrol_progress_pose = None
        self.patrol_progress_time = time.monotonic()

    def patrol_direction_stop(self, command):
        """Stop actual motion before changing turn direction; patrol only."""
        measured = self.odom.twist.twist.angular.z
        reversing = command.angular.z * measured < 0 and abs(measured) > .04 and abs(command.angular.z) > .04
        now = time.monotonic()
        if reversing and self.patrol_reversing_since is None:
            self.patrol_reversing_since = now
            self.event('정찰 회전 방향 전환: 실제 정지 후 반대 회전 허용')
        if self.patrol_reversing_since is None:
            return False
        self.stop()
        if self.stopped():
            self.patrol_reversing_since = None
        elif now-self.patrol_reversing_since > 4.:
            self.begin_patrol_recovery()
        self.detail = '정찰 회전 방향 전환: 전진·회전 정지 확인 중'
        return True

    def recovery_heading(self, pose):
        # Prefer recent path bearing. If planning failed, try a safe quarter turn
        # toward the goal, then the other side. This is rotation, not a shortcut.
        heading = None
        if time.monotonic()-self.patrol_path_stamp <= 5.:
            heading = escape_heading(self.patrol_path, pose,
                self.data['boundary'], self.data['rectangle'],
                self.p['footprint_half_length'], self.p['footprint_half_width'])
        delta = np.asarray(self.patrol_retry_target[:2])-pose[:2]
        sign = 1 if wrap(math.atan2(delta[1], delta[0])-pose[2]) >= 0 else -1
        candidates = ([] if heading is None else [heading]) + [pose[2]+sign*math.pi/2, pose[2]-sign*math.pi/2]
        for heading in candidates:
            error = wrap(heading-pose[2])
            if abs(error) < .08:
                continue
            # Check the complete rotation, then recheck live on every tick.
            cloud = self.cloud()
            if cloud is None or not swept_clear(cloud, 0., math.copysign(.25,error),
                    self.p['footprint_half_length'], self.p['footprint_half_width'],
                    self.p['obstacle_margin'], horizon=abs(error)/.25):
                continue
            if all(self.safe_patrol_command(np.array([pose[0],pose[1],pose[2]+a]),
                                           0., math.copysign(.25,error))
                   for a in np.linspace(0.,error,max(2,int(abs(error)/.05)+1))):
                return heading
        return None

    def on_patrol_path(self, msg):
        if self.intent != 'patrol' or self.nav_target is None or msg.header.frame_id != 'map':
            return
        points = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses])
        if (len(points) < 2 or not np.isfinite(points).all()
                or np.linalg.norm(points[-1]-self.nav_target[:2]) > .15):
            return
        self.patrol_path = points
        self.patrol_path_stamp = time.monotonic()

    def safe_patrol_command(self, pose, v, w):
        cloud = self.cloud()
        if (cloud is None or not self.motion_allowed(pose, v, angular=w)
                or not swept_clear(cloud, v, w, self.p['footprint_half_length'],
                                   self.p['footprint_half_width'], self.p['obstacle_margin'])):
            return False
        grid = self.return_grid()  # Physical map + boundary; exclusion checked above.
        for t in np.linspace(0., 1., 15):
            a = w*t
            xy = [v*t, 0.] if abs(w)<1e-8 else [v/w*math.sin(a),v/w*(1-math.cos(a))]
            xy = transform(np.asarray([xy]),pose)[0]
            if not grid.clear([*xy,pose[2]+a]):
                return False
        return True

    def begin_patrol_recovery(self):
        self.patrol_reversing_since = None
        self.patrol_retry_target = list(self.nav_target)
        self.stop(); self.cancel_nav()
        self.planning_anchor = None
        self.patrol_recovery_count += 1
        self.patrol_recovery_started = time.monotonic()
        self.patrol_rotation_deadline = None
        self.patrol_rotation_progress = self.patrol_recovery_started
        self.patrol_rotation_best = math.inf
        self.patrol_wait_reason = '같은 위치에서 경로 진행 실패가 반복됐습니다. 수동 위치 조정 후 정찰을 재개하세요.'
        self.patrol_heading = None
        self.patrol_recovery = 'STOP'
        if self.patrol_recovery_count >= 4:
            self.patrol_recovery = 'WAIT'
        elif self.patrol_recovery_count >= 2:
            self.patrol_extra_clearance = .15
        self.detail = '구간 차단: 현재 목표를 유지하고 정지·방향 정렬을 준비합니다.'

    def tick_patrol_recovery(self, pose):
        phase = self.patrol_recovery
        if phase is None:
            return False
        self.state = 'PATROL_BLOCKED' if phase == 'WAIT' else 'PATROL_RECOVERY'
        if self.nav_pending or self.nav_handle is not None:
            self.stop(); self.cancel_nav()
            self.detail = '기존 정찰 이동 취소 완료를 기다립니다.'
            return True
        if phase == 'WAIT':
            self.stop()
            self.detail = self.patrol_wait_reason
            return True
        now = time.monotonic()
        if phase != 'ROTATE' and now-self.patrol_recovery_started > 15.:
            self.stop(); self.patrol_recovery = 'WAIT'
            self.patrol_wait_reason = '정찰 복구의 정지·경로 준비 시간 초과: 수동 확인이 필요합니다.'
            self.detail = self.patrol_wait_reason
            return True
        if phase == 'SETTLE':
            self.stop()
            if self.stopped():
                self.patrol_recovery = None
                self.next_attempt = now+.3
                self.patrol_progress_time = now
            return True
        if phase in ('STOP', 'REPLAN'):
            self.stop()
            if not self.stopped():
                return True
            if phase == 'REPLAN':
                # publish_navigation_map runs before this method; allow costmap
                # subscribers to process the wider map before a new request.
                if now-self.patrol_recovery_started < 2.:
                    return True
                self.patrol_recovery = None
                self.next_attempt = now
                self.patrol_progress_time = now
                self.detail = '더 넓은 구간 여유로 같은 정찰 목표의 경로를 재계산합니다.'
                return True
            self.patrol_heading = self.recovery_heading(pose)
            if self.patrol_heading is None:
                self.patrol_recovery = 'WAIT'
                self.patrol_wait_reason = '제자리 복구 회전 공간 부족: 장애물·구간 경계를 확인하세요.'
                self.detail = self.patrol_wait_reason
                return True
            self.patrol_recovery = 'ROTATE'
            self.patrol_rotation_deadline = now+abs(wrap(self.patrol_heading-pose[2]))/.25+6.
            self.patrol_rotation_progress = now
            self.patrol_rotation_best = abs(wrap(self.patrol_heading-pose[2]))
        error = wrap(self.patrol_heading-pose[2])
        if abs(error) < .08:
            self.stop(); self.patrol_recovery = 'SETTLE'
            self.patrol_recovery_started = now
            self.detail = '방향 정렬 완료: 같은 정찰 목표로 이동을 재개합니다.'
            return True
        if abs(error) < self.patrol_rotation_best-.04:
            self.patrol_rotation_best = abs(error)
            self.patrol_rotation_progress = now
        if now > self.patrol_rotation_deadline or now-self.patrol_rotation_progress > 4.:
            self.stop(); self.patrol_recovery = 'WAIT'
            self.patrol_wait_reason = '정찰 복구 회전의 각도 진행 정체 또는 회전 시간 초과: 수동 확인이 필요합니다.'
            self.detail = self.patrol_wait_reason
            return True
        w = float(np.clip(error, -.25, .25))
        if not self.safe_patrol_command(pose, 0., w):
            self.stop()
            self.detail = '방향 정렬 공간 부족: 정지 상태로 대기합니다.'
            return True
        command = Twist(); command.angular.z = w
        self.velocity_pub.publish(command)
        self.detail = '구간을 침범하지 않는 경로 방향으로 제자리 정렬 중입니다.'
        return True


    def tick_patrol_near(self, pose):
        """Position-only final approach. Goal yaw is deliberately never read."""
        busy = self.nav_pending or self.nav_handle is not None
        near = self.patrol_near
        if near is None:
            if (not busy or self.nav_purpose != 'patrol' or self.cancel_requested
                    or self.nav_target is None):
                return False
            distance = np.linalg.norm(pose[:2]-np.asarray(self.nav_target[:2]))
            if distance > .40 or distance <= self.p['patrol_tolerance']:
                return False
            now = time.monotonic()
            near = dict(goal=np.asarray(self.nav_target[:2]).copy(), phase='STOP',
                        started=now, progress=now, best_distance=distance,
                        turn_sign=0, turn_deadline=None, best_angle=math.inf, turn_progress=now)
            self.patrol_near = near
            self.patrol_recovery = None
            self.stop(); self.cancel_nav()
            self.event('정찰 목표 근접: Nav2 종료 후 위치만 기준으로 저속 접근합니다.')
            return True
        self.state = 'PATROL_APPROACH'
        if self.patrol_recovery == 'WAIT':
            self.state = 'PATROL_BLOCKED'; self.stop(); self.detail = self.patrol_wait_reason
            return True
        if busy:
            self.stop(); self.cancel_nav()
            self.detail = '정찰 근접 제어 전 기존 이동 취소 완료 대기'
            return True
        now = time.monotonic()
        delta = near['goal']-pose[:2]
        distance = float(np.linalg.norm(delta))
        if distance <= self.p['patrol_tolerance']:
            self.stop(); self.patrol_visits += 1
            self.last_navigation_result = dict(purpose='patrol', status=5,
                cancel_requested=True, patrol_arrived=True)
            self.patrol_retry_target = None; self.nav_target = None
            self.init_patrol(); self.next_attempt = now
            self.event('정찰 목적지 도착: 위치 5cm 이내 확인, 다음 목적지를 선택합니다.')
            return True
        if distance > .65:
            self.stop()
            if self.stopped():
                self.patrol_retry_target = [*near['goal'], 0.]
                self.patrol_near = None; self.next_attempt = now+2.
                self.patrol_progress_time = now
            return True
        if distance < near['best_distance']-.015:
            near['best_distance'] = distance; near['progress'] = now
        error = wrap(math.atan2(delta[1],delta[0])-pose[2])
        def blocked(reason):
            self.stop(); self.patrol_recovery = 'WAIT'
            self.patrol_wait_reason = reason
            self.detail = reason; self.state = 'PATROL_BLOCKED'
        if now-near['started'] > 60.:
            blocked('정찰 목표 근접 접근 시간 초과: 목표 위치와 주변 공간을 확인하세요.')
            return True
        if near['phase'] == 'STOP':
            self.stop()
            if not self.stopped():
                if now-near['progress'] > 6.:
                    blocked('정찰 근접 제어: 실제 정지 확인 실패')
                return True
            if abs(error) > .08:
                near.update(phase='ROTATE', turn_sign=1 if error>0 else -1,
                            turn_deadline=now+abs(error)/.25+6.,
                            best_angle=abs(error), turn_progress=now)
            else:
                near.update(phase='DRIVE',progress=now)
        if near['phase'] == 'ROTATE':
            if abs(error) <= .08 or error*near['turn_sign'] < 0:
                self.stop(); near.update(phase='STOP',progress=now)
                return True
            if abs(error) < near['best_angle']-.04:
                near.update(best_angle=abs(error),turn_progress=now)
            if now > near['turn_deadline'] or now-near['turn_progress'] > 4.:
                blocked('정찰 근접 방향 전환 진행 정체 또는 시간 초과')
                return True
            w = float(np.clip(error,-.25,.25))
            if not self.safe_patrol_command(pose,0.,w):
                self.stop(); self.detail = '정찰 근접 방향 전환: 장애물 또는 구간 경계 확인 대기'
                return True
            command = Twist(); command.angular.z = w
            self.velocity_pub.publish(command)
            self.detail = '정찰 목표 위치로 이동하기 위한 방향 전환 중 (최종 heading 무관)'
            return True
        if abs(error) > .18:
            self.stop(); near.update(phase='STOP',progress=now)
            return True
        if now-near['progress'] > 6.:
            blocked('정찰 목표 근접 전진 정체: 장애물·실제 구동 상태를 확인하세요.')
            return True
        # Slow down for 5cm positional arrival, without a final yaw condition.
        remaining = max(0.,distance-self.p['patrol_tolerance']/2)
        v = min(.10, .8*remaining, math.sqrt(2*.2*remaining))
        if not self.safe_patrol_command(pose,v,0.):
            self.stop(); self.detail = '정찰 근접 전진: 장애물 또는 구간 경계 확인 대기'
            return True
        command = Twist(); command.linear.x = v
        self.velocity_pub.publish(command)
        self.detail = f'정찰 목표 위치로 저속 접근 중: 남은 거리 {distance*100:.1f}cm (최종 heading 무관)'
        return True
