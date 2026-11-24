"""Bounded patrol recovery: cancel barrier, measured stop, safe heading alignment."""
import math
import time
import numpy as np
from geometry_msgs.msg import Twist
from .core import wrap, swept_clear, travel_allowed, transform, stationary_allowed_many

REAR_TURN_ENTER = math.radians(120)
REAR_TURN_EXIT = math.radians(20)
REAR_TURN_SPEED = .30


def patrol_path_heading(points, pose):
    """Look ahead 30 cm along the path from the nearest segment projection."""
    if points is None or len(points) < 2:
        return None
    starts = points[:-1]
    delta = np.diff(points, axis=0)
    lengths = np.linalg.norm(delta, axis=1)
    t = np.clip(np.sum((pose[:2]-starts)*delta, axis=1)/np.maximum(lengths**2, 1e-12), 0., 1.)
    projected = starts+delta*t[:, None]
    distances = np.linalg.norm(projected-pose[:2], axis=1)
    distances[lengths < 1e-6] = np.inf
    i = int(np.argmin(distances))
    if distances[i] > .35:
        return None
    remaining = (1.-t[i])*lengths[i]
    while remaining < .30 and i+1 < len(lengths):
        i += 1
        remaining += lengths[i]
    if remaining < .15:
        return None
    ahead = points[i+1]-delta[i]*max(0.,remaining-.30)/max(lengths[i],1e-6)
    bearing = ahead-pose[:2]
    if np.linalg.norm(bearing) < .10:
        return None
    return math.atan2(bearing[1],bearing[0])


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
        self.patrol_rear_turn = None
        self.patrol_pause_started = None
        self.patrol_recovery_check = {}
        self.patrol_sensor_wait_started = None
        self.patrol_sensor_retries = 0
        self.patrol_sensor_key = None
        self.patrol_near = None
        self.patrol_reversing_since = None
        self.patrol_wait_reason = ''
        self.patrol_obstacle_wait = False
        self.patrol_obstacle_check_time = 0.
        self.patrol_obstacle_clear_since = None
        self.patrol_obstacle_clear_count = 0
        self.patrol_obstacle_key = None
        self.planning_anchor = None
        self.patrol_path = None
        self.patrol_path_stamp = 0.
        self.patrol_recovery = None
        self.patrol_recovery_count = 0
        self.patrol_extra_clearance = 0.
        self._progress_watch = None
        self.patrol_sensor_output_wait = False
        self.patrol_progress_pose = None
        self.patrol_progress_time = time.monotonic()

    def publish_patrol_command(self, pose, command):
        """Slew normal commands, but never slew a safety stop or an unsafe arc."""
        now = time.monotonic()
        previous = getattr(self, 'patrol_output', (0., 0.))
        stamp = getattr(self, 'patrol_output_stamp', now - .05)
        dt = min(.1, max(0., now - stamp))
        v, w = float(command.linear.x), float(command.angular.z)
        # Nav2 owns the path all the way to arrival. Slow continuously without
        # changing its curvature or switching to a stop/rotate/drive controller.
        if self.nav_target is not None:
            distance = float(np.linalg.norm(pose[:2] - np.asarray(self.nav_target[:2])))
            cap = min(.4, .8 * max(0., distance - self.p['patrol_tolerance'] / 2))
            if abs(v) > cap:
                scale = cap / abs(v)
                v *= scale
                w *= scale
        v = float(np.clip(v, 0., .4))
        w = float(np.clip(w, -.35, .35))
        limited_v = previous[0] + float(np.clip(v - previous[0], -.65 * dt, .65 * dt))
        limited_w = previous[1] + float(np.clip(w - previous[1], -.8 * dt, .8 * dt))
        # Both the requested and actual output arcs must be safe: smoothing
        # changes curvature and must not carry an old command into an obstacle.
        if (not self.safe_patrol_command(pose, v, w)
                or not self.safe_patrol_command(pose, limited_v, limited_w)):
            self.stop()
            reason = getattr(self, 'patrol_command_reason', 'collision')
            self.patrol_sensor_output_wait = reason == 'sensor_unavailable'
            self.detail = '정찰 안전 정지: ' + {'lidar_collision': 'LiDAR 장애물 충돌 예상',
                'map_collision': '지도상 차체 충돌 예상', 'region_boundary': '설정 구역 경계 침범 예상',
                'sensor_unavailable': '센서 관측 준비 대기'}.get(reason, reason)
            return False
        if (hasattr(self, '_nav_sample') and time.monotonic()-self.nav_velocity_stamp >= .2):
            self.stop('nav_command_expired_during_check')
            return False
        output = Twist()
        output.linear.x, output.angular.z = limited_v, limited_w
        self.velocity_pub.publish(output)
        self.patrol_sensor_output_wait = False
        self.patrol_output = (limited_v, limited_w)
        self.patrol_output_stamp = now
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
        # One immutable observation for the entire candidate evaluation. Recheck
        # freshness afterwards; never interpret unavailable observations as walls.
        cloud = self.cloud()
        self.patrol_recovery_check = {'reason': 'sensor_unavailable', 'candidates': []}
        if cloud is None:
            return None
        cloud = cloud.copy()
        grid = self.return_grid()
        selected = None
        for heading in candidates:
            error = wrap(heading-pose[2])
            if abs(error) < .08:
                continue
            reason = 'clear'
            if not swept_clear(cloud, 0., math.copysign(.25,error),
                    self.p['footprint_half_length'], self.p['footprint_half_width'],
                    self.p['obstacle_margin'], horizon=abs(error)/.25):
                reason = 'lidar_collision'
            elif not self.motion_allowed(pose, 0., angular=math.copysign(.25,error)):
                reason = 'region_boundary'
            else:
                poses = np.array([[pose[0],pose[1],pose[2]+a]
                    for a in np.linspace(0.,error,max(2,int(abs(error)/.05)+1))])
                clear = grid.clear_many(poses) if hasattr(grid,'clear_many') else [grid.clear(p) for p in poses]
                if not all(clear):
                    reason = 'map_collision'
            self.patrol_recovery_check['candidates'].append({'heading':float(heading),'reason':reason})
            if reason == 'clear':
                selected = heading
                break
        if self.cloud() is None:
            self.patrol_recovery_check['reason'] = 'sensor_unavailable'
            return None
        self.patrol_recovery_check['reason'] = 'clear' if selected is not None else 'collision'
        return selected

    def patrol_readiness_pause(self, paused):
        now = time.monotonic()
        started = getattr(self, 'patrol_pause_started', None)
        if paused and getattr(self, '_progress_watch', None) is not None:
            self._progress_watch.last_time = now
        if paused and started is None:
            self.patrol_pause_started = now
        elif not paused and started is not None:
            # Preserve genuine active no-progress time; exclude only this pause.
            self.patrol_progress_time += now-started
            self.patrol_pause_started = None

    def on_patrol_path(self, msg):
        if self.intent != 'patrol' or self.nav_target is None or msg.header.frame_id != 'map':
            return
        points = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses])
        if (len(points) < 2 or not np.isfinite(points).all()
                or np.linalg.norm(points[-1]-self.nav_target[:2]) > .15):
            return
        self.patrol_path = points
        self.patrol_path_stamp = time.monotonic()

    def start_patrol_rear_turn(self, pose):
        if (self.nav_handle is None or self.cancel_requested or self.nav_purpose != 'patrol'
                or self.nav_target is None or np.linalg.norm(pose[:2]-self.nav_target[:2]) <= .15
                or time.monotonic()-getattr(self, 'patrol_path_stamp', 0.) > 1.5):
            return False
        heading = patrol_path_heading(self.patrol_path, pose)
        if heading is None or abs(wrap(heading-pose[2])) < REAR_TURN_ENTER:
            return False
        self.patrol_retry_target = list(self.nav_target)
        self.patrol_rear_turn = dict(phase='STOP', heading=heading, sign=None,
                                     remaining=None, next_check=0., scan_key=None)
        self.stop('patrol_rear_turn_start')
        self.cancel_nav()
        self.detail = '정찰 경로가 뒤쪽에 있어 제자리 방향 정렬을 준비합니다.'
        return True

    def rear_turn_arc_clear(self, pose, angle, cloud, grid):
        offsets = np.linspace(0., angle, max(2, math.ceil(abs(angle)/.04)+1))
        poses = np.column_stack((np.repeat([pose[:2]], len(offsets), axis=0), pose[2]+offsets))
        half_length = self.p['footprint_half_length']
        half_width = self.p['footprint_half_width']
        if not stationary_allowed_many(poses, self.data['boundary'], self._patrol_exclusion(pose),
                                       half_length, half_width).all():
            return False
        if not grid.clear_many(poses).all():
            return False
        # Account for the corners' motion between angular samples.
        margin = self.p['obstacle_margin']+math.hypot(half_length,half_width)*.02
        c, s = np.cos(offsets[:,None]), np.sin(offsets[:,None])
        x, y = cloud[:,0], cloud[:,1]
        return bool(len(cloud) and not np.any(
            (abs(c*x+s*y) <= half_length+margin) & (abs(-s*x+c*y) <= half_width+margin)))

    def tick_patrol_rear_turn(self, pose):
        turn = self.patrol_rear_turn
        if turn is None:
            return False
        now = time.monotonic()
        self.state = 'PATROL_BLOCKED' if turn['phase'] in ('BLOCKED', 'STALLED') else 'PATROL_RECOVERY'
        if self.nav_pending or self.nav_handle is not None:
            self.stop(); self.cancel_nav()
            self.detail = '뒤쪽 정찰 경로 정렬: 기존 이동 취소 완료 대기'
            return True
        if turn['phase'] == 'STALLED':
            self.stop()
            self.detail = '뒤쪽 정찰 경로 정렬 정체: 구동 상태 확인 또는 수동 조작 필요'
            return True
        if turn['phase'] != 'ROTATE':
            self.stop()
            if not self.stopped():
                return True
            difference = wrap(turn['heading']-pose[2])
            if abs(difference) <= REAR_TURN_EXIT:
                self.patrol_rear_turn = None
                self.patrol_progress_time = self.next_attempt = now
                self._progress_watch = None
                self.detail = '뒤쪽 경로 방향 정렬 완료: 같은 정찰 목표로 이동을 재개합니다.'
                return True
            if now < turn['next_check']:
                return True
            turn['next_check'] = now+.5
            cloud = self.cloud()
            key = self.scan_key() if cloud is not None else None
            if not key or not all(key) or (turn['scan_key'] is not None and
                    any(a <= b for a,b in zip(key,turn['scan_key']))):
                self.detail = '뒤쪽 경로 정렬: 새 전·후방 관측 대기'
                return True
            turn['scan_key'] = key
            angles = {1: difference % math.tau, -1: -((-difference) % math.tau)}
            # In a near tie prefer clockwise. After a stop, try the previously
            # selected direction first unless the new observation blocks it.
            preferred = turn['sign'] or (-1 if abs(difference) >= math.radians(175)
                                         else (1 if difference > 0 else -1))
            grid = self.return_grid()
            angle = next((angles[s] for s in (preferred, -preferred)
                          if self.rear_turn_arc_clear(pose,angles[s],cloud,grid)), None)
            if angle is None or self.cloud() is None:
                turn['phase'] = 'BLOCKED'
                self.detail = '뒤쪽 경로 정렬 대기: 안전한 회전 공간 재검사 중'
                return True
            turn.update(phase='ROTATE', sign=1 if angle > 0 else -1, remaining=abs(angle),
                        last_yaw=float(pose[2]), best=abs(angle), progress=now,
                        deadline=now+abs(angle)/REAR_TURN_SPEED+5.)
            self.state = 'PATROL_RECOVERY'

        delta = wrap(pose[2]-turn['last_yaw'])
        if abs(delta) > .35:
            self.stop()
            turn['phase'] = 'STOP'
            return True
        turn['last_yaw'] = float(pose[2])
        turn['remaining'] -= turn['sign']*delta
        if turn['remaining'] <= REAR_TURN_EXIT:
            self.stop()
            turn['phase'] = 'SETTLE'
            return True
        if turn['remaining'] < turn['best']-.04:
            turn.update(best=turn['remaining'], progress=now)
        if now > turn['deadline'] or now-turn['progress'] > 3.:
            self.stop()
            turn['phase'] = 'STALLED'
            return True
        w = turn['sign']*min(REAR_TURN_SPEED, .65*turn['remaining'])
        if not self.safe_patrol_command(pose,0.,w):
            self.stop()
            turn['phase'] = 'BLOCKED'
            turn['scan_key'] = self.scan_key()
            self.detail = '뒤쪽 경로 정렬 정지: 새로운 관측으로 회전 공간을 다시 확인합니다.'
            return True
        command = Twist()
        command.angular.z = w
        self.velocity_pub.publish(command)
        self.detail = ('시계' if w < 0 else '반시계')+' 방향으로 뒤쪽 정찰 경로에 정렬 중'
        return True

    def safe_patrol_command(self, pose, v, w):
        cloud = self.cloud()
        self.patrol_command_reason = 'clear'
        if cloud is None:
            self.patrol_command_reason = 'sensor_unavailable'
            return False
        if not self.motion_allowed(pose, v, angular=w):
            self.patrol_command_reason = 'region_boundary'
            return False
        if not swept_clear(cloud, v, w, self.p['footprint_half_length'],
                           self.p['footprint_half_width'], self.p['obstacle_margin']):
            self.patrol_command_reason = 'lidar_collision'
            return False
        grid = self.return_grid()  # Physical map + boundary; exclusion checked above.
        for t in np.linspace(0., 1., 15):
            a = w*t
            xy = [v*t, 0.] if abs(w)<1e-8 else [v/w*math.sin(a),v/w*(1-math.cos(a))]
            xy = transform(np.asarray([xy]),pose)[0]
            if not grid.clear([*xy,pose[2]+a]):
                self.patrol_command_reason = 'map_collision'
                return False
        if self.cloud() is None:
            self.patrol_command_reason = 'sensor_unavailable'
            return False
        return True

    def begin_patrol_recovery(self):
        self.patrol_obstacle_wait = False
        self.patrol_obstacle_clear_since = None
        self.patrol_obstacle_clear_count = 0
        self.patrol_obstacle_key = None
        self.patrol_obstacle_check_time = 0.
        self.patrol_reversing_since = None
        self.patrol_retry_target = list(self.nav_target)
        self.stop(); self.cancel_nav()
        self.planning_anchor = None
        self.patrol_recovery_count += 1
        self.patrol_sensor_wait_started = None
        self.patrol_sensor_retries = 0
        self.patrol_sensor_key = None
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

    def recheck_patrol_obstacle(self, pose):
        """Replan after three fresh, safe checks; never publish motion here.

        Called only after the mission readiness and cancellation barriers.
        Static-map, retry-budget and actuator-progress failures stay latched.
        """
        now = time.monotonic()
        if now-self.patrol_obstacle_check_time < .5:
            return
        gap = now-self.patrol_obstacle_check_time
        self.patrol_obstacle_check_time = now
        key = self.scan_key() if self.cloud() is not None else None
        fresh = (key is not None and (self.patrol_obstacle_key is None or
                 all(a>b for a,b in zip(key,self.patrol_obstacle_key))))
        if gap > 1.5 or not fresh or not self.stopped():
            self.patrol_obstacle_clear_since = None
            self.patrol_obstacle_clear_count = 0
        if not fresh or not self.stopped():
            return
        self.patrol_obstacle_key = key
        # Reuse the full footprint, LiDAR, physical-map and region checks.
        heading = self.recovery_heading(pose)
        if heading is None:
            self.patrol_obstacle_clear_since = None
            self.patrol_obstacle_clear_count = 0
            self.detail = '장애물 대기: 최신 관측으로 안전 공간을 재검사 중입니다.'
            return
        if self.patrol_obstacle_clear_since is None:
            self.patrol_obstacle_clear_since = now
        self.patrol_obstacle_clear_count += 1
        if self.patrol_obstacle_clear_count < 3 or now-self.patrol_obstacle_clear_since < 1.:
            self.detail = '장애물 해소 확인 중: 연속 관측 후 같은 목표로 경로를 재계산합니다.'
            return
        self.patrol_obstacle_wait = False
        self.patrol_recovery = None
        # A confirmed environmental change starts a new recovery episode.
        self.patrol_recovery_count = 0
        self.patrol_wait_reason = ''
        self.planning_anchor = None
        self.patrol_path = None
        self.patrol_path_stamp = 0.
        self.patrol_progress_time = now
        self._progress_watch = None
        self.next_attempt = now
        self.state = 'PATROL_SEARCH'
        self.detail = '장애물 해소 확인: 같은 정찰 목표로 새 경로를 요청합니다.'

    def tick_patrol_recovery(self, pose):
        phase = self.patrol_recovery
        if phase is None:
            return False
        self.state = 'PATROL_BLOCKED' if phase == 'WAIT' else 'PATROL_RECOVERY'
        if self.nav_pending or self.nav_handle is not None:
            self.stop(); self.cancel_nav()
            self.detail = '기존 정찰 이동 취소 완료를 기다립니다.'
            return True
        if phase == 'SENSOR_WAIT':
            self.stop()
            now = time.monotonic()
            if now-self.patrol_sensor_wait_started >= 5. or self.patrol_sensor_retries >= 3:
                self.patrol_recovery = 'WAIT'
                self.patrol_wait_reason = '복구 회전 센서 대기 한도 초과: 충돌 판정이 아닙니다. 센서 상태를 확인하세요.'
                self.detail = self.patrol_wait_reason
                return True
            cloud = self.cloud()
            key = self.scan_key() if cloud is not None else None
            if key is None or (self.patrol_sensor_key is not None and
                    any(a <= b for a,b in zip(key,self.patrol_sensor_key))):
                self.detail = self.patrol_wait_reason
                return True
            self.patrol_sensor_key = key
            self.patrol_sensor_retries += 1
            self.patrol_recovery = 'STOP'
            phase = 'STOP'
        if phase == 'WAIT':
            self.stop()
            self.detail = self.patrol_wait_reason
            if self.patrol_obstacle_wait:
                PatrolControl.recheck_patrol_obstacle(self, pose)
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
                if self.patrol_recovery_check.get('reason') == 'sensor_unavailable':
                    self.patrol_recovery = 'SENSOR_WAIT'
                    self.patrol_sensor_wait_started = self.patrol_sensor_wait_started or now
                    self.patrol_sensor_key = self.scan_key()
                    self.patrol_wait_reason = '복구 회전 센서 대기: 새 전·후방 스캔 수신 후 재검사합니다.'
                else:
                    self.patrol_recovery = 'WAIT'
                    reasons = sorted({c['reason'] for c in self.patrol_recovery_check.get('candidates',[])})
                    self.patrol_obstacle_wait = 'lidar_collision' in reasons
                    self.patrol_wait_reason = '복구 회전 충돌 검사 거부: '+', '.join(reasons)
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
            reason = getattr(self,'patrol_command_reason','collision')
            if reason == 'sensor_unavailable':
                self.patrol_recovery = 'SENSOR_WAIT'
                self.patrol_sensor_wait_started = self.patrol_sensor_wait_started or now
                self.patrol_sensor_key = self.scan_key()
                self.patrol_wait_reason = '복구 회전 센서 대기: 새 전·후방 스캔 수신 후 재검사합니다.'
                self.detail = self.patrol_wait_reason
            else:
                self.detail = '방향 정렬 충돌 검사 거부: '+reason
                if reason == 'lidar_collision':
                    self.patrol_recovery = 'WAIT'
                    self.patrol_obstacle_wait = True
                    self.patrol_obstacle_clear_since = None
                    self.patrol_obstacle_clear_count = 0
                    self.patrol_obstacle_key = None
                    self.patrol_obstacle_check_time = now
                    self.patrol_wait_reason = self.detail
            return True
        command = Twist(); command.angular.z = w
        self.velocity_pub.publish(command)
        self.detail = '구간을 침범하지 않는 경로 방향으로 제자리 정렬 중입니다.'
        return True
