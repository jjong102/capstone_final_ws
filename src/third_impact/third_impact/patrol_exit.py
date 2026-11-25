"""One straight, collision-checked departure from the saved patrol exclusion."""
import math
import time

import numpy as np
from geometry_msgs.msg import Twist

from .core import outside_exclusion, relative, stationary_allowed_many, wrap
from .return_route import FootprintMap

EXIT_SPEED = .14
EXIT_TURN_SPEED = .25
EXIT_MARGIN = .04
EXIT_TURN_TOLERANCE = .025
EXIT_CROSS_TRACK = .07


def straight_exit_candidates(pose, boundary, exclusion, grid, half_length, half_width):
    """Rank straight departures by clearance, then turn angle and distance."""
    padded = FootprintMap(grid.cells, grid.resolution, grid.origin,
                          half_length+EXIT_MARGIN, half_width+EXIT_MARGIN)
    extra = FootprintMap(grid.cells, grid.resolution, grid.origin,
                         half_length+.08, half_width+.08)
    step = min(.025,grid.resolution/2)
    extent = math.hypot(*exclusion[3:])+1.5
    distance = np.linspace(0.,extent,math.ceil(extent/step)+1)
    angles = [pose[2], *[exclusion[2]+math.radians(a) for a in range(0,360,5)]]
    candidates = []
    for heading in angles:
        direction = np.array([math.cos(heading),math.sin(heading)])
        points = pose[:2]+distance[:,None]*direction
        samples = np.column_stack((points,np.full(len(points),heading)))
        allowed = stationary_allowed_many(samples,boundary,exclusion,half_length,half_width)
        outside = np.flatnonzero(allowed)
        if not len(outside):
            continue
        end = np.searchsorted(distance,distance[outside[0]]+.10)
        if end >= len(distance) or not allowed[end]:
            continue
        samples = samples[:end+1]
        if (not stationary_allowed_many(samples,boundary,None,half_length,half_width).all()
                or not padded.clear(samples[-1])
                or not padded.clear_many(samples).all()):
            continue
        error = wrap(heading-pose[2])
        if not padded.rotation(pose[:2],pose[2],pose[2]+error):
            continue
        roomier = extra.clear_many(samples).all()
        candidates.append((not roomier,abs(error),distance[end],
                           {'start': pose[:2].tolist(), 'heading': float(wrap(heading)),
                            'length': float(distance[end]), 'end': points[end].tolist()}))
    return [entry for *_,entry in sorted(candidates,key=lambda item:item[:3])]


def exit_line_clear(pose, plan, cloud, half_length, half_width):
    """Reject live returns in the remaining swept rectangle of the fixed line."""
    progress, offset, angle = relative([*plan['start'],plan['heading']],pose)
    if not len(cloud):
        return False
    c, s = math.cos(angle),math.sin(angle)
    x = c*cloud[:,0]-s*cloud[:,1]+progress
    y = s*cloud[:,0]+c*cloud[:,1]+offset
    return not np.any((x >= progress-half_length-EXIT_MARGIN)
                      & (x <= plan['length']+half_length+EXIT_MARGIN)
                      & (np.abs(y) <= half_width+EXIT_MARGIN))


class PatrolExit:
    def tick_patrol_exit(self, pose):
        """Own velocity output until the robot has stopped outside the exclusion."""
        now = time.monotonic()
        self.state = 'PATROL_EXIT'
        if self.nav_pending or self.nav_handle is not None:
            self.stop()
            self.cancel_nav()
            self.detail = '제한 구역 출구: 기존 정찰 이동 취소 완료 대기'
            return
        width = self.p['footprint_half_width']
        length = self.p['footprint_half_length']
        plan = self.patrol_exit_plan
        radius = math.hypot(length,width)+.05
        outside = outside_exclusion(pose,self.data['rectangle'],radius+.06)
        if outside and (plan is None or plan['phase'] == 'DRIVE'):
            self.stop()
            if self.stopped():
                self._initial_exclusion_exit = False
                self.patrol_exit_plan = None
                self.patrol_exit_candidates = None
                self.planning_anchor = None
                self.patrol_progress_time = self.next_attempt = now
                self.event('제한 구역 직진 이탈 완료: 일반 정찰 시작')
            return
        if plan is None:
            self.stop()
            if not self.stopped() or now < self.patrol_exit_retry:
                return
            grid = self.return_grid()
            cached = self.patrol_exit_candidates
            if (cached is None or cached['grid'] is not grid
                    or np.linalg.norm(pose[:2]-cached['pose'][:2]) > .02
                    or abs(wrap(pose[2]-cached['pose'][2])) > .025):
                self.patrol_exit_candidates = dict(
                    grid=grid, pose=pose.copy(), entries=straight_exit_candidates(
                        pose,self.data['boundary'],self.data['rectangle'],grid,length,width))
                self.detail = '지도에서 직진 출구 검사 완료: 최신 LiDAR 확인 대기'
                # Geometry can take longer than scan freshness. Confirm with
                # the next control tick's pose and observation before moving.
                return
            self.patrol_exit_retry = now+1.
            cloud = self.cloud()
            if cloud is None:
                self.detail = '제한 구역 출구 검사: 최신 LiDAR 관측 대기'
                return
            for candidate in cached['entries']:
                turn = wrap(candidate['heading']-pose[2])
                if (self.rear_turn_arc_clear(pose,turn,cloud,grid)
                        and exit_line_clear(pose,candidate,cloud,length,width)
                        and self.cloud() is not None):
                    self.patrol_exit_plan = dict(candidate,phase='TURN',best=-abs(turn),
                                                 progress_at=time.monotonic(),check_grid=grid)
                    self.detail = '열린 방향 확인: 제자리 정렬 후 직진 이탈 준비'
                    return
            self.detail = '제한 구역에서 안전한 직진 출구를 찾지 못해 대기'
            return

        if self.return_grid() is not plan['check_grid']:
            self.stop()
            self.patrol_exit_plan = None
            return
        if plan['phase'] == 'STALLED':
            self.stop()
            self.detail = '제한 구역 이탈 진행 정체: 수동 확인 필요'
            return
        if plan['phase'] == 'TURN':
            error = wrap(plan['heading']-pose[2])
            if np.linalg.norm(pose[:2]-plan['start']) > EXIT_CROSS_TRACK:
                self.stop()
                self.patrol_exit_plan = None
                return
            if abs(error) <= EXIT_TURN_TOLERANCE:
                self.stop()
                if self.stopped():
                    plan.update(phase='DRIVE',best=0.,progress_at=now)
                return
            v, w = 0., float(np.clip(1.5*error,-EXIT_TURN_SPEED,EXIT_TURN_SPEED))
            progress, threshold = -abs(error), .025
        else:
            forward, lateral, angle = relative([*plan['start'],plan['heading']],pose)
            if forward >= plan['length'] or abs(lateral) > EXIT_CROSS_TRACK or abs(angle) > .15:
                self.stop()
                plan['phase'] = 'STALLED'
                return
            cloud = self.cloud()
            if cloud is None or not exit_line_clear(pose,plan,cloud,length,width):
                self.stop()
                plan['progress_at'] = now
                self.detail = '제한 구역 출구에 장애물 또는 관측 부족: 정지 대기'
                return
            v = min(EXIT_SPEED,max(.05,.6*(plan['length']-forward)))
            # Small steering corrections keep the robot on its chosen straight line.
            w = float(np.clip(-angle-1.3*lateral,-.10,.10))
            progress, threshold = forward, .02

        if not self.safe_patrol_command(pose,v,w):
            self.stop()
            plan['progress_at'] = now
            self.detail = '제한 구역 이탈 안전 검사 대기'
            return
        if progress > plan['best']+threshold:
            plan.update(best=float(progress),progress_at=now)
        if now-plan['progress_at'] > 4.:
            self.stop()
            plan['phase'] = 'STALLED'
            return
        command = Twist()
        command.linear.x, command.angular.z = v,w
        self.velocity_pub.publish(command)
        self.detail = ('열린 출구 방향으로 제자리 정렬 중' if plan['phase']=='TURN'
                       else '열린 출구를 향해 저속 직진 중')
