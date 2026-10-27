"""Stationary 2-D scan-to-map initialization. No velocity commands or ROS code.

A previous pose is only a candidate; whole-map alternatives are always searched.
Ambiguous or incomplete searches never authorize initialization.
"""
import hashlib
import math
import time
import numpy as np
from scipy.ndimage import binary_dilation, distance_transform_edt
from .core import transform, relative, register_scan, wrap, voxel


class GridModel:
    def __init__(self, cells, resolution, origin):
        self.cells = np.asarray(cells, dtype=np.int16)
        self.resolution = float(resolution)
        self.origin = np.asarray(origin, dtype=float)
        self.height, self.width = self.cells.shape
        self.free = self.cells == 0
        self.occupied = self.cells >= 65
        self.clearance = distance_transform_edt(np.pad(self.free, 1))[1:-1, 1:-1]*self.resolution
        self.distance = distance_transform_edt(~self.occupied)*self.resolution
        boundary = self.occupied & binary_dilation(self.free)
        row, col = np.nonzero(boundary)
        self.reference = np.column_stack(((col+.5)*self.resolution, (row+.5)*self.resolution))
        self.fingerprint = hashlib.sha256(self.cells.tobytes()+repr((resolution, list(origin))).encode()).hexdigest()

    def world_pose(self, local):
        xy = transform(np.asarray([local[:2]]), self.origin)[0]
        return np.array([*xy, wrap(local[2]+self.origin[2])])

    def local_pose(self, world):
        return relative(self.origin, world)

    def indices(self, points):
        ij = np.floor(np.asarray(points)/self.resolution).astype(int)
        good = (ij[..., 0]>=0) & (ij[..., 0]<self.width) & (ij[..., 1]>=0) & (ij[..., 1]<self.height)
        return np.clip(ij[..., 1], 0, self.height-1), np.clip(ij[..., 0], 0, self.width-1), good

    def scores(self, points, poses):
        """Robust endpoint cost; unknown/outside endpoints are penalized."""
        poses = np.asarray(poses)
        c, s = np.cos(poses[:, 2, None]), np.sin(poses[:, 2, None])
        x = c*points[None, :, 0]-s*points[None, :, 1]+poses[:, 0, None]
        y = s*points[None, :, 0]+c*points[None, :, 1]+poses[:, 1, None]
        row, col, good = self.indices(np.stack((x, y), axis=-1))
        distances = np.minimum(self.distance[row, col], .4)
        distances[~good | (self.cells[row, col]<0)] = .4
        # Retain 85%; a few moving people should not determine the pose.
        k = max(1, int(points.shape[0]*.85))
        return np.partition(distances, k-1, axis=1)[:, :k].mean(axis=1)

    def footprint_clear(self, pose):
        x = np.linspace(-.33, .33, 7)
        y = np.linspace(-.26, .26, 7)
        boundary = np.vstack((np.column_stack((x, x*0-.26)), np.column_stack((x, x*0+.26)),
                              np.column_stack((y*0-.33, y)), np.column_stack((y*0+.33, y)), [[0., 0.]]))
        row, col, good = self.indices(transform(boundary, pose))
        return bool(np.all(good & self.free[row, col]))

    def rays_clear(self, points, pose):
        # Wrong hypotheses often match endpoints through intervening walls.
        samples = np.vstack([points*f for f in (.25, .5, .75)])
        row, col, good = self.indices(transform(samples, pose))
        return float(np.mean(good & self.free[row, col])) >= .90

    def verify(self, points, world_pose):
        """Local geometric consistency, not a guarantee of global uniqueness."""
        pose = self.local_pose(world_pose)
        moved = transform(points, pose)
        row, col, good = self.indices(moved)
        distances = np.where(good, self.distance[row, col], 1.)
        overlap = float(np.mean(good & (self.cells[row, col]>=0) & (distances<.12)))
        score = float(self.scores(points, np.array([pose]))[0])
        return {'valid': overlap>=.70 and score<=.065 and self.footprint_clear(pose) and self.rays_clear(points, pose),
                'overlap': overlap, 'score': score}


def distinct(poses, max_count, distance=.4, angle=math.radians(18)):
    chosen = []
    for pose in poses:
        if all(np.linalg.norm(pose[:2]-p[:2])>distance or abs(wrap(pose[2]-p[2]))>angle for p in chosen):
            chosen.append(pose)
            if len(chosen) >= max_count:
                break
    return chosen


def search_map(model, points, previous_pose=None, timeout=20., cancel=None):
    started = time.monotonic()
    def interrupted():
        return (cancel is not None and cancel.is_set()) or time.monotonic()-started>timeout
    def failure(reason):
        return {'confident': False, 'reason': reason, 'elapsed': time.monotonic()-started}
    points = voxel(points, .025)
    if len(points)<80 or len(model.reference)<80:
        return failure('insufficient_geometry')
    sampled = points[np.linspace(0, len(points)-1, min(180, len(points))).astype(int)]
    step = max(1, round(.25/model.resolution))
    rows, cols = np.mgrid[0:model.height:step, 0:model.width:step]
    valid = model.clearance[rows, cols] >= .22
    xy = np.column_stack(((cols[valid]+.5)*model.resolution, (rows[valid]+.5)*model.resolution))
    if not len(xy):
        return failure('no_free_candidates')
    angles = np.arange(-math.pi, math.pi, math.radians(15))
    poses = np.column_stack((np.repeat(xy, len(angles), axis=0), np.tile(angles, len(xy))))
    if len(poses)>400000:
        return failure('map_too_large')
    scores = []
    for i in range(0, len(poses), 128):
        if interrupted(): return failure('search_interrupted')
        scores.extend(model.scores(sampled, poses[i:i+128]))
    order = np.argsort(scores)
    seeds = distinct(poses[order], 32)
    if previous_pose is not None:
        seeds.append(model.local_pose(previous_pose))
    fine = []
    offsets = np.linspace(-step*model.resolution*.75, step*model.resolution*.75, 5)
    turns = np.linspace(-math.radians(9), math.radians(9), 7)
    shifts = np.array([[x, y, a] for x in offsets for y in offsets for a in turns])
    for seed in seeds:
        if interrupted(): return failure('search_interrupted')
        candidates = seed+shifts
        costs = model.scores(sampled, candidates)
        fine.append((float(costs.min()), candidates[int(np.argmin(costs))]))
    fine.sort(key=lambda x:x[0])
    refined = []
    current = points[np.linspace(0,len(points)-1,min(700,len(points))).astype(int)]
    for seed in distinct([p for _,p in fine], 24, .25, math.radians(10)):
        if interrupted(): return failure('search_interrupted')
        result = register_scan(current, model.reference, seed, max_pair=.3)
        if result is None or result['overlap']<.65 or result['observability']<.01 or result['condition']>1000:
            continue
        local = result['pose']
        world = model.world_pose(local)
        verification = model.verify(current, world)
        if not verification['valid']:
            continue
        if any(np.linalg.norm(local[:2]-r['local'][:2])<.25 and abs(wrap(local[2]-r['local'][2]))<math.radians(10) for r in refined):
            continue
        refined.append({'pose':world, 'local':local, **verification})
    if not refined:
        return failure('no_consistent_pose')
    refined.sort(key=lambda r:r['score'])
    best = refined[0]
    runner = refined[1]['score'] if len(refined)>1 else None
    # Do not seed an equally plausible rotated/translated copy of the room.
    confident = runner is None or (runner-best['score']>max(.01, model.resolution*.25)
                                   and runner>best['score']*1.5)
    return {'confident':confident, 'reason':'unique_match' if confident else 'ambiguous',
            'pose':best['pose'], 'score':best['score'], 'overlap':best['overlap'],
            'runner_score':runner, 'candidates':len(refined),
            'alternatives':[{'pose':r['pose'].tolist(),'score':r['score'],'overlap':r['overlap']} for r in refined[:4]],
            'elapsed':time.monotonic()-started}
