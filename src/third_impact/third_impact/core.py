"""ROS-independent geometry, registration, persistence and navigation policies."""
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile

import numpy as np
from scipy.spatial import cKDTree
import yaml


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def transform(points, pose):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return np.asarray(points) @ np.array([[c, s], [-s, c]]) + pose[:2]


def relative(origin, pose):
    c, s = math.cos(origin[2]), math.sin(origin[2])
    dx, dy = pose[0]-origin[0], pose[1]-origin[1]
    return np.array([c*dx+s*dy, -s*dx+c*dy, wrap(pose[2]-origin[2])])


def inside_rectangle(point, rect):
    local = relative(rect[:3], [point[0], point[1], 0.])
    return abs(local[0]) <= rect[3]/2 and abs(local[1]) <= rect[4]/2


def map_fingerprint(path):
    path = Path(path).expanduser().resolve()
    config = yaml.safe_load(path.read_text())
    image = (path.parent / config.pop('image')).resolve()
    digest = hashlib.sha256(json.dumps(config, sort_keys=True).encode())
    digest.update(image.read_bytes())
    return digest.hexdigest()


def atomic_save(path, data):
    path = Path(path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=path.parent, prefix='.goal-')
    try:
        with os.fdopen(fd, 'w') as f:
            json.dump(data, f, allow_nan=False)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
    finally:
        if os.path.exists(tmp):
            os.unlink(tmp)


def voxel(points, size=0.015):
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    _, indices = np.unique(np.floor(points/size).astype(int), axis=0, return_index=True)
    return points[indices]


def register_scan(current, reference, initial, max_pair=0.15):
    """Robust local ICP, returning current-base pose in saved-base coordinates.

    Surface-normal Hessian rejects scenes lacking independent x/y/yaw evidence.
    This diagnostic is NOT a certificate of physical millimetre accuracy.
    """
    current, reference = np.asarray(current), np.asarray(reference)
    if len(current) < 50 or len(reference) < 50:
        return None
    tree = cKDTree(reference)
    pose = np.array(initial, dtype=float)
    for _ in range(25):
        moved = transform(current, pose)
        dist, idx = tree.query(moved)
        keep = dist < max_pair
        if keep.sum() < 50:
            return None
        cutoff = min(max_pair, max(0.02, float(np.quantile(dist[keep], .85))))
        keep &= dist <= cutoff
        a, b = moved[keep], reference[idx[keep]]
        ac, bc = a.mean(axis=0), b.mean(axis=0)
        u, _, vt = np.linalg.svd((a-ac).T @ (b-bc))
        rot = vt.T @ u.T
        if np.linalg.det(rot) < 0:
            vt[-1] *= -1
            rot = vt.T @ u.T
        shift = bc-rot @ ac
        angle = math.atan2(rot[1, 0], rot[0, 0])
        pose[:2] = rot @ pose[:2]+shift
        pose[2] = wrap(pose[2]+angle)
        if np.linalg.norm(shift) < .0001 and abs(angle) < .0001:
            break
    moved = transform(current, pose)
    dist, idx = tree.query(moved)
    keep = dist < .05
    overlap = float(keep.mean())
    if keep.sum() < 50:
        return None
    # Estimate local surface normals from reference neighborhoods.
    _, neighbors = tree.query(reference[idx[keep]], k=7)
    patches = reference[neighbors]
    centered = patches-patches.mean(axis=1, keepdims=True)
    cov = np.einsum('nki,nkj->nij', centered, centered)
    eig, vectors = np.linalg.eigh(cov)
    planar = eig[:, 0] < .2*np.maximum(eig[:, 1], 1e-12)
    normals = vectors[planar, :, 0]
    p = moved[keep][planar]
    if len(p) < 30:
        return None
    j = np.column_stack((normals, -normals[:, 0]*p[:, 1]+normals[:, 1]*p[:, 0]))
    h = j.T @ j/len(j)
    eigen = np.linalg.eigvalsh(h)
    return {'pose': pose, 'rmse': float(np.sqrt(np.mean(dist[keep]**2))),
            'overlap': overlap, 'observability': float(eigen[0]),
            'condition': float(eigen[-1]/max(eigen[0], 1e-12))}


def alignment_command(error, xy_tolerance, yaw_tolerance, max_v=.035, max_w=.12):
    """Nonholonomic pose approach; error is current pose in goal frame."""
    x, y, yaw = error
    distance = math.hypot(x, y)
    if distance <= xy_tolerance:
        return 0., float(np.clip(-yaw*.8, -max_w, max_w)) if abs(yaw)>yaw_tolerance else 0.
    bearing = wrap(math.atan2(-y, -x)-yaw)
    direction = 1.
    if abs(bearing) > math.pi/2:
        direction = -1.
        bearing = wrap(bearing-math.copysign(math.pi, bearing))
    w = float(np.clip(bearing, -max_w, max_w))
    v = direction*min(max_v, distance*.5)*max(0., math.cos(bearing))
    if abs(bearing) > .25:
        v = 0.
    return v, w


def swept_clear(points, v, w, half_length=.31, half_width=.24, margin=.04, horizon=1.):
    """Check swept rectangular footprint against measured obstacle returns."""
    if len(points) == 0:
        return False
    for t in np.linspace(0., horizon, 15):
        angle = w*t
        x = v*t if abs(w)<1e-8 else v/w*math.sin(angle)
        y = 0. if abs(w)<1e-8 else v/w*(1-math.cos(angle))
        c, s = math.cos(angle), math.sin(angle)
        q = (np.asarray(points)-[x, y]) @ np.array([[c, -s], [s, c]])
        if np.any((abs(q[:, 0]) < half_length+margin) & (abs(q[:, 1]) < half_width+margin)):
            return False
    return True


def accepts_return(intent):
    return intent != 'hold'


def accepts_patrol(intent):
    return intent not in ('return',)
