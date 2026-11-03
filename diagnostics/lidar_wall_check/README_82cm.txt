Stationary front/rear wall comparison, 2026-11-03 (KST)
User corrected BOTH outer-envelope-to-wall measurements to approximately 0.82 m.

Front: 20261103_190147/bag, last 15 s ending at user front marker:
19:02:43.468--19:02:58.468, 150 raw scans + 150 filtered scans.
First recording terminated with SQLite database-locked error after this interval.
Rear: rear_20261103_190410/bag, final 15 s:
19:04:20.599--19:04:35.599, 150 raw scans + 150 filtered scans.
Rear recording stopped normally. Recording was not queried while active.

Methods:
Use recorded sensor TF (full quaternion), select outward-facing wall sector,
RANSAC wall line then total least squares on inliers, per scan.
Sector base outward x 0.4--1.7 m, lateral |y|<0.65 m; residual gate 15 mm.
Report perpendicular distance, not an arbitrary single central range beam.
Odom indicates both selected windows stationary; this is not external ground truth.
Raw/filtered results are identical for selected outward wall sectors.

                        Front            Rear
Sensor-to-wall median   0.845823 m       0.848334 m
Base-to-wall median     1.156823 m       1.159328 m
Outer-to-wall median    0.807323 m       0.809834 m
Normal vs facing axis   -0.0963 deg      +0.3317 deg
Wall line residual RMS  1.70 mm          2.39 mm

URDF: sensor centers x=+/-0.311 m; cylinder radius .0385 m.
Assumed outer reference: outermost LiDAR housing, x=+/-0.3495 m,
consistent with conservative 70 cm footprint. NOT necessarily tape reference
if user measured from chassis (chassis x=+/-0.305 m).
Expected sensor distance for outer-reference .82 m is approx .8585 m.
Residual relative to approximate .82 m outer measurement: -12.68 mm front,
-10.17 mm rear. Do not infer calibration corrections from approximate tape data.

Difference between sensor distance medians: 2.51 mm. This is agreement between
separately positioned measurements, NOT proof of 2.51 mm sensor accuracy.
Different placement, rotation angle, tape reference, wall flatness are confounders.
No evidence of the earlier ~13 cm rear map-boundary residual in this stationary
single-wall test. Does not exclude angular calibration, other ranges/materials,
motion timestamp issues, map distortion or AMCL pose error. No runtime changes.
Reproduce: validate.py; full output: comparison_82cm.json.
