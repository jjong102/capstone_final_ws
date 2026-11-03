Production dual-LiDAR AMCL validation (2026-11-03)

Harness: ../validate_fusion.py
Bag: src/third_impact/debug_log/20261103_174827_039173
Actual third_impact.amcl_scan node, odom time compensation, then real Nav2 AMCL.
Localhost only / ROS_DOMAIN_ID=215; no robot command publishers.
Replay duration after first common initialpose: 72.45 s.
Recorded later initialpose and map->odom excluded, same seed for all variants.

Production fusion: 722 scans published, 69 AMCL pose updates.
Diagnostic timer: 725 waiting_for_new_pair; 3 waiting_for_both_scans at startup.
No stale, skew, or TF rejection observed in this run.

Mean endpoint agreement within 12 cm of mapped occupied/free boundary:
                       Front        Rear
Filtered front only     3.48%        5.02%
Filtered rear only      1.70%       13.04%
Production fusion      95.00%       59.44%
Original raw front      7.09%       12.20%

These are offline wall-boundary agreement scores, NOT absolute localization
accuracy or the RViz UI's map score. No physical ground truth is available.
Single-sensor results reflect the shared seed without later localization resets;
they do not represent all possible initializations or general sensor quality.
Rear agreement remains lower; dual input does not establish full resolution of
rear observations. This replay does not validate live driving safety.
Detailed trajectories: results.json. Node diagnostic counts: fusion_status.json.
