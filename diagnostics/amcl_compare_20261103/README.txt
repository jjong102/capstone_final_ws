Offline actual Nav2 AMCL comparison, 2026-11-03.
Input: 20261103_174827_039173. Duration 72.45s after first recorded initialpose.
Isolated domains 215/216, ROS_LOCALHOST_ONLY=1, localhost-only discovery peers.
No hardware nodes, cmd_vel, robot-domain publication, or production config edits.
Map, odom->base_link TF, static sensor transforms and scans from bag.
Recorded map->odom and subsequent initialpose messages deliberately excluded.
Same first initial pose and covariance, AMCL model/particle/beam parameters.
front/rear/combined: filtered scans transformed using full sensor quaternion,
self-footprint returns removed, range 0.1..6m, virtual base_link scan with 720 bins.
Combined takes nearest return per bin; pairs <=0.15s, no motion deskew.
original_front: original raw /rplidar1/scan and native sensor TF, baseline.
Scoring: mean across AMCL updates of per-scan fraction of points within 0.12m
of occupied/free map boundary (not original occupied-interior overlap).
Each sensor scored separately. Nearest recorded scan timestamp within 0.2s.
Stability: AMCL update pose difference after accounting for recorded odometry;
includes initial convergence, not ground-truth localization error.
Repeat produced identical metrics; this does not establish robustness to other seeds.
Combined is strongest in this test, but rear remains only 60.5% boundary overlap.
No independent ground truth; live driving safety and other initializations untested.
run.py creates AMCL subprocesses, lifecycle activates and terminates them on exit.
Requires sourced ROS Humble and third_impact Python path, isolated DDS environment.
results.json/repeat/results.json: trajectories and scores. summary.json: both runs.
