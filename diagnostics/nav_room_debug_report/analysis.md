# Navigation recording analysis

Recording: `nav_room_debug`, 118.602 s. Analysis performed offline; no robot commands or navigation parameters changed.

## Confirmed findings

- `/goal_pose` has no recorded messages. The endpoint of `/plan` is constant: x=2.16980147 m, y=1.51918983 m, yaw=173.104185 deg. Goal error below uses this endpoint as a proxy for the requested goal.
- Around 22 s: map-frame robot position is about 0.130 m from the endpoint; yaw error is about 149 deg.
- Five controller progress failures occur at 31.085, 41.341, 51.471, 76.565, and 86.684 s after bag start. Corresponding log lines explicitly report `Failed to make progress` and abort FollowPath.
- At 51.489 s behavior_server runs a recovery spin, which times out at 61.493 s. Around 86.699 s it runs backup, which ends with a collision prediction at 95.102 s. An earlier backup at 31.099 s also immediately ends with a collision prediction.
- Controller reports goal reached at 97.272 s; bt_navigator reports Goal succeeded at 97.308 s.
- All 1462 local path messages contain poses. Of these, 922 have effectively zero translational length: pure rotation paths, which need not appear as visible lines in RViz.

## Primary explanation

Configured SimpleProgressChecker requires a 0.15 m translation within 10 s. Its Humble implementation checks only x/y displacement, not yaw progress. Large final rotations can therefore be interrupted despite ongoing angular progress. Recorded yaw rates during substantial rotation are often around 0.15-0.20 rad/s; turning 149 deg can take roughly 13-17 s, exceeding 10 s.

The progress failures trigger recovery spins and backups. These recoveries explain the observed additional turns and reverse motion; no arm-task intervention is needed to explain this recording.

## Secondary observations and limitations

Distance crosses approximately 0.16 m during early rotations, and DWB rotation commands switch sign. Replanning resets DWB critics; near-boundary motion can re-enter path-following behavior. Exact critic scores and costmaps were not recorded, so the cause of every direction reversal is not determined.

Final angular response is slower than several commanded rotation values. Motor code integrates cmd_vel with a fixed 0.05 s step into integer wheel-position targets. Actual rates, encoder units and quantization warrant separate validation if oscillation persists after allowing enough time for final alignment.

## Suggested first controlled change

Keep both goal distance tolerances at 0.16 m and yaw tolerance at 0.10 rad. Change only progress_checker.movement_time_allowance from 10.0 to 60.0 s for a repeat test. This preserves requested arrival tolerances but delays recovery for a genuinely stuck robot. It is a diagnostic tuning step, not a guarantee of physical positioning accuracy.

Source: https://github.com/ros-navigation/navigation2/blob/humble/nav2_controller/plugins/simple_progress_checker.cpp

See `relevant_logs.txt`, `timeline.csv`, and `goal_approach.png` for evidence. Timeline commands are last-observed values and may be stale across publisher gaps; the plotted command samples show actual message timestamps.
