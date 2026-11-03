20261103_184025_014330 patrol blocked analysis (no runtime edits)

18:40:33.450 navigation accepted; 33.909 canceled.
18:40:37.656 navigation accepted; 40.555 canceled.
18:40:44.206 navigation accepted; 45.405 canceled, recovery STOP.
18:40:46.455 recovery WAIT, front_scan_stale: accepted age .506067 s,
scan timeout .45 s. 18:40:46.639 health ready, but recovery stays WAIT.
18:40:45.854 odom_stale age .43089 s, threshold .3 s.
Localization readiness repeatedly dropped; progress timeout is elapsed wall time
since .20 m progress, and does not exclude localization pauses. Recovery began
about 12 seconds after first navigation acceptance, consistent with timeout.
Exact branch invoking recovery is not independently recorded.

Recorded latest-TF/scan reconstruction at 18:40:46:
pose [-1.42213,1.61196,1.51555]; exclusion distance 2.165 m vs .4634 m margin.
Boundary and exclusion tests passed. Current footprint and both 90-degree map
rotation sweeps passed. Both 90-degree LiDAR rotation sweeps passed.
Fresh-snapshot recovery_heading can find a heading (~0.10 s in this offline run).
This does NOT reproduce exact callback scheduling or original path candidate.

recovery_heading returns None for missing/stale cloud as well as collision.
Caller converts every None to 'rotation space insufficient' and permanently
latches WAIT; sensor recovery alone cannot resume it. Staleness conflation is
therefore the strongest explanation, not a confirmed physical obstacle.
Timing data do not uniquely prove the scheduling/CPU cause of the stale scan.

Recommended: explicit reason codes for unavailable observation vs collision;
bounded fresh-sensor retries; cache one validated snapshot for rotation checks
with final freshness recheck before motion; exclude readiness pauses from progress
timeout. Keep footprint and collision/region margins unchanged.

After implementation:
- after_fix.txt: same recorded pose/scans passed fresh and simulated aging
  recovery_heading checks. ~3 ms per check versus ~100 ms in earlier local
  snapshot run. Only two cloud freshness calls, immutable candidate snapshot.
- Geometry remains URDF 70x44 cm, margin .04 m, existing region rules.
- This is offline snapshot validation, not closed-loop physical driving.
- New tests cover stale-data wait, requiring both new scans, expiration during
  candidate checks, 5-second timeout, 3-retry budget, and progress pause accounting.
