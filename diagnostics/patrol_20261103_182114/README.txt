Bag analysis: 20261103_182114_228934
No runtime source/config edits made during this analysis.

18:21:47 first patrol destination completed; patrol_visits became 1.
18:22:25.188 second destination switched from Nav2 to near-goal approach.
18:22:25.219 navigation cancellation was expected handoff, not a Nav2 failure.
18:22:26.735 readiness false: localization_not_ready. Then SEARCHING/SEARCH_FAILED
repeated through recording end. Both scans remained fresh at transition (~0.10 s).

Reconstructed latest recorded TF and filtered scans, same GridModel verification:
18:22:27 overlap .9292; wall-boundary agreement .3662; rays free fraction .8585;
footprint clear true. rays_clear requires >= .90, thus verification fails.
End: overlap .9327; boundary agreement .3853; free ray fraction .8603.
These reconstruction snapshots use latest received transforms/scans (not an exact
replay of callback scheduling). geometry.json retains one-second samples.

GridModel distance measures distance to ANY occupied cell, including solid black
interior. Endpoint overlap can increase while actual boundary agreement drops.
At 18:22:17 reconstructed boundary agreement .4714; at 18:22:22 .3221.
localization.tick returns READY without recomputing geometry while ready and
moving, delaying detection until stopped for near-goal handoff.
Physical cause of scan/map disagreement (map, TF/timing, localization drift) is
not uniquely established by this bag analysis. Do not relax the .90 gate merely
to resume motion. Actual ground truth unavailable.

Plan: boundary-based per-sensor diagnostics and scores; synchronized observation/
pose evaluation during motion; cautious stop and bounded local revalidation
before global search; expose exact failure and age in UI/log; unify localization
footprint with URDF-based 70x44cm geometry (current check uses 66x52cm rectangle).
Replay regression plus physical validation, preserve patrol 5cm and region rules.

Implementation validation (same bag, production geometry / readiness functions):
- replay_validation.py uses tf2 Buffer with recorded TF, scan timestamps, odom,
  and recorded AMCL. No ROS graph or velocity output is used.
- Production on_scan/cloud/pose/tick process observations, including inter-scan
  motion compensation and timestamped map pose lookup.
- Global search execution is stubbed as unsuccessful; this replay verifies the
  transition into search, NOT the outcome of a new global search or AMCL feedback.
- At 18:22:15.795: DEGRADED; at 18:22:16.335: WAIT_STOP (62.9% wall overlap,
  11.0 cm boundary cost). Old system stopped at 18:22:26.735, 10.4 s later.
- At 18:22:26.543: LOCAL_RECHECK; at 18:22:32.730: SEARCHING after bounded
  stationary observation period. Recorded trajectory is unchanged by our stop.
- Replay also rejects sustained bad geometry at 18:21:37.154, so live operation
  may stop earlier in this run. This is not evidence of completed live patrol.
- Last sample: combined overlap 35.8%; front 15.3%; rear 51.4%; combined free-ray
  fraction 86.3%. Timestamp alignment/voxel sampling differ from old snapshots.
- 111 tests passed / 2 skipped; follow-up localization tests 11 passed after
  ensuring movement resets the six-second stationary recheck period.
- third_impact C++/Python build completed, including RViz panel.
