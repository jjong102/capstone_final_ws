# Mapping diagnostics

```bash
ros2 launch all_in_one_package generate_map.launch.py use_rviz:=false log:=true
```

`log` defaults to `false`. Enabled sessions create a new `YYYYMMDD_HHMMSS_microseconds`
folder here (Asia/Seoul). Override the parent with `log_dir:=/absolute/path`.

- `bag/`: SQLite rosbag of all discovered topics, including hidden topics. Includes
  raw/filtered scans, raw IMU text, IMU, wheel/EKF odometry, /joy, /cmd_vel (when published), TF, map, Cartographer
  outputs, rosout, diagnostics and parameter events when published. Splits at 1 GiB.
  `/tf_static` uses transient-local QoS to retain transforms published before discovery.
- `config/`: copies of installed launch/configuration files and robot description.
- `session.json`: launch arguments, selected ROS environment, bag command, snapshot errors.
- `process_logs/`: stdout/stderr of every process launched by this launch.
- `process_events.jsonl`: process start/exit times, commands, PIDs and exit codes.
- `ros_logs/`: native ROS node log files.
- `runtime/`: node/topic/service inventory, actual node parameters and topic endpoint
  QoS snapshots. Begins after 12 seconds and repeats 60 seconds after each collection.
  Failed/timed-out commands are recorded explicitly.
- `host_metrics.jsonl`: load, memory, CPU counters, disk counters and free space every 5 seconds.

Stop with Ctrl+C once and wait for launch to exit so rosbag can flush its cache and
write `bag/metadata.yaml` (up to 30 seconds before launch escalates shutdown).
Check a completed recording with `ros2 bag info <session>/bag`.
If recording stops early, launch reports it and its output remains in process_logs.
All-topic recording also includes unrelated nodes on the same ROS domain, which can
increase disk/CPU load. No total-size cap or automatic deletion is applied.
A low-disk warning is emitted below 2 GiB. Do not let the disk fill during mapping.

Topics that are never published cannot be recorded. A bag preserves published map
messages and the inputs needed to repeat SLAM; it is not a saved Cartographer
`.pbstream` state or a separately exported PGM. Save the final map normally as well.
Recordings are ignored by Git.

Joystick driving can command the motor directly without publishing /cmd_vel. In that
case /joy and /wheelmotor remain available; runtime topic snapshots explicitly show
whether /cmd_vel exists.
