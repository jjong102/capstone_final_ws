Current mapping network selection
=================================
Default: network:=wifi (wlP1p1s0, laptop 10.96.147.234).
Use network:=jetson for eno1 and wired UI 192.168.10.1.

ros2 launch all_in_one_package generate_map.launch.py use_rviz:=false log:=true network:=wifi
ros2 launch all_in_one_package generate_map.launch.py use_rviz:=false log:=true network:=jetson

Stop the existing mapping launch before switching. These are single-interface
profiles, not simultaneous wired/Wi-Fi delivery. All mapping children and the
recorder inherit the selected profile before they start. Receiver configuration
is unchanged. The robot's map/URDF/navigation logic is unchanged.
An explicit nonempty dds_config:=/absolute/path.xml overrides the selected
profile. The network value must still be wifi or jetson. The startup log reports
the selected mode, override flag and resolved XML. session.json records network
and the effective CYCLONEDDS_URI when log:=true.
Both profiles are installed with all_in_one_package; rebuild that package.
This change applies only to mapping; impact/make_goal configuration is unchanged.

Historical dual-network investigation (not the mapping default)
==============================================================
Mapping with wired UI receiver and Wi-Fi RViz

Robot: eno1 192.168.10.2/24; wlP1p1s0 10.96.151.196/16.
Wired UI: 192.168.10.1. Laptop Wi-Fi: 10.96.147.234.
Domain: 3, RMW: rmw_cyclonedds_cpp, ROS_LOCALHOST_ONLY=0.

generate_map.launch.py now loads the installed all_in_one_package/config/jetson_dual.xml
before creating any sensor, mapping or recording processes. It overrides inherited
wired-only /home/moonshot/.ros/cyclonedds.xml or Wi-Fi-only environment settings.
Override with dds_config:=/absolute/path/config.xml and domain_id:=3 if needed.
No host routes, network addresses or remote UI nodes are modified.
Existing processes must be restarted to use the new environment.

The LAN receiver keeps its existing LAN configuration and UI/command topics.
The laptop uses config/remote_rviz/laptop.xml (copy to ~/remote_rviz/laptop.xml),
then exports ROS_DOMAIN_ID=3 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
and CYCLONEDDS_URI=file://$HOME/remote_rviz/laptop.xml before starting rviz2.
Raw rviz2 and ROS CLI commands do not inherit environment changes made inside a
separately launched laptop_rviz_launch.py process.

Both network interfaces initialized successfully in a read-only ROS participant
check. End-to-end UI delivery has not been verified on the remote machines.
Cartographer timestamp failures are a separate issue, not repaired by this change.
Run mapping sensor/TF stack only once; stop impact navigation before mapping.

2026-11-03 dual-network revision
--------------------------------
Robot config now enables both interfaces with RedundantNetworking, prefers Wi-Fi,
and sets DontRoute=true. Laptop and wired UI also require DontRoute=true: each
receiver must ignore the robot locator on the other, unreachable subnet.
This profile assumes directly connected 10.96.0.0/16 and 192.168.10.0/24 networks;
it is not intended for VPNs or routed peers. UDP message size is capped at 1400 B
and DDS fragments at 1200 B to avoid ordinary Ethernet/Wi-Fi IP fragmentation.

Deploy laptop.xml to ~/remote_rviz/laptop.xml on the laptop. Deploy ui_wired.xml
on 192.168.10.1 and export its absolute file URI before starting the UI process:
  export ROS_DOMAIN_ID=3 ROS_LOCALHOST_ONLY=0
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=file://$HOME/remote_rviz/ui_wired.xml
Restart the UI/RViz process after changing the environment (daemon restart alone
is not sufficient). Do not start extra sensor, EKF or mapping nodes on receivers.

Mapping uses all_in_one_package/config/jetson_dual.xml; impact and make_goal use
third_impact/config/jetson_dual.xml, with domain_id:=3 and dds_config overrides.
The three robot XML copies must remain identical. Wi-Fi-only fallback remains:
  dds_config:=/home/moonshot/capstone_final_ws/config/remote_rviz/jetson_wifi.xml

Validation: installed Cyclone DDS 0.10.5 option definitions were checked locally.
Remote simultaneous map/TF/scan reception and late-join/reconnect still require
verification on both physical receivers; local NIC tests cannot establish this.
