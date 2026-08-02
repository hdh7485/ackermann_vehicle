# ackermann_vehicle

ROS packages for simulating a vehicle with Ackermann steering in Gazebo
Classic. The repository is maintained at
<https://github.com/hdh7485/ackermann_vehicle> and keeps the original
Wunderkammer Laboratory attribution in the package metadata and source files.

## Support policy

| Platform | Status | Verification |
| --- | --- | --- |
| ROS Noetic / Ubuntu 20.04 / Gazebo Classic 11 | Supported | Build and headless launch smoke test in CI and `scripts/noetic_smoke_test.sh` |
| ROS Melodic / Ubuntu 18.04 / Gazebo Classic 9 | Legacy | Configuration is retained, but this release does not run a Melodic CI job |
| ROS 2 / modern Gazebo | Roadmap | See [`docs/ROS2_GAZEBO_PLAN.md`](docs/ROS2_GAZEBO_PLAN.md); not supported by this ROS 1 release |

The supported release is ROS Noetic. The sensor variants require the matching
Gazebo sensor plugins; the default headless smoke test intentionally exercises
the base vehicle and controller path.

## Installation (ROS Noetic)

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone --branch noetic https://github.com/hdh7485/ackermann_vehicle.git
cd ~/catkin_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

For a repeatable, display-free check from the workspace, run:

```bash
bash src/ackermann_vehicle/scripts/noetic_smoke_test.sh
```

The script starts a temporary ROS master, launches
`ackermann_vehicle_gazebo/launch/ackermann_vehicle_noetic.launch` with Gazebo
GUI disabled, and verifies that the Ackermann node and controller-manager
services are available. To run it manually with a visible simulator:

```bash
roslaunch ackermann_vehicle_gazebo ackermann_vehicle_noetic.launch
```

## Driving the simulated vehicle

The controller consumes `ackermann_msgs/AckermannDrive` on `/ackermann_cmd`.
The navigation package also provides a `cmd_vel` converter:

```bash
rosrun ackermann_vehicle_navigation cmd_vel_to_ackermann_drive.py
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}'
```

## Known Gazebo/controller issue coverage

The Noetic launch files now load the controller YAML before invoking the
Noetic `controller_manager/spawner`, pass an explicit service timeout, and
expose `gui`, `paused`, and `use_sim_time` arguments. This addresses the
controller-spawner race/failure mode reported in issue #3 and makes the
headless path used by issue #4 reproducible. The `joy` launch file now points
to the converter in `ackermann_vehicle_navigation`, where catkin installs it.

If a local Gazebo installation still fails to load the model, capture the
smoke-test log from `/tmp/ackermann_vehicle_ros_smoke` and include the ROS,
Gazebo, and controller versions in the issue report. Hardware-specific Hokuyo
topics are outside the base controller smoke test.

## Original research and attribution

This package was updated from earlier open-source Ackermann vehicle work. See
the Apache-2.0 license and the per-package changelogs for attribution and
maintenance history.

![Test run steering terminal](images/testrunackermann.jpg)

## Video

[![Watch the video](https://img.youtube.com/vi/nZZEMrxxz2o/maxresdefault.jpg)](https://youtu.be/nZZEMrxxz2o)
