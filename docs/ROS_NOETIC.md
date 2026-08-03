# ROS Noetic verification

This document applies to the `noetic` branch, which targets ROS Noetic on
Ubuntu 20.04 with Gazebo Classic 11. It is a ROS 1 catkin workspace: clone it
below `src/`, install the declared dependencies with `rosdep`, and build with
`catkin_make`. The ROS 2 packages are maintained on the `foxy`, `humble`, and
`jazzy` branches.

## Reproducible Docker check

The same check used by CI can be run on a Linux host with Docker:

```bash
docker run --rm -it --env DEBIAN_FRONTEND=noninteractive ros:noetic-ros-base bash
apt-get update
apt-get install -y \
  ros-noetic-ackermann-msgs ros-noetic-gazebo-ros \
  ros-noetic-gazebo-ros-control ros-noetic-gazebo-plugins \
  ros-noetic-hector-models ros-noetic-ros-controllers \
  ros-noetic-joint-state-publisher ros-noetic-robot-state-publisher \
  ros-noetic-tf ros-noetic-tf2-ros ros-noetic-tf-conversions \
  ros-noetic-xacro python3-numpy python3-rospkg
mkdir -p /root/catkin_ws/src
# Copy or clone this repository to /root/catkin_ws/src/ackermann_vehicle.
cd /root/catkin_ws
source /opt/ros/noetic/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
catkin_make
catkin_make install
source devel/setup.bash
bash src/ackermann_vehicle/scripts/noetic_smoke_test.sh
```

The smoke test runs headless (`gui:=false`) with physics unpaused so Gazebo can
service controller callbacks; it does not require an X server or a manual
driving command. The launch path exercises the xacro-generated description and
Gazebo model spawn, then checks the long-lived Ackermann controller node,
controller-manager service, and expected controller topics.

## Scope and limitations

* The tested path uses Gazebo Classic and `gazebo_ros_control`.
* The Hokuyo and Hokuyo-IMU launch files are supported as optional sensor
  variants, but sensor plugin availability is host-dependent.
* The verified controller path uses the root namespace (`namespace:=/`).
  Custom namespaces require a separate controller-manager integration check.
* Melodic compatibility is retained as legacy configuration and is not covered
  by the Noetic CI job.
* ROS 2 support is maintained on the separate `foxy`, `humble`, and `jazzy`
  branches. Gazebo/controller migration remains distribution-specific.
