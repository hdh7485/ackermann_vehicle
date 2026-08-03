# ROS 2 Foxy and Gazebo plan

## Completed ROS 2 core milestone (0.2.0)

The repository now contains the `ackermann_vehicle_ros2` ament-Python package.
It preserves a stable ROS-level command boundary: `geometry_msgs/Twist` is
converted to `ackermann_msgs/AckermannDriveStamped` using explicit Ackermann
geometry, and the E-Maxx xacro is installed for ROS 2
`robot_state_publisher` tooling.

| Distribution | Base platform | Status | CI/runtime evidence |
| --- | --- | --- | --- |
| Foxy | Ubuntu 20.04 | Legacy core support | `colcon build`, kinematics tests, bridge publish/subscribe smoke, robot-description launch |

`scripts/ros2_smoke_test.sh` owns a separate ROS 2 domain by default and
verifies both the transformed command and robot-description process without a
GUI. `.github/workflows/ros2.yml` repeats this validation only for Foxy. The
`humble` and `jazzy` branches own their respective compatibility contracts.

Foxy is end-of-life, so its status is deliberately compatibility-only: CI
signals when the common package regresses but does not promise new Foxy-only
features.

## Branch maintenance

`jazzy` is the default branch and upstream integration target for shared ROS 2
work. Land distribution-independent changes there first, then deliberately
backport them to `humble` and `foxy` only after their branch-local CI passes.
Keep Foxy-only dependency, compatibility, and documentation changes on this
branch.

## Deliberately separate simulator migration

The original xacro still contains ROS 1 Gazebo Classic transmission and plugin
definitions. A full simulator port must not pretend that one plugin stack
works across all ROS 2 distributions:

1. **Description split** — extract ROS-neutral geometry from the existing
   xacro, then add separate Classic and modern-Gazebo control overlays.
2. **Controller port** — replace ROS 1 `gazebo_ros_control` and per-joint
   controller-manager wiring with `ros2_control`, preserving the documented
   command and joint semantics where practical.
3. **Classic adapter (Foxy/Humble)** — add a headless Gazebo Classic launch
   backed by `gazebo_ros2_control`, with its own controller and spawn tests.
4. **Modern Gazebo adapter (Jazzy)** — add a `ros_gz_sim` / `gz_ros2_control`
   adapter, SDF resources, and a Harmonic headless smoke test. Modern Gazebo
   uses a different transport and plugin model from Classic.
5. **Sensors and navigation** — port Hokuyo/IMU plugins and the remaining
   path-follower helpers only after their message and simulator contracts have
   regression coverage.

The roadmap intentionally does not claim vehicle dynamics, sensors, worlds,
or controller parity until the distribution-specific simulator tests exist.
