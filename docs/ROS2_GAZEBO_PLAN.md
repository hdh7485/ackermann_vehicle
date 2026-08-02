# ROS 2 and modern Gazebo plan

Version 0.1.5 remains a ROS 1 / Gazebo Classic maintenance release. A full
migration would change the controller interfaces, launch system, message
transport, and simulator APIs, so it is deliberately tracked as a separate
body of work.

## Staged roadmap

1. **Inventory and contracts** — record topic names, frame names, controller
   parameters, and expected Ackermann kinematics from the ROS 1 smoke test.
2. **Description port** — make the xacro model portable, remove ROS 1-only
   launch assumptions, and validate it with ROS 2 `robot_state_publisher`.
3. **Control port** — replace `gazebo_ros_control` and ROS 1 controller-manager
   wiring with `ros2_control`, preserving the existing command and joint
   semantics where practical.
4. **Simulator adapters** — validate the model in modern Gazebo (the successor
   to Gazebo Classic), add a headless container smoke test, and document any
   sensor-plugin differences.
5. **Navigation nodes** — port the command converter and path helpers to
   `rclpy`, add ROS 2 launch tests, and publish a migration example.
6. **Release and deprecation** — publish a separate ROS 2 package/repository
   or major-version branch only after the ROS 1 behavior contract is covered.

No ROS 2 dependency or partial compatibility claim is introduced by this
maintenance release. Contributions toward the roadmap should first add a
reproducible test and document the API or simulator assumption it changes.
