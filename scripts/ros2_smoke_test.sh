#!/usr/bin/env bash

# Verify the ROS 2 command bridge and portable robot description without a GUI.
set -euo pipefail

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "ROS_DISTRO is required; source a supported ROS 2 installation first." >&2
  exit 1
fi

# Keep this smoke graph separate from an interactive ROS 2 graph and from
# unrelated distributions running on the same host. Callers can override it.
export ROS_DOMAIN_ID="${ACKERMANN_ROS_DOMAIN_ID:-203}"

log_directory="${ROS2_SMOKE_LOG_DIRECTORY:-/tmp/ackermann_vehicle_ros2_smoke}"
mkdir -p "$log_directory"

launch_pid=""
description_pid=""

cleanup() {
  if [[ -n "$description_pid" ]] && kill -0 "$description_pid" 2>/dev/null; then
    kill -TERM -- "-$description_pid" 2>/dev/null || true
    wait "$description_pid" 2>/dev/null || true
  fi
  if [[ -n "$launch_pid" ]] && kill -0 "$launch_pid" 2>/dev/null; then
    kill -TERM -- "-$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

wait_for_node() {
  local node_name="$1"
  for _ in $(seq 1 30); do
    if ros2 node list | grep -Fxq "$node_name"; then
      return 0
    fi
    sleep 1
  done
  echo "Timed out waiting for $node_name" >&2
  return 1
}

setsid ros2 launch ackermann_vehicle_ros2 cmd_vel_to_ackermann_drive.launch.py \
  >"$log_directory/command_bridge.log" 2>&1 &
launch_pid=$!
wait_for_node /cmd_vel_to_ackermann_drive

python3 "$(dirname "${BASH_SOURCE[0]}")/ros2_ackermann_probe.py" \
  >"$log_directory/ackermann_cmd.yaml" 2>&1 &
probe_pid=$!
for _ in $(seq 1 10); do
  if ! kill -0 "$probe_pid" 2>/dev/null; then
    break
  fi
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 2.0}, angular: {z: 1.0}}' >/dev/null
  sleep 1
done
wait "$probe_pid"

setsid ros2 launch ackermann_vehicle_ros2 robot_description.launch.py \
  >"$log_directory/robot_description.log" 2>&1 &
description_pid=$!
wait_for_node /robot_state_publisher
for _ in $(seq 1 20); do
  if grep -Eq 'got segment chassis|Robot initialized' \
      "$log_directory/robot_description.log"; then
    break
  fi
  sleep 1
done
grep -Eq 'got segment chassis|Robot initialized' \
  "$log_directory/robot_description.log"

echo "ROS 2 ${ROS_DISTRO} smoke test passed. Logs: $log_directory"
