#!/usr/bin/env bash

set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
CATKIN_WS="${CATKIN_WS:-$(pwd)}"
LOG_DIR="${ROS_SMOKE_LOG_DIR:-${TMPDIR:-/tmp}/ackermann_vehicle_ros_smoke}"
LAUNCH_LOG="${LOG_DIR}/roslaunch.log"
ROSCORE_LOG="${LOG_DIR}/roscore.log"
mkdir -p "${LOG_DIR}"

# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ -f "${CATKIN_WS}/devel/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "${CATKIN_WS}/devel/setup.bash"
fi

# Always own an isolated local master. This prevents a failed roscore from
# falling through to an ambient ROS_MASTER_URI and touching another graph.
ROS_MASTER_PORT="${ROS_MASTER_PORT:-}"
if [[ -z "${ROS_MASTER_PORT}" ]]; then
  ROS_MASTER_PORT="$(python3 - <<'PY'
import socket

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
    sock.bind(("127.0.0.1", 0))
    print(sock.getsockname()[1])
PY
)"
fi
export ROS_MASTER_URI="http://127.0.0.1:${ROS_MASTER_PORT}"

roscore -p "${ROS_MASTER_PORT}" >"${ROSCORE_LOG}" 2>&1 &
roscore_pid=$!
launch_pid=""

cleanup() {
  if [[ -n "${launch_pid}" ]]; then
    # Start roslaunch in its own session below so Gazebo and controller
    # children can be stopped without touching nodes in another ROS graph.
    # The isolated master makes this node list owned by this smoke test.
    while read -r node; do
      [[ "${node}" == "/rosout" ]] && continue
      rosnode kill "${node}" 2>/dev/null || true
    done < <(rosnode list 2>/dev/null || true)
    kill -INT -- "-${launch_pid}" 2>/dev/null || kill -INT "${launch_pid}" 2>/dev/null || true
    for _ in $(seq 1 10); do
      if ! kill -0 "${launch_pid}" 2>/dev/null && \
         ! kill -0 -- "-${launch_pid}" 2>/dev/null; then
        break
      fi
      sleep 1
    done
    kill -TERM -- "-${launch_pid}" 2>/dev/null || kill -TERM "${launch_pid}" 2>/dev/null || true
    sleep 1
    kill -KILL -- "-${launch_pid}" 2>/dev/null || true
  fi
  if kill -0 "${roscore_pid}" 2>/dev/null; then
    kill "${roscore_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

for _ in $(seq 1 30); do
  if ! kill -0 "${roscore_pid}" 2>/dev/null; then
    break
  fi
  if rosnode list >/dev/null 2>&1; then
    break
  fi
  sleep 1
done
if ! kill -0 "${roscore_pid}" 2>/dev/null || ! rosnode list >/dev/null 2>&1; then
  echo "Unable to start an isolated ROS master at ${ROS_MASTER_URI}." >&2
  cat "${ROSCORE_LOG}"
  exit 1
fi

setsid roslaunch ackermann_vehicle_gazebo ackermann_vehicle_noetic.launch \
  gui:=false paused:=false use_sim_time:=true >"${LAUNCH_LOG}" 2>&1 &
launch_pid=$!

controller_service=""
for _ in $(seq 1 90); do
  if ! kill -0 "${launch_pid}" 2>/dev/null; then
    break
  fi
  if rosnode list 2>/dev/null | grep -qx '/ackermann_controller'; then
    if rosservice list 2>/dev/null | grep -qx '/controller_manager/list_controllers' && \
       rosservice call /controller_manager/list_controllers >/dev/null 2>&1; then
      controller_service="/controller_manager/list_controllers"
      break
    fi
  fi
  sleep 1
done

if [[ -z "${controller_service}" ]]; then
  echo "Gazebo/controller smoke test failed. Logs: ${LOG_DIR}" >&2
  cat "${LAUNCH_LOG}"
  exit 1
fi

controller_topics=""
for _ in $(seq 1 30); do
  controller_topics="$(rostopic list 2>/dev/null || true)"
  if grep -qx '/joint_states' <<<"${controller_topics}" && \
     grep -qx '/left_steering_ctrlr/command' <<<"${controller_topics}"; then
    break
  fi
  sleep 1
done
if ! grep -qx '/joint_states' <<<"${controller_topics}" || \
   ! grep -qx '/left_steering_ctrlr/command' <<<"${controller_topics}"; then
  echo "Controller manager is reachable but expected controller topics were not loaded." >&2
  cat "${LAUNCH_LOG}"
  exit 1
fi

echo "ROS Noetic headless smoke test passed. Master: ${ROS_MASTER_URI}. Logs: ${LOG_DIR}"
