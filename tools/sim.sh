#!/usr/bin/env bash
# Run the conveyor cell (isaac_sim/main.py), e.g.  tools/sim.sh --view headless --seed 1
# Settings (ISAACSIM, ROS_DOMAIN_ID, RMW_IMPLEMENTATION, FOXGLOVE_PORT) come from tools/env.sh; ROS nodes in other
# terminals source the same file. Isaac Sim runs Python 3.12 with its own bundled Humble libs, so it gets the
# environment without the /opt/ros (Python 3.10) paths.
# Alongside the sim, on the system Humble + ros2_ws (build it first: cd ros2_ws && colcon build):
#   robot_state_publisher (franka_description) for the panda links on /tf
#   foxglove_bridge on ws://localhost:$FOXGLOVE_PORT (default 8765) and the Foxglove app connected to it;
#   --no-foxglove skips both, --no-foxglove-app starts only the bridge
set -eo pipefail

ROOT="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)"
source "$ROOT/tools/env.sh"

ros=true
foxglove=true
foxglove_app=true
args=()
for arg in "$@"; do
    case "$arg" in
        --no-foxglove) foxglove=false ;;
        --no-foxglove-app) foxglove_app=false ;;
        --no-ros) ros=false; foxglove=false; args+=("$arg") ;;
        *) args+=("$arg") ;;
    esac
done

# Background jobs of a script start with SIGINT ignored: `trap - INT` restores it, so that the SIGINT on exit stops
# them (ros2 launch ignores it otherwise, and on SIGTERM leaves its nodes running)
pids=()
trap 'kill -INT "${pids[@]}" 2> /dev/null || true; wait "${pids[@]}" 2> /dev/null || true' EXIT

# Without a built ros2_ws (env.sh warns) there are no panda links on /tf
if $ros && ros2 pkg prefix franka_description > /dev/null 2>&1; then
    (trap - INT; exec ros2 launch franka_description robot_state_publisher.launch.py) &
    pids+=($!)
fi

if $foxglove && ss -Hltn "sport = :$FOXGLOVE_PORT" | grep -q .; then
    echo "[sim.sh] port $FOXGLOVE_PORT is in use (another bridge?): not starting foxglove_bridge" >&2
    foxglove=false
fi
if $foxglove; then
    # ros2_ws on the ament path lets the bridge serve the package:// meshes of /robot_description
    (trap - INT; exec ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:="$FOXGLOVE_PORT" use_sim_time:=true) &
    pids+=($!)
    if $foxglove_app && command -v foxglove-studio > /dev/null; then
        # The app retries until the bridge is up; it stays open after the sim exits
        foxglove-studio "foxglove://open?ds=foxglove-websocket&ds.url=ws://localhost:$FOXGLOVE_PORT" \
            > /dev/null 2>&1 &
    fi
fi

cd "$ROOT/isaac_sim"
env -u PYTHONPATH -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH \
    ROS_DISTRO=humble \
    LD_LIBRARY_PATH="$ISAACSIM/exts/isaacsim.ros2.core/humble/lib" \
    "$ISAACSIM/python.sh" main.py "${args[@]}"
