# Project environment for ROS 2 terminals (also sourced by tools/sim.sh):  source tools/env.sh
# System Humble + the ros2_ws overlay, with the same middleware as the sim. Values already set in the shell are kept.

ISAAC_MANIPULATOR_ROOT="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)"
export ISAAC_MANIPULATOR_ROOT
export ISAACSIM="${ISAACSIM:-$HOME/isaacsim}"      # Isaac Sim install (python.sh)
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export FOXGLOVE_PORT="${FOXGLOVE_PORT:-8765}"

source /opt/ros/humble/setup.bash
if [ -f "$ISAAC_MANIPULATOR_ROOT/ros2_ws/install/setup.bash" ]; then
    source "$ISAAC_MANIPULATOR_ROOT/ros2_ws/install/setup.bash"
else
    echo "[env.sh] ros2_ws is not built yet: cd $ISAAC_MANIPULATOR_ROOT/ros2_ws && colcon build" >&2
fi
