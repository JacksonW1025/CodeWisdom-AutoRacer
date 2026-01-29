#!/bin/bash

# Detect and source ROS2 distribution
ROS2_DISTROS=("rolling" "jazzy" "iron" "humble" "galactic" "foxy")
ROS2_DISTRO_FOUND=""

for distro in "${ROS2_DISTROS[@]}"; do
    if [ -f "/opt/ros/$distro/setup.bash" ]; then
        ROS2_DISTRO_FOUND="$distro"
        break
    fi
done

if [ -n "$ROS2_DISTRO_FOUND" ]; then
    source "/opt/ros/$ROS2_DISTRO_FOUND/setup.bash"
    echo "Sourced ROS2 $ROS2_DISTRO_FOUND: /opt/ros/$ROS2_DISTRO_FOUND/setup.bash"

    # 仅本机通信，禁止 DDS 局域网发现（避免其他机器人干扰）
    export ROS_LOCALHOST_ONLY=1
    echo "ROS_LOCALHOST_ONLY=1 (disabled network DDS discovery)"
else
    echo "Error: No ROS2 distribution found in /opt/ros/"
    return 1 2>/dev/null || exit 1
fi

# Source workspace install setup (if it exists)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "$SCRIPT_DIR/install/setup.bash" ]; then
    source "$SCRIPT_DIR/install/setup.bash"
    echo "Sourced workspace: $SCRIPT_DIR/install/setup.bash"
else
    echo "Warning: install/setup.bash not found. Run 'colcon build' first."
fi
