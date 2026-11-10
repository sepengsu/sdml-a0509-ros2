#!/bin/bash
set -e

# Export DISPLAY for GUI applications
export DISPLAY="${DISPLAY:-:99}"

# Start VNC services
if [ -f /start-vnc.sh ]; then
    source /start-vnc.sh
fi

# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Source workspace if built
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
    echo "Sourced workspace: /ros2_ws/install/setup.bash"
fi

# Auto-launch ROS2 if enabled
if [ "${AUTO_LAUNCH:-false}" = "true" ]; then
    # Check if workspace is built, build if not
    if [ ! -f /ros2_ws/install/setup.bash ]; then
        echo "============================================"
        echo " Workspace not built. Building now..."
        echo " (This may take a few minutes on first run)"
        echo "============================================"
        cd /ros2_ws
        if ! colcon build --symlink-install; then
            echo "============================================"
            echo " BUILD FAILED! Dropping to bash shell."
            echo " Fix the errors and run 'colcon build' manually."
            echo "============================================"
            exec bash
        fi
        source /ros2_ws/install/setup.bash
        echo "Build complete!"
    fi

    echo "AUTO_LAUNCH enabled. Starting ROS2 launch..."
    LAUNCH_FILE="${LAUNCH_FILE:-dsr_bringup2_rviz.launch.py}"
    LAUNCH_PKG="${LAUNCH_PKG:-dsr_bringup2}"

    # Wait for emulator to be ready (virtual mode)
    if [ "${ROBOT_MODE:-virtual}" = "virtual" ]; then
        echo "Waiting for emulator at ${ROBOT_HOST}:12345..."
        for i in {1..30}; do
            if nc -z "${ROBOT_HOST:-robot-emulator}" 12345 2>/dev/null; then
                echo "Emulator is ready!"
                break
            fi
            echo "  Waiting... ($i/30)"
            sleep 2
        done
    fi

    # Launch in background and keep container running
    ros2 launch "$LAUNCH_PKG" "$LAUNCH_FILE" \
        host:="${ROBOT_HOST:-127.0.0.1}" \
        mode:="${ROBOT_MODE:-virtual}" \
        model:="${ROBOT_MODEL:-a0509}" &

    # If command is 'bash', run interactive shell
    if [ "$1" = "bash" ]; then
        exec bash
    fi
fi

# Execute passed command
exec "$@"
