#!/usr/bin/env bash

set -e  # 에러 나면 즉시 중단

echo "=============================="
echo " ROS 2 Humble MoveIt + Modbus "
echo "=============================="

# ROS2 환경 확인
if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "[ERROR] ROS 2 Humble not found in /opt/ros/humble"
  exit 1
fi

echo "[INFO] Sourcing ROS 2 Humble"
source /opt/ros/humble/setup.bash

echo "[INFO] Updating apt..."
sudo apt update

echo "------------------------------"
echo "[INFO] Installing MoveIt planners"
echo "------------------------------"

sudo apt install -y \
  ros-humble-ompl \
  ros-humble-moveit-planners-ompl \
  ros-humble-moveit-planners-chomp 

echo "------------------------------"
echo "[INFO] Installing Modbus (C++)"
echo "------------------------------"

sudo apt install -y \
  libmodbus-dev \
  libmodbus5

echo "------------------------------"
echo "[INFO] Installing build tools"
echo "------------------------------"

sudo apt install -y \
  build-essential \
  cmake \
  pkg-config

echo "------------------------------"
echo "[SUCCESS] Installation completed!"
echo "------------------------------"

echo ""
echo "Verify:"
echo "  pkg-config --modversion libmodbus"
echo "  ros2 pkg list | grep moveit"
