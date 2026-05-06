#!/bin/bash
set -e

WS=~/ros2_ws
AGENT=$WS/src/lgdxrobot2_agent

echo "===================================="
echo " Disabling LGDXRobot CLOUD modules"
echo "===================================="

# 1. Patch CMakeLists.txt
echo "[1/5] Patching CMakeLists.txt..."
sed -i 's/find_package(lgdxrobot_cloud_msgs REQUIRED)/find_package(lgdxrobot_cloud_msgs QUIET)/g' $AGENT/CMakeLists.txt || true

# 2. Disable cloud in Agent.cpp
echo "[2/5] Patching Agent.cpp..."
cp $AGENT/src/Agent.cpp $AGENT/src/Agent.cpp.bak

sed -i 's/bool useCloud = this->get_parameter("use_cloud").as_bool();/bool useCloud = false; \/\/ CLOUD DISABLED/g' $AGENT/src/Agent.cpp || true

# 3. Remove cloud includes (safe fallback)
echo "[3/5] Patching Cloud.hpp..."
if [ -f $AGENT/include/lgdxrobot2_agent/Cloud.hpp ]; then
  sed -i 's/#include "lgdxrobot_cloud_msgs\/msg\/robot_data.hpp"/\/\/ cloud disabled/g' $AGENT/include/lgdxrobot2_agent/Cloud.hpp || true
  sed -i 's/#include "lgdxrobot_cloud_msgs\/srv\/mcu_sn.hpp"/\/\/ cloud disabled/g' $AGENT/include/lgdxrobot2_agent/Cloud.hpp || true
fi

# 4. Clean launch cloud references
echo "[4/5] Cleaning launch files..."
LAUNCH_DIR=$WS/src/lgdxrobot2_bringup/launch

grep -rl "lgdxrobot_cloud" $LAUNCH_DIR 2>/dev/null | while read f; do
  sed -i 's/lgdxrobot_cloud_node/# cloud disabled/g' "$f" || true
done || true

# 5. Clean build
echo "[5/5] Cleaning build..."
cd $WS
rm -rf build install log

echo "===================================="
echo " CLOUD DISABLED"
echo " Now run: colcon build --symlink-install"
echo "===================================="
