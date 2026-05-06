#!/bin/bash
set -e

WS=~/ros2_ws/src

echo "=== Removing cloud dependency completely ==="

# 1. Remove dependency from CMakeLists.txt
find $WS -name "CMakeLists.txt" -exec sed -i '/lgdxrobot_cloud_msgs/d' {} \;

# 2. Remove headers include
find $WS -type f \( -name "*.cpp" -o -name "*.hpp" \) -exec sed -i '/lgdxrobot_cloud_msgs/d' {} \;

# 3. Disable Cloud class usage completely (safe stub)
find $WS/lgdxrobot2_agent -type f -name "*.cpp" -exec sed -i \
's/std::make_unique<Cloud>(/nullptr; \/\/ cloud disabled/g' {} \;

# 4. Force disable use_cloud everywhere
find $WS -type f -exec sed -i 's/use_cloud.*true/use_cloud false/g' {} \;

echo "DONE"
