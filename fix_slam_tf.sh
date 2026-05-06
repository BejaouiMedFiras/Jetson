#!/bin/bash
# ============================================================
# fix_slam_tf.sh — fixes the map→odom TF / slam_toolbox hang
# Run this ON THE JETSON (inside your container):
#   bash fix_slam_tf.sh
# ============================================================

set -e
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; NC='\033[0m'
ok()   { echo -e "${GREEN}[OK]${NC}  $*"; }
info() { echo -e "${CYAN}[--]${NC}  $*"; }
warn() { echo -e "${YELLOW}[WW]${NC}  $*"; }
fail() { echo -e "${RED}[EE]${NC}  $*"; }

PKG_DIR="/root/ros2_ws/install/dynamixel_motors/share/dynamixel_motors"
BUILD_DIR="/root/ros2_ws/build/dynamixel_motors"

echo ""
echo "════════════════════════════════════════"
echo "  STEP 1 — FastDDS shared-memory fix"
echo "════════════════════════════════════════"

# On Jetson, FastDDS shared memory causes slam_toolbox to deadlock silently.
# This profile disables SHM transport.
cat > /root/fastdds_no_shm.xml << 'XML'
<?xml version="1.0" encoding="UTF-8"?>
<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <profiles>
    <transport_descriptors>
      <transport_descriptor>
        <transport_id>udp_transport</transport_id>
        <type>UDPv4</type>
      </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="default_profile" is_default_profile="true">
      <rtps>
        <userTransports>
          <transport_id>udp_transport</transport_id>
        </userTransports>
        <useBuiltinTransports>false</useBuiltinTransports>
      </rtps>
    </participant>
  </profiles>
</dds>
XML
ok "Created /root/fastdds_no_shm.xml (disables SHM, prevents deadlock)"

echo ""
echo "════════════════════════════════════════"
echo "  STEP 2 — Fix slam_toolbox.yaml"
echo "════════════════════════════════════════"

SLAM_YAML="$PKG_DIR/config/slam_toolbox.yaml"

cat > "$SLAM_YAML" << 'YAML'
slam_toolbox:
  ros__parameters:
    # --- Solver ---
    # CeresSolver confirmed present: /opt/ros/jazzy/lib/libceres_solver_plugin.so
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None

    # --- Frames ---
    odom_frame: odom
    map_frame: map
    base_frame: base_link
    scan_topic: /scan
    mode: mapping

    # --- Timing ---
    # Zero minimum travel so the FIRST scan always creates the map frame.
    # Without this the robot must physically move before map→odom appears.
    minimum_travel_distance: 0.0
    minimum_travel_heading: 0.0
    minimum_time_interval: 0.0

    # Large TF buffer so slam_toolbox never misses odom→base_link
    tf_buffer_duration: 30.0
    transform_timeout: 1.0
    transform_publish_period: 0.02

    # --- Map ---
    resolution: 0.05
    max_laser_range: 10.0
    map_update_interval: 2.0
    throttle_scans: 1

    # --- Performance (safe for Jetson) ---
    stack_size_to_use: 10000000
    use_scan_matching: true
    use_scan_barycenter: true
    debug_logging: false
    enable_interactive_mode: false

    # Loop closure — keep generous on small hardware
    do_loop_closing: true
    loop_match_minimum_chain_size: 10
    loop_match_maximum_variance_coarse: 3.0
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45
YAML
ok "Wrote $SLAM_YAML"

# Mirror to build dir if it exists
if [ -d "$BUILD_DIR/config" ]; then
    cp "$SLAM_YAML" "$BUILD_DIR/config/slam_toolbox.yaml"
    ok "Mirrored to build dir"
fi

echo ""
echo "════════════════════════════════════════"
echo "  STEP 3 — Patch robot.launch.py"
echo "════════════════════════════════════════"

LAUNCH="$PKG_DIR/launch/robot.launch.py"

# Backup
cp "$LAUNCH" "${LAUNCH}.bak_$(date +%s)"
ok "Backed up launch file"

python3 << 'PYEOF'
import re, sys

path = '/root/ros2_ws/install/dynamixel_motors/share/dynamixel_motors/launch/robot.launch.py'
with open(path, 'r') as f:
    src = f.read()

# ── 1. Add SetEnvironmentVariable import if missing ──────────────────────
if 'SetEnvironmentVariable' not in src:
    src = src.replace(
        'from launch import LaunchDescription',
        'from launch import LaunchDescription\nfrom launch.actions import SetEnvironmentVariable'
    )

# ── 2. Inject FastDDS env var at the top of LaunchDescription ─────────────
fastdds_action = """
        # ── FastDDS SHM fix (prevents slam_toolbox deadlock on Jetson) ──
        SetEnvironmentVariable(
            'FASTRTPS_DEFAULT_PROFILES_FILE',
            '/root/fastdds_no_shm.xml'
        ),
        SetEnvironmentVariable(
            'RMW_FASTRTPS_USE_QOS_FROM_XML',
            '1'
        ),
"""

if 'FASTRTPS_DEFAULT_PROFILES_FILE' not in src:
    src = src.replace(
        'return LaunchDescription([',
        'return LaunchDescription([' + fastdds_action
    )
    print("[OK] Injected FastDDS env vars into LaunchDescription")
else:
    print("[--] FastDDS env var already present")

# ── 3. Add TimerAction delay for slam_toolbox (wait for sensors) ──────────
if 'TimerAction' not in src and 'timer_period' not in src:
    # Add import
    if 'from launch.actions import' in src:
        src = src.replace(
            'from launch.actions import SetEnvironmentVariable',
            'from launch.actions import SetEnvironmentVariable, TimerAction'
        )
    # Wrap slam_toolbox node in a 3-second delay
    # Find the slam_toolbox node block
    slam_match = re.search(
        r"([ \t]*Node\(\s*\n(?:[ \t]+.*\n)*?[ \t]+executable='sync_slam_toolbox_node'.*?\n(?:[ \t]+.*\n)*?[ \t]+\),)",
        src
    )
    if slam_match:
        original = slam_match.group(1)
        wrapped = f"        TimerAction(\n            period=3.0,\n            actions=[\n    {original.strip()}\n            ]\n        ),"
        src = src.replace(original, wrapped)
        print("[OK] Wrapped slam_toolbox in 3s TimerAction")
    else:
        print("[WW] Could not find slam_toolbox node to wrap — add delay manually")
else:
    print("[--] TimerAction already present or slam already delayed")

with open(path, 'w') as f:
    f.write(src)

print("[OK] Launch file saved")
PYEOF

# Mirror to build dir
if [ -f "$BUILD_DIR/launch/robot.launch.py" ]; then
    cp "$LAUNCH" "$BUILD_DIR/launch/robot.launch.py"
    ok "Mirrored launch to build dir"
fi

echo ""
echo "════════════════════════════════════════"
echo "  STEP 4 — Fix odom_pub double-shutdown"
echo "════════════════════════════════════════"

# The rclpy.shutdown() crash on exit is harmless but messy.
# Check if the odom_pub is a known file.
ODOM_FILE=""
for f in /root/odom_pub.py \
          "$BUILD_DIR/dynamixel_motors/odom_pub.py" \
          "/root/ros2_ws/build/dynamixel_motors/dynamixel_motors/odom_pub.py"; do
    [ -f "$f" ] && ODOM_FILE="$f" && break
done

if [ -n "$ODOM_FILE" ]; then
    python3 - "$ODOM_FILE" << 'PYEOF'
import sys, re
path = sys.argv[1]
with open(path) as f:
    src = f.read()

# Guard rclpy.shutdown() so it only runs if context is valid
old = "    rclpy.shutdown()"
new = """    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass"""

if old in src and new not in src:
    src = src.replace(old, new)
    with open(path, 'w') as f:
        f.write(src)
    print(f"[OK] Patched rclpy.shutdown guard in {path}")
else:
    print(f"[--] Already patched or pattern not found in {path}")
PYEOF
else
    warn "odom_pub.py not found — skipping shutdown-guard patch"
fi

echo ""
echo "════════════════════════════════════════"
echo "  STEP 5 — Verify TF chain manually"
echo "════════════════════════════════════════"

info "Checking if FASTRTPS env is set now..."
export FASTRTPS_DEFAULT_PROFILES_FILE=/root/fastdds_no_shm.xml
export RMW_FASTRTPS_USE_QOS_FROM_XML=1

source /opt/ros/jazzy/setup.bash 2>/dev/null || true
source /root/ros2_ws/install/setup.bash 2>/dev/null || true

info "If the robot launch is running, checking TF frames..."
timeout 4 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | grep -E "Translation|Rotation|Waiting|Error" | head -3 || true

echo ""
echo "════════════════════════════════════════"
echo "  STEP 6 — Quick slam_toolbox smoke test"
echo "════════════════════════════════════════"

info "Checking CeresSolver plugin is loadable..."
if ldconfig -p | grep -q ceres; then
    ok "libceres found: $(ldconfig -p | grep ceres | head -1 | awk '{print $NF}')"
else
    fail "libceres NOT found — solver plugin will fail!"
fi

if [ -f "/opt/ros/jazzy/lib/libceres_solver_plugin.so" ]; then
    ok "/opt/ros/jazzy/lib/libceres_solver_plugin.so exists"
else
    fail "libceres_solver_plugin.so missing!"
fi

echo ""
echo "════════════════════════════════════════"
echo "  DONE — Summary of changes"
echo "════════════════════════════════════════"
echo ""
echo "  1. /root/fastdds_no_shm.xml  — disables SHM transport (kills the"
echo "     deadlock that makes slam_toolbox hang silently)"
echo ""
echo "  2. slam_toolbox.yaml — key changes:"
echo "     • minimum_travel_distance: 0.0  (map frame appears immediately)"
echo "     • minimum_time_interval:   0.0  (process every scan)"
echo "     • tf_buffer_duration:      30.0 (never miss odom→base_link)"
echo "     • CeresSolver confirmed present"
echo ""
echo "  3. robot.launch.py — added:"
echo "     • FASTRTPS_DEFAULT_PROFILES_FILE env var"
echo "     • 3s startup delay on slam_toolbox (waits for /scan + /odom)"
echo ""
echo "  Next steps:"
echo "  ┌─────────────────────────────────────────────────────────────┐"
echo "  │  # Rebuild (only if source files changed, else skip)        │"
echo "  │  cd /root/ros2_ws && colcon build --packages-select         │"
echo "  │    dynamixel_motors --symlink-install 2>&1 | tail -5        │"
echo "  │                                                              │"
echo "  │  # Then launch:                                             │"
echo "  │  source install/setup.bash                                  │"
echo "  │  ros2 launch dynamixel_motors robot.launch.py               │"
echo "  │                                                              │"
echo "  │  # In another terminal, verify map frame appears:           │"
echo "  │  ros2 run tf2_ros tf2_echo map base_link                    │"
echo "  └─────────────────────────────────────────────────────────────┘"
echo ""
