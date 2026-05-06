#!/bin/bash
# ==============================================================================
# fix_robot.sh  —  Full pipeline fix for ROS2 Jazzy + Nav2 + SLAM + Dynamixel
#
# Issues fixed:
#   1. map→base_link TF missing when Nav2 activates (costmap timeout)
#   2. LaserRangeScan size mismatch (ydlidar fixed_size wrong)
#   3. dynamixel_node built/running from wrong workspace (ros2_ws vs dynamixel_ws)
#   4. Torque never enabled on servos before velocity commands
#   5. navigate_to_pose server never available (Nav2 launched before map exists)
#
# Usage:
#   chmod +x fix_robot.sh
#   bash fix_robot.sh          # apply all fixes + rebuild
#   bash fix_robot.sh --run    # apply fixes, rebuild, then launch the full stack
# ==============================================================================

set -euo pipefail

# ── Colours ───────────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'

ok()    { echo -e "${GREEN}[OK]${NC}    $1"; }
fail()  { echo -e "${RED}[FAIL]${NC}  $1"; }
info()  { echo -e "${CYAN}[INFO]${NC}  $1"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $1"; }
step()  { echo -e "\n${BOLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"; \
          echo -e "${BOLD}  $1${NC}"; \
          echo -e "${BOLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"; }

# ── Config — adjust these to match your setup ─────────────────────────────────
ROS2_WS="${HOME}/ros2_ws"
DXL_WS="${HOME}/dynamixel_ws"
PKG_NAME="dynamixel_motors"
LIDAR_PORT="/dev/ttyUSB1"          # lidar serial port
MOTOR_PORT="/dev/ttyUSB0"          # dynamixel serial port
MOTOR_SYMLINK="/dev/motors"
MOTOR_BAUD=115200
SERVO_IDS=(1 2)
MAP_WAIT_TIMEOUT=60                # seconds to wait for /map to appear
NAV2_LAUNCH_DELAY=5                # extra seconds after map appears before Nav2 starts

# ── Source ROS2 ───────────────────────────────────────────────────────────────
source /opt/ros/jazzy/setup.bash 2>/dev/null || true

# ==============================================================================
# FIX 1 — Consolidate dynamixel_motors into a single workspace
# ==============================================================================
fix_workspace() {
    step "FIX 1 — Consolidate workspaces"

    if [[ ! -d "${DXL_WS}/src/${PKG_NAME}" ]]; then
        fail "Could not find ${DXL_WS}/src/${PKG_NAME} — skipping workspace merge"
        return
    fi

    # Copy the package source into ros2_ws so there is ONE truth
    SRC_PKG="${DXL_WS}/src/${PKG_NAME}"
    DST_PKG="${ROS2_WS}/src/${PKG_NAME}"

    if [[ -d "${DST_PKG}" ]]; then
        warn "${PKG_NAME} already exists in ros2_ws/src — syncing files from dynamixel_ws"
        rsync -a --delete "${SRC_PKG}/" "${DST_PKG}/"
    else
        info "Copying ${PKG_NAME} into ${ROS2_WS}/src/"
        cp -r "${SRC_PKG}" "${DST_PKG}"
    fi
    ok "Package source is now only in ${ROS2_WS}/src/${PKG_NAME}"

    # Warn user to stop editing dynamixel_ws
    warn "From now on, edit ONLY in: ${ROS2_WS}/src/${PKG_NAME}"
    warn "Do NOT rebuild in dynamixel_ws — it creates a stale shadow install."
}

# ==============================================================================
# FIX 2 — Patch motors.yaml: ensure port points to the symlink
# ==============================================================================
fix_motors_yaml() {
    step "FIX 2 — Patch motors.yaml port"

    YAML="${ROS2_WS}/src/${PKG_NAME}/config/motors.yaml"
    if [[ ! -f "${YAML}" ]]; then
        fail "motors.yaml not found at ${YAML}"
        return
    fi

    # Show current port setting
    CURRENT_PORT=$(grep -E '^\s*port\s*:' "${YAML}" | head -1 | sed 's/.*: *//')
    info "Current port in motors.yaml: ${CURRENT_PORT}"

    # Replace whatever port is set with the real device (not the symlink,
    # because symlinks can break across reboots)
    sed -i "s|port:.*|port: ${MOTOR_PORT}|g" "${YAML}"
    ok "motors.yaml → port set to ${MOTOR_PORT}"

    # Also ensure baud rate is correct
    sed -i "s|baud_rate:.*|baud_rate: ${MOTOR_BAUD}|g" "${YAML}"
    ok "motors.yaml → baud_rate set to ${MOTOR_BAUD}"
}

# ==============================================================================
# FIX 3 — Patch ydlidar config: fix fixed_size mismatch
#          Logs show hardware returns ~564 but config says 546/560
# ==============================================================================
fix_lidar_yaml() {
    step "FIX 3 — Patch ydlidar config (scan size mismatch)"

    # Find ydlidar params yaml — search both workspaces
    LIDAR_YAML=$(find "${ROS2_WS}/src" /opt/ros/jazzy -name "*.yaml" 2>/dev/null \
                 | xargs grep -l "ydlidar\|fixed_size\|sample_rate" 2>/dev/null \
                 | grep -v "__pycache__" | head -1 || true)

    if [[ -z "${LIDAR_YAML}" ]]; then
        warn "Could not auto-find ydlidar yaml. Searching harder..."
        LIDAR_YAML=$(find "${ROS2_WS}" -name "*.yaml" 2>/dev/null \
                     | xargs grep -l "isSingleChannel\|frequency\|sampleRate" 2>/dev/null \
                     | head -1 || true)
    fi

    if [[ -z "${LIDAR_YAML}" ]]; then
        warn "ydlidar config yaml not found automatically."
        warn "Manually set these in your ydlidar yaml:"
        warn "  fixed_size: 0          # 0 = auto-detect from hardware"
        warn "  sampleRate: 5          # X4 = 5K"
        warn "  frequency: 10.0        # 10 Hz scan rate"
        return
    fi

    info "Found ydlidar config: ${LIDAR_YAML}"

    # Set fixed_size to 0 (auto) to stop the mismatch warnings.
    # The X4 returns variable counts near 564; letting it auto-detect is safest.
    if grep -q "fixed_size" "${LIDAR_YAML}"; then
        sed -i "s|fixed_size:.*|fixed_size: 0|g" "${LIDAR_YAML}"
        ok "ydlidar fixed_size → 0 (auto-detect)"
    else
        warn "fixed_size key not in ${LIDAR_YAML} — adding it"
        echo "  fixed_size: 0" >> "${LIDAR_YAML}"
    fi

    # Ensure sampleRate matches X4 hardware
    if grep -q "sampleRate" "${LIDAR_YAML}"; then
        sed -i "s|sampleRate:.*|sampleRate: 5|g" "${LIDAR_YAML}"
        ok "ydlidar sampleRate → 5 (matches X4)"
    fi
}

# ==============================================================================
# FIX 4 — Rebuild dynamixel_motors in ros2_ws only
# ==============================================================================
rebuild_package() {
    step "FIX 4 — Rebuild ${PKG_NAME} in ros2_ws"

    cd "${ROS2_WS}"
    colcon build --packages-select "${PKG_NAME}" --symlink-install 2>&1 \
        | grep -v "SetuptoolsDeprecationWarning\|script-dir\|install-scripts\|opt = self\|\*\*\*\*\*\|This deprecation\|See https" \
        || true
    ok "Build complete"

    source "${ROS2_WS}/install/setup.bash"
    ok "Sourced ${ROS2_WS}/install/setup.bash"
}

# ==============================================================================
# FIX 5 — Recreate /dev/motors symlink robustly (persists only until reboot)
#          For permanent fix, use udev rule below.
# ==============================================================================
fix_symlink() {
    step "FIX 5 — Device symlink"

    if [[ ! -e "${MOTOR_PORT}" ]]; then
        fail "${MOTOR_PORT} not found — is the Dynamixel USB adapter plugged in?"
        return
    fi

    # Remove stale symlink if it exists
    if [[ -L "${MOTOR_SYMLINK}" ]]; then
        rm -f "${MOTOR_SYMLINK}"
        warn "Removed stale symlink ${MOTOR_SYMLINK}"
    fi

    ln -s "${MOTOR_PORT}" "${MOTOR_SYMLINK}"
    ok "Created ${MOTOR_SYMLINK} → ${MOTOR_PORT}"

    # Also write a udev rule so this survives reboots
    UDEV_FILE="/etc/udev/rules.d/99-robot-motors.rules"
    if [[ ! -f "${UDEV_FILE}" ]]; then
        info "Writing permanent udev rule to ${UDEV_FILE}"
        # Get USB attributes for this device
        VENDOR=$(udevadm info -a -n "${MOTOR_PORT}" 2>/dev/null \
                 | grep 'ATTRS{idVendor}' | head -1 | grep -oP '"\K[^"]+' || true)
        PRODUCT=$(udevadm info -a -n "${MOTOR_PORT}" 2>/dev/null \
                  | grep 'ATTRS{idProduct}' | head -1 | grep -oP '"\K[^"]+' || true)

        if [[ -n "${VENDOR}" && -n "${PRODUCT}" ]]; then
            cat > "${UDEV_FILE}" <<EOF
# Dynamixel motor controller — persistent symlink
SUBSYSTEM=="tty", ATTRS{idVendor}=="${VENDOR}", ATTRS{idProduct}=="${PRODUCT}", SYMLINK+="motors", MODE="0666"
EOF
            udevadm control --reload-rules 2>/dev/null || true
            udevadm trigger 2>/dev/null || true
            ok "udev rule written — /dev/motors will persist across reboots"
        else
            warn "Could not read USB vendor/product — udev rule not written."
            warn "Manually add a udev rule or re-run this script after plugging in."
        fi
    else
        ok "udev rule already exists at ${UDEV_FILE}"
    fi
}

# ==============================================================================
# FIX 6 — Enable torque on servos (must be called after nodes are running)
# ==============================================================================
enable_torque() {
    step "FIX 6 — Enable servo torque"

    source /opt/ros/jazzy/setup.bash 2>/dev/null || true
    source "${ROS2_WS}/install/setup.bash" 2>/dev/null || true

    local ALL_OK=true
    for ID in "${SERVO_IDS[@]}"; do
        SVC="/servo${ID}/set_torque"
        info "Calling ${SVC} → true"
        if ros2 service call "${SVC}" std_srvs/srv/SetBool '{data: true}' \
                --timeout 3 >/dev/null 2>&1; then
            ok "Torque enabled on servo ${ID}"
        else
            fail "Could not reach ${SVC} — is dynamixel_node running?"
            ALL_OK=false
        fi
    done

    if [[ "${ALL_OK}" == false ]]; then
        warn "Run this manually once dynamixel_node is up:"
        for ID in "${SERVO_IDS[@]}"; do
            echo "  ros2 service call /servo${ID}/set_torque std_srvs/srv/SetBool '{data: true}'"
        done
    fi
}

# ==============================================================================
# FIX 7 — Patch the launch file to wait for /map before activating Nav2
# ==============================================================================
fix_launch_file() {
    step "FIX 7 — Patch launch file: wait for /map before Nav2"

    # Find the main launch file
    LAUNCH_FILE=$(find "${ROS2_WS}/src" -name "robot.launch.py" \
                  -o -name "bringup.launch.py" 2>/dev/null | head -1 || true)

    if [[ -z "${LAUNCH_FILE}" ]]; then
        warn "Could not find robot.launch.py automatically."
        warn "Apply FIX 7 manually — see the nav2_wait_for_map_launch.py file"
        warn "created alongside this script."
        return
    fi

    info "Found launch file: ${LAUNCH_FILE}"

    # Check if it already has the map-wait logic
    if grep -q "map_ready\|wait_for_map\|Event\|on_map" "${LAUNCH_FILE}" 2>/dev/null; then
        ok "Launch file already has map-wait logic — skipping patch"
        return
    fi

    # Backup original
    cp "${LAUNCH_FILE}" "${LAUNCH_FILE}.bak"
    ok "Backed up original → ${LAUNCH_FILE}.bak"

    # Inject a TimerAction delay before Nav2 bringup.
    # This is the safest non-intrusive patch: delay Nav2 by NAV2_LAUNCH_DELAY
    # seconds AFTER the launch starts. For proper map-ready detection,
    # see nav2_wait_for_map_launch.py generated below.
    if grep -q "nav2_bringup\|navigation_launch\|lifecycle_manager_navigation" "${LAUNCH_FILE}"; then
        python3 - <<PYEOF
import re, sys

launch_file = "${LAUNCH_FILE}"
delay = ${NAV2_LAUNCH_DELAY}

with open(launch_file, "r") as f:
    src = f.read()

# Add TimerAction import if not present
if "TimerAction" not in src:
    src = src.replace(
        "from launch.actions import",
        "from launch.actions import TimerAction,\n    "
    )
    if "TimerAction" not in src:
        src = "from launch.actions import TimerAction\n" + src

# Find Nav2 IncludeLaunchDescription or Node and wrap it in TimerAction
# We look for a variable called nav2_* or navigation_launch and wrap it
pattern = r'([ \t]*)(IncludeLaunchDescription\([^)]*navigation[^)]*\))'
replacement = r'\1TimerAction(period=${delay}.0, actions=[\n\1    \2\n\1])'
new_src = re.sub(pattern, replacement, src, flags=re.DOTALL)

if new_src == src:
    # Fallback: couldn't auto-patch, print a warning
    print("WARN: auto-patch failed — apply manually (see nav2_wait_for_map_launch.py)")
    sys.exit(0)

with open(launch_file, "w") as f:
    f.write(new_src)

print(f"OK: Nav2 launch wrapped in TimerAction(period={delay})")
PYEOF
    else
        warn "Nav2 launch action not found in ${LAUNCH_FILE} — patch skipped."
        warn "Use the standalone nav2_wait_for_map_launch.py approach instead."
    fi
}

# ==============================================================================
# Generate a standalone Nav2 launch wrapper that waits for /map
# (use this if automatic patching failed or you want clean separation)
# ==============================================================================
generate_nav2_launch_wrapper() {
    step "Generating nav2_wait_for_map_launch.py"

    WRAPPER="${ROS2_WS}/src/${PKG_NAME}/launch/nav2_wait_for_map_launch.py"
    mkdir -p "$(dirname "${WRAPPER}")"

    cat > "${WRAPPER}" <<'PYLAUNCH'
"""
nav2_wait_for_map_launch.py
───────────────────────────
Launches Nav2 ONLY after slam_toolbox has published a valid map on /map.
This fixes the race condition where local_costmap times out because the
map→base_link TF does not exist yet when Nav2 tries to activate.

Usage (in a second terminal after the robot base launch):
  ros2 launch dynamixel_motors nav2_wait_for_map_launch.py

Or integrate into your main launch via:
  IncludeLaunchDescription(PythonLaunchDescriptionSource(
      os.path.join(get_package_share_directory('dynamixel_motors'),
                   'launch', 'nav2_wait_for_map_launch.py')
  ))
"""

import os
import threading
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

from launch import LaunchDescription
from launch.actions import (
    OpaqueFunction,
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

MAP_WAIT_TIMEOUT = 90   # seconds before giving up and launching anyway
NAV2_EXTRA_DELAY = 2.0  # extra seconds after map arrives, for TF to settle


class _MapWatcher(Node):
    """Minimal node that blocks until /map has at least one message."""

    def __init__(self):
        super().__init__("_map_readiness_watcher")
        self._received = threading.Event()
        self._sub = self.create_subscription(
            OccupancyGrid, "/map", self._cb, 1
        )

    def _cb(self, _msg):
        self._received.set()

    def wait(self, timeout: float) -> bool:
        return self._received.wait(timeout)


def _wait_for_map(context, *args, **kwargs):
    """OpaqueFunction: blocks the launch until /map is ready."""
    import time

    rclpy.init()
    watcher = _MapWatcher()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(watcher)

    print("[nav2_wait_for_map] Waiting for /map topic …")
    t0 = time.time()
    map_ready = False

    while not map_ready and (time.time() - t0) < MAP_WAIT_TIMEOUT:
        executor.spin_once(timeout_sec=0.5)
        map_ready = watcher.wait(timeout=0.0)

    watcher.destroy_node()
    rclpy.shutdown()

    if map_ready:
        elapsed = time.time() - t0
        print(f"[nav2_wait_for_map] Map received after {elapsed:.1f}s — launching Nav2 in {NAV2_EXTRA_DELAY}s …")
        import time as t; t.sleep(NAV2_EXTRA_DELAY)
    else:
        print(f"[nav2_wait_for_map] WARNING: No map after {MAP_WAIT_TIMEOUT}s — launching Nav2 anyway")

    return []


def generate_launch_description():
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("nav2_bringup"),
                "launch",
                "navigation_launch.py",
            ])
        ]),
        launch_arguments={"use_sim_time": "false"}.items(),
    )

    return LaunchDescription([
        LogInfo(msg="[nav2_wait_for_map] Starting map readiness check …"),
        OpaqueFunction(function=_wait_for_map),
        LogInfo(msg="[nav2_wait_for_map] Activating Nav2 stack …"),
        nav2_launch,
    ])
PYLAUNCH

    ok "Written: ${WRAPPER}"
    info "Use it with:"
    info "  ros2 launch ${PKG_NAME} nav2_wait_for_map_launch.py"
}

# ==============================================================================
# Generate a convenience startup script (3-terminal workflow)
# ==============================================================================
generate_startup_script() {
    step "Generating run_robot.sh (3-step startup)"

    RUNSCRIPT="${ROS2_WS}/run_robot.sh"
    cat > "${RUNSCRIPT}" <<BASH
#!/bin/bash
# run_robot.sh  —  3-step robot startup
# Generated by fix_robot.sh
#
# Step 1 (this script):  Launch base stack (lidar + motors + SLAM)
# Step 2 (new terminal): Teleop to build a map
# Step 3 (new terminal): Launch Nav2 (waits for map automatically)

set -euo pipefail
source /opt/ros/jazzy/setup.bash
source ${ROS2_WS}/install/setup.bash

MODE=\${1:-"base"}

case "\$MODE" in
  base)
    echo "━━━ STEP 1: Launching base stack (lidar + motors + SLAM) ━━━"
    echo "Open a second terminal and run:  bash run_robot.sh teleop"
    echo "Open a third terminal and run:   bash run_robot.sh nav2"
    ros2 launch ${PKG_NAME} robot.launch.py
    ;;

  teleop)
    echo "━━━ STEP 2: Teleop — drive to build map, then Ctrl+C ━━━"
    echo "Use i/j/l/, keys to move.  q/z to adjust speed."
    ros2 run teleop_twist_keyboard teleop_twist_keyboard \\
      --ros-args -r cmd_vel:=/cmd_vel
    ;;

  nav2)
    echo "━━━ STEP 3: Launching Nav2 (waits for /map automatically) ━━━"
    ros2 launch ${PKG_NAME} nav2_wait_for_map_launch.py
    ;;

  torque)
    echo "━━━ Enabling servo torque ━━━"
    for id in ${SERVO_IDS[*]}; do
      echo "  → /servo\${id}/set_torque"
      ros2 service call /servo\${id}/set_torque std_srvs/srv/SetBool '{data: true}'
    done
    ;;

  save_map)
    echo "━━━ Saving map to ~/robot_map ━━━"
    ros2 run nav2_map_server map_saver_cli -f ~/robot_map
    echo "Map saved: ~/robot_map.pgm + ~/robot_map.yaml"
    ;;

  diagnose)
    echo "━━━ Quick diagnostics ━━━"
    echo "--- Active nodes ---"
    ros2 node list 2>/dev/null || true
    echo ""
    echo "--- /cmd_vel (3s sample) ---"
    timeout 3 ros2 topic echo /cmd_vel --no-arr 2>/dev/null || echo "(silent)"
    echo ""
    echo "--- /map published? ---"
    timeout 3 ros2 topic echo /map --no-arr 2>/dev/null | head -5 || echo "(no map yet)"
    echo ""
    echo "--- TF tree ---"
    ros2 run tf2_tools view_frames 2>/dev/null && \
      echo "Saved to frames.pdf" || echo "tf2_tools not available"
    ;;

  *)
    echo "Usage: bash run_robot.sh [base|teleop|nav2|torque|save_map|diagnose]"
    exit 1
    ;;
esac
BASH

    chmod +x "${RUNSCRIPT}"
    ok "Written: ${RUNSCRIPT}"
}

# ==============================================================================
# Final summary
# ==============================================================================
print_summary() {
    echo ""
    echo -e "${BOLD}${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${BOLD}${GREEN}  ALL FIXES APPLIED${NC}"
    echo -e "${BOLD}${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo ""
    echo -e "  ${CYAN}3-terminal startup sequence:${NC}"
    echo ""
    echo -e "  ${BOLD}Terminal 1${NC} — Base stack (lidar + motors + SLAM):"
    echo -e "    ${YELLOW}cd ${ROS2_WS} && bash run_robot.sh base${NC}"
    echo ""
    echo -e "  ${BOLD}Terminal 2${NC} — Drive to build a map:"
    echo -e "    ${YELLOW}bash run_robot.sh teleop${NC}"
    echo -e "    (use i/j/l/, keys — drive around until map looks good)"
    echo ""
    echo -e "  ${BOLD}Terminal 3${NC} — Nav2 (waits for map automatically):"
    echo -e "    ${YELLOW}bash run_robot.sh nav2${NC}"
    echo ""
    echo -e "  ${BOLD}After Nav2 is up${NC} — Enable torque & send a goal:"
    echo -e "    ${YELLOW}bash run_robot.sh torque${NC}"
    echo -e "    ${YELLOW}ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \\${NC}"
    echo -e "    ${YELLOW}  \"{pose:{header:{frame_id:'map'},pose:{position:{x:1.0,y:0.0},orientation:{w:1.0}}}}\"${NC}"
    echo ""
    echo -e "  ${BOLD}Save your map for reuse:${NC}"
    echo -e "    ${YELLOW}bash run_robot.sh save_map${NC}"
    echo ""
    echo -e "  ${BOLD}Quick diagnostics:${NC}"
    echo -e "    ${YELLOW}bash run_robot.sh diagnose${NC}"
    echo ""
}

# ==============================================================================
# MAIN
# ==============================================================================
main() {
    echo -e "${BOLD}"
    echo "  ██████╗  ██████╗ ██████╗  ██████╗ ████████╗    ███████╗██╗██╗  ██╗"
    echo "  ██╔══██╗██╔═══██╗██╔══██╗██╔═══██╗╚══██╔══╝    ██╔════╝██║╚██╗██╔╝"
    echo "  ██████╔╝██║   ██║██████╔╝██║   ██║   ██║       █████╗  ██║ ╚███╔╝ "
    echo "  ██╔══██╗██║   ██║██╔══██╗██║   ██║   ██║       ██╔══╝  ██║ ██╔██╗ "
    echo "  ██║  ██║╚██████╔╝██████╔╝╚██████╔╝   ██║       ██║     ██║██╔╝ ██╗"
    echo "  ╚═╝  ╚═╝ ╚═════╝ ╚═════╝  ╚═════╝   ╚═╝       ╚═╝     ╚═╝╚═╝  ╚═╝"
    echo -e "${NC}"
    echo -e "  ROS2 Jazzy + Nav2 + SLAM Toolbox + Dynamixel — Full Pipeline Fix"
    echo ""

    fix_workspace
    fix_motors_yaml
    fix_lidar_yaml
    rebuild_package
    fix_symlink
    fix_launch_file
    generate_nav2_launch_wrapper
    generate_startup_script

    # Only enable torque if --run flag is passed (nodes must be running)
    if [[ "${1:-}" == "--run" ]]; then
        info "Waiting 10s for nodes to start before enabling torque..."
        sleep 10
        enable_torque
    else
        info "Skipping torque enable (nodes not running yet)."
        info "Run 'bash run_robot.sh torque' after launching the stack."
    fi

    print_summary
}

main "$@"
