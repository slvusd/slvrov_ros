#!/bin/bash

# ============================================================
# SLVROV Controller Pi — ROS2 Service Installer
#
# Installs one systemd service on the controller Pi (Pi46):
#
#   slvrov-controller  (enabled, auto-start on boot)
#                      joy_node_left  (device_id=0 → /joy_left)
#                      joy_node_right (device_id=1 → /joy_right)
#
# Usage:
#   ./install_controller.sh
#
# Assumes:
#   - Repo at /home/pi/slvrov_ros/
#   - Workspace already built: colcon build run inside that dir
#   - ROS2 Jazzy at /opt/ros/jazzy/
#   - Python venv at /home/pi/venv/
#   - Two USB joysticks connected as /dev/input/js0 and js1
# ============================================================

set -e

REPO="/home/pi/slvrov_ros"
USER="pi"

log()  { echo ""; echo "━━━ $1 ━━━"; }
ok()   { echo "  ✅ $1"; }
warn() { echo "  ⚠️  $1"; }
die()  { echo "  ❌ $1"; exit 1; }

[ "$(uname -s)" = "Linux" ]   || die "This script runs on the Pi only."
[ -d "$REPO" ]                 || die "Repo not found at $REPO"
[ -d "$REPO/install" ]         || die "Workspace not built — run 'colcon build' inside $REPO first."

service_install() {
    local name="$1"
    local unit="$2"
    echo ""
    echo "  Writing /etc/systemd/system/${name}.service..."
    echo "$unit" | sudo tee "/etc/systemd/system/${name}.service" > /dev/null
    sudo systemctl daemon-reload
    sudo systemctl enable  "$name"
    sudo systemctl restart "$name"
    sleep 2
    if systemctl is-active --quiet "$name"; then
        ok "$name is running."
    else
        warn "$name failed to start. Check: journalctl -u $name -n 30"
    fi
}

service_stop() {
    if systemctl is-active --quiet "$1" 2>/dev/null; then
        echo "  Stopping $1..."
        sudo systemctl stop "$1"
    fi
}

# ════════════════════════════════════════════════════════════
# slvrov-controller  (joy nodes)
# ════════════════════════════════════════════════════════════

log "slvrov-controller (joy_node_left + joy_node_right)"

WRAPPER="$REPO/start_controller_nodes.sh"
cat > "$WRAPPER" << 'WRAPPER_EOF'
#!/bin/bash
[ -f /opt/ros/jazzy/setup.bash ]              && source /opt/ros/jazzy/setup.bash
[ -f /home/pi/slvrov_ros/install/setup.bash ] && source /home/pi/slvrov_ros/install/setup.bash
[ -f /home/pi/venv/bin/activate ]             && source /home/pi/venv/bin/activate
export PYTHONPATH=/home/pi/venv/lib/python3.12/site-packages:${PYTHONPATH}
export ROS_DOMAIN_ID=42
[ -f /home/pi/fastdds_config.xml ] && export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi/fastdds_config.xml
export SLVROV_REPO=/home/pi/slvrov_ros
exec ros2 launch slvrov_nodes_python controller.launch.py
WRAPPER_EOF
chmod +x "$WRAPPER"
ok "Wrapper: $WRAPPER"

service_stop "slvrov-controller"

service_install "slvrov-controller" \
"[Unit]
Description=SLVROV Controller Nodes (joy_node_left + joy_node_right)
After=network.target

[Service]
Type=simple
User=${USER}
WorkingDirectory=${REPO}
ExecStart=${WRAPPER}
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target"

# ════════════════════════════════════════════════════════════
# Summary
# ════════════════════════════════════════════════════════════

echo ""
echo "════════════════════════════════════════"
echo " Controller install complete!"
echo "════════════════════════════════════════"
echo ""
echo "  Logs:"
echo "    journalctl -u slvrov-controller -f"
echo ""
echo "  Restart:"
echo "    sudo systemctl restart slvrov-controller"
echo ""
