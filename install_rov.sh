#!/bin/bash

# ============================================================
# SLVROV ROV Pi — ROS2 Service Installer
#
# Installs two systemd services on the ROV Pi (Pi52, etc.):
#
#   slvrov-rov      (enabled, auto-start on boot)
#                   pca9685_pin_configs_server
#                   joystick_logic
#                   thruster_bridge
#
#   slvrov-pca9685  (registered but DISABLED — toggle via stats UI)
#                   pca9685_node (actual hardware driver)
#
# Also writes a sudoers rule so stats.py can start/stop
# slvrov-pca9685 without a password prompt.
#
# Usage:
#   ./install_rov.sh
#
# Assumes:
#   - Repo at /home/pi/slvrov_ros/
#   - Workspace already built: colcon build run inside that dir
#   - ROS2 Jazzy at /opt/ros/jazzy/
#   - Python venv at /home/pi/venv/
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
# 1.  slvrov-rov  (always-on core nodes)
# ════════════════════════════════════════════════════════════

log "slvrov-rov (pin configs server + joystick logic + thruster bridge)"

WRAPPER_ROV="$REPO/start_rov_nodes.sh"
cat > "$WRAPPER_ROV" << 'WRAPPER_EOF'
#!/bin/bash
[ -f /opt/ros/jazzy/setup.bash ]              && source /opt/ros/jazzy/setup.bash
[ -f /home/pi/slvrov_ros/install/setup.bash ] && source /home/pi/slvrov_ros/install/setup.bash
[ -f /home/pi/venv/bin/activate ]             && source /home/pi/venv/bin/activate
export PYTHONPATH=/home/pi/venv/lib/python3.12/site-packages:${PYTHONPATH}
export ROS_DOMAIN_ID=42
[ -f /home/pi/fastdds_config.xml ] && export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi/fastdds_config.xml
export SLVROV_REPO=/home/pi/slvrov_ros
exec ros2 launch slvrov_nodes_python rov.launch.py
WRAPPER_EOF
chmod +x "$WRAPPER_ROV"
ok "Wrapper: $WRAPPER_ROV"

service_stop "slvrov-rov"

service_install "slvrov-rov" \
"[Unit]
Description=SLVROV ROV Core Nodes (pin configs, joystick logic, thruster bridge)
After=network.target

[Service]
Type=simple
User=${USER}
WorkingDirectory=${REPO}
ExecStartPre=/bin/sleep 5
ExecStart=${WRAPPER_ROV}
Restart=on-failure
RestartSec=10

[Install]
WantedBy=multi-user.target"

# ════════════════════════════════════════════════════════════
# 2.  slvrov-pca9685  (hardware driver — disabled by default)
# ════════════════════════════════════════════════════════════

log "slvrov-pca9685 (pca9685 hardware driver — toggleable via stats UI)"

WRAPPER_PCA="$REPO/start_pca9685.sh"
cat > "$WRAPPER_PCA" << 'WRAPPER_EOF'
#!/bin/bash
[ -f /opt/ros/jazzy/setup.bash ]              && source /opt/ros/jazzy/setup.bash
[ -f /home/pi/slvrov_ros/install/setup.bash ] && source /home/pi/slvrov_ros/install/setup.bash
[ -f /home/pi/venv/bin/activate ]             && source /home/pi/venv/bin/activate
export PYTHONPATH=/home/pi/venv/lib/python3.12/site-packages:${PYTHONPATH}
export ROS_DOMAIN_ID=42
[ -f /home/pi/fastdds_config.xml ] && export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi/fastdds_config.xml
exec ros2 run slvrov_nodes_python pca9685_node
WRAPPER_EOF
chmod +x "$WRAPPER_PCA"
ok "Wrapper: $WRAPPER_PCA"

service_stop "slvrov-pca9685"

echo ""
echo "  Writing /etc/systemd/system/slvrov-pca9685.service..."
sudo tee "/etc/systemd/system/slvrov-pca9685.service" > /dev/null << EOF
[Unit]
Description=SLVROV PCA9685 Hardware Driver (thrusters and servos)
After=slvrov-rov.service

[Service]
Type=simple
User=${USER}
WorkingDirectory=${REPO}
ExecStart=${WRAPPER_PCA}
Restart=no

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl disable slvrov-pca9685 2>/dev/null || true
ok "slvrov-pca9685 registered and disabled (won't start on boot)."
ok "Toggle via stats UI or: sudo systemctl start slvrov-pca9685"

# ════════════════════════════════════════════════════════════
# 3.  sudoers  — let stats.py (running as pi) toggle pca9685
# ════════════════════════════════════════════════════════════

log "sudoers — allow pi to start/stop slvrov-pca9685 without password"

SUDOERS_FILE="/etc/sudoers.d/slvrov-pca9685"
echo "pi ALL=(ALL) NOPASSWD: /bin/systemctl start slvrov-pca9685, /bin/systemctl stop slvrov-pca9685" \
    | sudo tee "$SUDOERS_FILE" > /dev/null
sudo chmod 0440 "$SUDOERS_FILE"
ok "Sudoers rule: $SUDOERS_FILE"

# ════════════════════════════════════════════════════════════
# Summary
# ════════════════════════════════════════════════════════════

echo ""
echo "════════════════════════════════════════"
echo " ROV install complete!"
echo "════════════════════════════════════════"
echo ""
echo "  Always-on nodes:"
echo "    journalctl -u slvrov-rov -f"
echo "    sudo systemctl restart slvrov-rov"
echo ""
echo "  Thruster hardware (toggle from stats UI or manually):"
echo "    sudo systemctl start slvrov-pca9685"
echo "    sudo systemctl stop  slvrov-pca9685"
echo "    journalctl -u slvrov-pca9685 -f"
echo ""
