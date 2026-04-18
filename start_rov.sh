
#!/bin/bash
echo "Cleaning up previous session..."
pkill -f slvrov_nodes_python
sleep 3

# also kill any leftover joy nodes on the surface pi
ssh pi@192.168.3.46 "pkill -f joy_node" 2>/dev/null || true
sleep 1

echo "Starting nodes..."
# ... rest of script
# Environment setup on Pi41 (ROV)
export ROS_DOMAIN_ID=42
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi/fastdds_config.xml

source /opt/ros/jazzy/setup.bash
source ~/slvrov_ros/install/setup.bash
source ~/venv/bin/activate

# Launch joy nodes on Pi46 (surface)
ssh pi@192.168.3.46 "
  export ROS_DOMAIN_ID=42
  export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi/fastdds_config.xml
  source /opt/ros/jazzy/setup.bash
  source ~/slvrov_ros/install/setup.bash
  source ~/venv/bin/activate
  ros2 run joy joy_node --ros-args -r /joy:=/joy_left -p device_id:=0 &
  ros2 run joy joy_node --ros-args -r /joy:=/joy_right -p device_id:=1 &
" &

sleep 1

# Launch ROV nodes on Pi41
ros2 run slvrov_nodes_python pca9685_pin_configs_server &
sleep 1
ros2 run slvrov_nodes_python pca9685_node &
ros2 run slvrov_nodes_python joystick_logic --ros-args -p mapping_file:=/home/pi/slvrov_ros/joy_mappings.yaml &
ros2 run slvrov_nodes_python thruster_bridge &

sleep 2
echo "All nodes started - monitoring /pca9685_command"
echo ""

