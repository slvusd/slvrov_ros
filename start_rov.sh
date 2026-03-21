#!/bin/bash
source /opt/ros/jazzy/setup.bash
source ~/slvrov_ros/install/setup.bash
source ~/venv/bin/activate
export ROS_DOMAIN_ID=42

# Silence all background node output
ssh pi@192.168.3.46 "export ROS_DOMAIN_ID=42 && source /opt/ros/jazzy/setup.bash && source ~/slvrov_ros/install/setup.bash && source ~/venv/bin/activate && ros2 run joy joy_node --ros-args -r /joy:=/joy_left -p device_id:=0" > /dev/null 2>&1 &
ssh pi@192.168.3.46 "export ROS_DOMAIN_ID=42 && source /opt/ros/jazzy/setup.bash && source ~/slvrov_ros/install/setup.bash && source ~/venv/bin/activate && ros2 run joy joy_node --ros-args -r /joy:=/joy_right -p device_id:=1" > /dev/null 2>&1 &

ros2 run slvrov_nodes_python pca9685_pin_configs_server > /dev/null 2>&1 &
sleep 1
ros2 run slvrov_nodes_python pca9685_node > /dev/null 2>&1 &
ros2 run slvrov_nodes_python joystick_logic --ros-args -p mapping_file:=/home/pi/slvrov_ros/joy_mappings.yaml > /dev/null 2>&1 &
ros2 run slvrov_nodes_python thruster_bridge > /dev/null 2>&1 &

sleep 2
echo "All nodes started — monitoring /pca9685_command"
echo ""

# Poll and print one updating line
while true; do
    LINE=$(ros2 topic echo /pca9685_command --once 2>/dev/null | grep -A1 "pwm:" | tail -1)
    printf "\r PWM: %-60s" "$LINE"
    sleep 0.1
done
