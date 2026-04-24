#!/bin/bash
[ -f /opt/ros/jazzy/setup.bash ]              && source /opt/ros/jazzy/setup.bash
[ -f /home/pi/slvrov_ros/install/setup.bash ] && source /home/pi/slvrov_ros/install/setup.bash
[ -f /home/pi/venv/bin/activate ]             && source /home/pi/venv/bin/activate
export PYTHONPATH=/home/pi/venv/lib/python3.12/site-packages:${PYTHONPATH}
export ROS_DOMAIN_ID=42
[ -f /home/pi/fastdds_config.xml ] && export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pi/fastdds_config.xml
exec ros2 run slvrov_nodes_python pca9685_node
