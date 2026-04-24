# THIS IS A BACKUP OF MAIN BRANCH ON 04/24/2026 at 13:07 (1:07PM)
## Packages
- `slvrov_nodes_python` – ROS2 nodes written in python
- `slvrov_interfaces` – Messages and interfaces
- `slvrov_tools` – Shared python tools (submodule)

## Build
```bash
git clone --recurse-submodules https://github.com/slvusd/slvrov_ros.git
cd ./slvrov_ros/src/slvrov_tools_vendor/slvrov_tools
make
cd ./../../..
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

## Environment Setup?

## Launch
```bash
ros2 launch slvrov_launch launch.py
```
