# SLVROV ROS 2 Stack

## Packages
- `slvrov_nodes_python` – ROS2 nodes written in python
- `slvrov_interfaces` – Messages and interfaces
- `slvrov_tools` – Shared python tools (submodule)

## Build
```bash
git clone --recurse-submodules https://github.com/LegionaryOfLogic/slvrov_ros.git
cd ./slvrov_ros/src/slvrov_tools_vendor/slvrov_tools
make
cd ./../../..
colcon build
source install/setup.bash
