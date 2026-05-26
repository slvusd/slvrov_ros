
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

## Documentation
- [ROS 2 Python Style Guide](system_documents/ros2_python_style_guide.md) - style guidance for `slvrov_core_python` Python code.

## Launch
```bash
ros2 launch slvrov_launch launch.py
```
