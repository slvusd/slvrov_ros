
## Packages
- `slvrov_core_python` – ROS2 control nodes, Flask/UI scaffolding, MediaMTX support, and web adapters
- `slvrov_interfaces` – Messages and interfaces
- `slvrov_launch` – Launch files for approved ROV configs
- `slvrov_tools` – Shared Python tools installed into the active virtual environment

## Build
```bash
python3 -m venv venv
source venv/bin/activate
pip install empy==3.3.4 catkin-pkg lark pyparsing
git clone https://github.com/LegionaryOfLogic/slvrov-tools.git
cd slvrov-tools
make
pip install .
cd ..
git clone https://github.com/slvusd/slvrov_ros.git
cd slvrov_ros
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

## Environment Setup?

## Documentation
- [ROS 2 Python Style Guide](system_documents/ros2_python_style_guide.md) - style guidance for `slvrov_core_python` Python code.

## Launch
```bash
ros2 launch slvrov_launch two_joystick_launch.py
```
