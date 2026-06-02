.
|--src
|  |--slvrov_core_python
|  |--slvrov_interfaces
|  `--slvrov_launch
|
|--rov_config
|  |--motors
|  |--actions
|  |--controls
|  |--rovs

## slvrov_core_python
Contains:
* ROS2 control nodes
* flask servers
* mediamtx files
* UI for setup, pilot, and developer modes

Questions:
* Will is be structured the same as a traditional ROS2 package?
* How will we separate flask from mediamtx from ROS2 from UI structurally?
* How will we serve all UI assets and interface with the necessary ROS2 nodes from a single flask server?

## slvrov_interfaces
* stores custom interfaces
* split by node (e.g. folder within srv for JoystickMapper services)

## slvrov_launch
* stores launch files for each config in rov_configs/rovs