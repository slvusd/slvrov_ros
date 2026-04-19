# Summer 2026 Update - slvrov_ros Structure

## Packages

Currently, our repository is split into four ROS2 packages:
- slvrov_interfaces
- slvrov_launch
- slvrov_nodes_python
- slvrov_tools_vendor

This structure works, but it will become messy and convoluted as our codebase expands. 
Also, going into the future, if any nodes are in a package, the language name should be appended to the end of the package name.

### slvrov_interfaces

This package contains the custom interfaces needed for the ROV nodes. Therefore, it is currently a dependency of all ROV node packages. This package is already organized, and its purpose should not change. It just needs to me properly maintained:
- custom topic interfaces go in /msgs
- custom services fo in /srv
- services and interfaces are exposed in CMakeLists.txt

This will be something that stays the same in the Summer 2026 update.

### slvrov_launch

The purpose of this package is to hold all launch and launch configuration files for the ROV. This is the recommended way to start multiple nodes. This should be expanded on in the Summer 2026 update, as its functionality is currently not in use and has not been tested.

### slvrov_nodes_python

This package contains all of the nodes for Godzillah, including joystick logic and pca9685. However, if we're going to build off of this codebase in the future (as I would recommend, since starting from scratch every year sucks), basic nodes, like the ones that are involved in driving the ROV, will start to get cluttered with task/year-specific nodes. Therefore, it would be good practice to separate the core nodes into a separate ROS2 package and let task-specific be kept elsewhere.

New structure:
- slvrov_core_nodes_<language_name>
- slvrov_nodes_python

### slvrov_tools_vendor

Originally, when I was trying to use ROS2, I had set up the environment wrong and so I couldn't use external packages, and so I included the slvrov-tools python library as a submodule instead.
However, this issue has been resolved, so I think that it would be better to remove this submodule and just install slvrov-tools separately.