# SLVROV ROS 2 Stack — Testing Tips

## Prerequisites and Setting Up SLVROS Workspace
Before testing, ensure that the workspace is setup correctly and everything is installed. Ensure that that the steps in the setup document have been followed.

If you are returning to a setup worksapce, install any updates and rebuild the system. This must be done before pushing changes and is good practice to keep up to date during testing.

```bash
ssh pi@192.168.3.21

cd ./slvrov_ros
git pull

colcon build
source ./install/setup.bash
```

## Listing Packages
One of the most basic tests. This is useful after building a new package.
```bash
ros2 pkg list
```
Pipe to ``` grep slvrov``` for showing slvrov_ros packages.

## Listing Interfaces
One of the most basic tests. This is useful after creating new interfaces.
```bash
ros2 interface package <package_name>
```
This will list all of the interfaces in a specific package.

## Launching Nodes
This is useful for catching basic errors in the launching process of nodes.

To launch an ROS2 node from the terminal, use
```bash
ros2 run <package name> <node name>
```

Example:
```bash
ros2 run slvrov_nodes_python pca9685_node
```

## Probing Nodes
ROS2 provides various command line tools for testing interactions of nodes without having to create your own node.

### Publishing Synthetic Messages
ROS2 can create a publisher that publishes a specified message at a certain frequency.
This is useful for testing subscribers.

To publish a synthetic message, use
```bash
ros2 topic pub <topic name> <interface name> <message>
```

Example:
```bash
ros2 topic pub /pca9685_command slvrov_interfaces/msg/PCA9685Command "{id: ['my_servo'], pwm: [0.0]}"
```

### Subscribing to Topics
ROS2 can subscribe to a specified topic.
This is useful for testing publishers to see if they are publishing the correct messages and they are being recieved.

To subcribe to a topic, use
```bash
ros2 topic echo <topic name>
```

Example:
```bash
ros2 topic echo /pca9685_command
```

### Requesting a Service
ROS2 can, similar to ```ros2 topic pub```, request services.
This is useful for testing servers without having to build your own client.

To request a service, use
```bash
ros2 service call <service name> <interface name> <request>
```

Example;
```bash
ros2 service call /add_pca9685_pin_configs slvrov_interfaces/srv/AddPCA9685PinConfigs "{id: 'servo15', pins: [15], minimum: 1000, default: 1500, maximum: 2000}"
```

ROS2 will also display the response from the server.

## /rosout Topic
Whenever ROS2 runs a node, it will start a few default topics and services.
One of these topic is ```/rosout```. This topic will print out the log messages from all running nodes' loggers.
This is useful for monitoring multiple nodes at once and debugging system-wide communication.

Use
```bash
ros2 topic echo /pca9685_command
```

This is commonly used with the ``` /rosout``` topic, which is is subscribed to all of the loggers. It can be very useful for debugging system-wide communication.

