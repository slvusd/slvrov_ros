# SLVROV ROS 2 Stack

This is a system that uses ROS2 to control the ROV for SLVHS robotics (Sea Exploration League), and is run on a Raspberry Pi.
The system consists fo the following:
* joystick nodes (not created by us)
* joystick calibration node
* joystick logic node
* pca9685 (pwm hardware) node
* config nodes to edit motor definitions

Joystick Nodes --sensor_msgs/Joy--> Joystick Logic Nodes --slvrov_interfaces/PCA9685Command--> PCA9685/PWM Node --PWM signal (non-ROS2)--> Motors/Servos

### Graphs Key
```mermaid
graph TB
    hardware["Hardware"]
    ros(["ROS Node"])
    otherprogram[["Other Programs"]]
    conn{{"Wired Interfaces"}}
    othertype("Other Software Concept")
    file[("File or Database")]

    otherprogram-->|"Misc Data"|file
    ros-->|"Control Data"|othertype
    hardware-->|"Video Data"|conn
    othertype-->|"Misc Data"|otherprogram

    linkStyle 0 stroke:green,stroke-width:1px;
    linkStyle 1 stroke:blue,stroke-width:1px;
    linkStyle 2 stroke:orange,stroke-width:1px;
    linkStyle 3 stroke:red,stroke-width:1px;
```

## Harware System Abstract
```mermaid
graph TD
    CamOut["Display"]
    ConIn["Joystick"]
    RPi5["Raspberry PI 5"]

    RPi5-->CamOut
    ConIn-->RPi5

    Eth{{"Ethernet"}}
    Eth<-->|ROV|RPi4
    Eth<-->|Surface|RPi5

    RPi4["Raspberry Pi 4"]
    CamIn["Cameras"]
    ConOut["Motors"]

    RPi4-->ConOut
    CamIn-->RPi4

    linkStyle 1 stroke:blue,stroke-width:1px;
    linkStyle 4 stroke:blue,stroke-width:1px;

    linkStyle 0 stroke:orange,stroke-width:1px;
    linkStyle 5 stroke:orange,stroke-width:1px;
```
### Surface
On the surface, a Raspberry Pi 5 is connected to a joystick(s) and a display. The Pi 5 is in charge of interpreting joystick input and passing it down to the ROV through ethernet. The surface Pi also recieves a camera feed and displays it.

### ROV
In the ROV, a Raspberry Pi 4 is connected to motors/servos as well as a camera(s). From the forwarded interpreted input, the Pi manipulates the motors. It also runs live video up to the surface.

## Software System Abstract
```mermaid
graph TD
    PinConfCl(["Pin Configuration Client"])
    JoyIn(["Joystick Input"])
    JoyCal(["Joystick Calibration"])
    JoyCalFil[("Calibrated Joystick File")]
    JoyLog(["Joystick Logic"])
    GstOut[["Display Video with GStreamer"]]

    JoyIn--"joy topics"-->JoyCal
    JoyCal-->JoyCalFil
    JoyCalFil-->JoyLog
    JoyIn--"joy topics"-->JoyLog

    UbD("Ubuntu Desktop")
    UbS("Ubuntu Server")

    PinConfSr(["Pin Configuration Server"])
    PinConfFil[("Pin Configurations JSON")]
    Pwm(["PCA9685 (PWM) Interface"])
    GstIn[["Stream Video with GStreamer"]]

    PinConfCl--"pin_config services"-->UbD
    UbS--"pin_config services"-->PinConfSr
    JoyLog--"pca9685_command"-->UbD
    UbS--"pca9685_command"-->Pwm
    GstIn--"gstreamer pipeline"-->UbS
    UbD--"gstreamer pipeline"-->GstOut

    PinConfSr-->PinConfFil
    PinConfFil-->Pwm

    UbD<-->|ethernet|UbS

    linkStyle 0 stroke:red,stroke-width:1px;
    linkStyle 1 stroke:red,stroke-width:1px;
    linkStyle 2 stroke:red,stroke-width:1px;

    linkStyle 3 stroke:blue,stroke-width:1px;
    linkStyle 6 stroke:blue,stroke-width:1px;
    linkStyle 7 stroke:blue,stroke-width:1px;

    linkStyle 4 stroke:green,stroke-width:1px;
    linkStyle 5 stroke:green,stroke-width:1px;
    linkStyle 10 stroke:green,stroke-width:1px;
    linkStyle 11 stroke:green,stroke-width:1px;

    linkStyle 8 stroke:orange,stroke-width:1px;
    linkStyle 9 stroke:orange,stroke-width:1px;
```
### Ubuntu Desktop (Surface/Raspberry Pi 5)
Ubuntu Desktop will be responsible for the joystick input, calibration, joystick logic, and pin configuration ROS2 nodes. Additionally, it will accept live video from the ROV using a GStreamer program.

#### Joysticks
Throughout operation, the joystick node will read and pubish input messages to a topic, e.g. "joy0". In some cases, there may be multiple joystick nodes if more than one joystick is being used. Before input can be interpreted and sent to the ROV, a file describing how joystick indices and axes map to each type of movement. This is best done once using the joystick calibration node. Once this is saved, the joystick logic node can interpret joystick input from the joystick topic and send it to the ROV.

#### Pin Configuration
The pin configuration client can be run to edit the physical pin map of software-defined pwm devices. Through a few services, a user could view existing configurations or add new ones. These will exist on a file on the ROV computer.

#### Gstreamer Streaming
For simple cases, a script located in slvrov-tools (slvrov_tools_vendor/slvrov-tools) called 'udp-cam' could be used to display a live udp stream from the ROV. For more complex cases, more verbose gstreamer commands can be used (e.g. using compositor to display multiple feeds at once).

### Ubuntu Server (ROV/Raspberry Pi 4)
...

## Testing

### 1. Log into Raspberry Pi with dependencies installed
   * ssh [usr]@[rpi-ip]
   * scp /path/to/slvrov_ros [usr]@[rpi-ip]

### 2. Follow build instructions for this repo

### 3. Baisc Testing
   * using ros2 topic pub to test subscriber nodes
   * using ros2 service req to test server nodes

### 4. Intermediate Testing
   Once the nodes have been tested by probing them with ros2 commands, run multiple nodes without peripherals to see if they interface correctly.
   Succes should be indicated, at the base level, by well-placed self.get_logger() calls.

### 5. Advanced Testing
   Attach needed peripherals to Raspberry Pi.
   Run multiple nodes with the necessary hardware.
   If any problems arise, use diagnostic tools like i2cdetect to see where the problem occurs, if not located in the code.

### 6. Editing/Updating Code
   If any changes in the code need to be made, make sure to rebuild the ros2 package that changed.
   Run 'colcon build --packages-select [package names]'
