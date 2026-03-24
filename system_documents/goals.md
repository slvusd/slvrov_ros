# SLVROV ROS 2 Stack - Requirements and Restraints

## Requirements
### Control
* ROV can be piloted with different types of control hardware (game controller, single joystick, multiple)
* Joystick calibration node should map ROV movement to the joystick topic and axis/button
* ROV movement/functions can be skipped safely in calibration
* Calibration states can be saved to disk for reuse
* Joystick logic node should recieve updates from joystick topics, apply correct math, and then send appropriate command
* Joystick logic node should load calibration state
* Skipped ROV movement/functions in calibration states should make operations in logic node unsafe or incorrect

Redundancies:
* Allow use of buttons for 4-way movement in case math fails -- should be able to be setup in calibration
* Text-based 4-way movement + functions for last redundancy

### Camera
* Mediamtx for WebRTC served with html by flask app
* 3+ cameras
* website has adjustable views
* adjusting camera settings through ui on website should have minimal latency
* mediamtx yml's ffmpeg commands need to be ultra low-latency
* the code for the frontend, backend, metdamtx should live in a simple, understandable, replacable structure

## Restraints
* All code running on ubuntu server on raspberry pi simultaniously
* Should not (thouogh not could not) use external event loops in ROS2 nodes
* Avoid using external libraries that don't exist with slvrov_ros or slvrov-tools <-- provides opencv & numpy