# SLVROV ROS 2 Stack — Setup Guide

## Overview

This guide walks you through setting up the SLVROV ROS 2 stack on a Raspberry Pi. By the end, you'll be able to run nodes that control servos and thrusters via a PCA9685 PWM driver.

---

## Prerequisites

- Raspberry Pi with Ubuntu and ROS 2 Jazzy installed
- Internet connection
- Basic familiarity with the terminal

---

## Step 1: Update and Install System Dependencies

Start by making sure your package lists are up to date, then install required system packages:

```bash
sudo apt-get update
sudo apt install build-essential python3-dev python3-pip
sudo apt install python3-opencv
```

---

## Step 2: Create a Python Virtual Environment

ROS 2 works best with a dedicated virtual environment to avoid package conflicts.

```bash
python3 -m venv venv
source venv/bin/activate
```

> ⚠️ You will need to run `source venv/bin/activate` every time you open a new terminal.

---

## Step 3: Install Python Build Dependencies

These packages are required for `colcon build` to work correctly inside a virtual environment:

```bash
pip install empy==3.3.4 catkin-pkg lark pyparsing
```

> ⚠️ The version pin on `empy` matters — newer versions break the ROS 2 build system.

---

## Step 4: Source ROS 2

Before building anything, ROS 2 needs to be sourced so your terminal knows where everything is:

```bash
source /opt/ros/jazzy/setup.bash
```

To make this permanent so you don't have to do it every session:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Step 5: Clone and Build the Tools Library

This is a separate C/Python library that the ROS nodes depend on.

```bash
git clone https://github.com/LegionaryOfLogic/slvrov-tools
cd slvrov-tools
```

Compile the shared C library:

```bash
make
```

Install it as a Python package:

```bash
pip3 install .
```

---

## Step 6: Clone the ROS 2 Stack

Go back to your home directory (or wherever you want the project to live), then clone the main ROS 2 repository:

```bash
cd ~
git clone --recurse-submodules https://github.com/LegionaryOfLogic/slvrov_ros.git
cd slvrov_ros
```

> The `--recurse-submodules` flag is important — it also downloads `slvrov_tools` as a submodule inside the ROS workspace.

---

## Step 7: Build the ROS 2 Workspace

From inside the workspace root (the folder containing `src/`):

```bash
colcon build
```

Then source the workspace so ROS 2 knows about your packages:

```bash
source install/setup.bash
```

> ⚠️ You need to run `source install/setup.bash` every time you open a new terminal, after sourcing ROS 2 in Step 4.

---

## Step 8: Verify the Build

Check that the packages are recognized:

```bash
ros2 pkg list | grep slvrov
```

You should see:
```
slvrov_interfaces
slvrov_nodes_python
slvrov_tools_vendor
```

---

## Every New Terminal — Checklist

Each time you open a fresh terminal, run these lines before doing anything else (from the workspace root):

```bash
source venv/bin/activate
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

---

## Running the Nodes
To launch an ROS2 node from the terminal, use
```bash
ros2 run [package name] [node name]
```

Example:
```bash
ros2 run slvrov_nodes_python pca9685_node
```