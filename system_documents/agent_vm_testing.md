# Agent VM Testing Guide

This guide is for agents testing `slvrov_ros` on the Ubuntu VM. The VM is available over SSH from the development machine.

## 1. Connect to the VM

From the local workspace, note the branch you are working on:

```bash
git branch --show-current
```

Then log into the Ubuntu VM:

```bash
ssh codex-tester
```

## 2. Pull the Latest Code to Test

On the VM, enter the ROS workspace. The expected checkout location is `~/slvrov_ros`.

```bash
cd ~/slvrov_ros
```

If the checkout does not exist yet, clone it first:

```bash
git clone --recurse-submodules https://github.com/slvusd/slvrov_ros.git ~/slvrov_ros
cd ~/slvrov_ros
```

Sync the VM checkout to the branch being tested:

```bash
git status --short
git fetch --all --prune
git checkout <branch-name>
git pull --ff-only origin <branch-name>
git submodule update --init --recursive
```

Replace `<branch-name>` with the branch from the local workspace. If the local changes are not committed and pushed, the VM cannot pull them from git yet.

## 3. Build the Workspace

Set up ROS 2, build, and source the local install:

```bash
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

If build artifacts appear stale after switching branches, rebuild from a clean workspace before trusting the result:

```bash
rm -rf build install log
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

Only remove `build`, `install`, and `log` inside the VM checkout.

## 4. Check ROS 2 Health With ros2doctor

Before running package tests, launch files, or nodes, run `ros2 doctor` from the sourced VM workspace:

```bash
ros2 doctor
```

If `ros2 doctor` reports warnings, read them before continuing. Some warnings may be harmless for an offline VM, but environment, middleware, daemon, or graph warnings can explain failed tests and missing topics.

For a complete diagnostic snapshot, run:

```bash
ros2 doctor --report
```

Run `ros2 doctor` again while code is running if nodes cannot discover each other, expected topics or services are missing, or behavior changes between SSH sessions. In each new SSH terminal, repeat:

```bash
cd ~/slvrov_ros
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 doctor
```

Include important `ros2 doctor` warnings in the final test report.

## 5. Run Automated Tests

Run the package tests and print failures:

```bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

If the test run fails, capture the failing package, test name, and the important error lines before making changes.

## 6. Run ROS 2 Smoke Tests

The existing examples in [testing.md](testing.md) are useful for checking whether the workspace is discoverable and whether basic ROS 2 interfaces work.

List packages:

```bash
ros2 pkg list | grep slvrov
```

List interfaces for a package:

```bash
ros2 interface package slvrov_interfaces
```

Launch a node or launch file relevant to the change:

```bash
ros2 run <package_name> <node_name>
ros2 launch slvrov_launch launch.py
```

Use ROS 2 CLI probes to inspect behavior from another SSH session if a node is running.

Publish a synthetic message:

```bash
ros2 topic pub /pca9685_command slvrov_interfaces/msg/PCA9685Command "{id: ['my_servo'], pwm: [0.0]}"
```

Subscribe to a topic:

```bash
ros2 topic echo /pca9685_command
```

Request a service:

```bash
ros2 service call /add_pca9685_pin_configs slvrov_interfaces/srv/AddPCA9685PinConfigs "{id: 'servo15', pins: [15], minimum: 1000, default: 1500, maximum: 2000}"
```

Watch ROS logs:

```bash
ros2 topic echo /rosout
```

## 7. Report Results

When testing is complete, report:

- VM branch and commit tested:

  ```bash
  git branch --show-current
  git rev-parse --short HEAD
  ```

- Build result: pass or fail.
- `ros2 doctor` result: pass, warnings, or fail.
- Automated test result: pass or fail, with failing package/test names if any.
- Manual ROS 2 smoke tests run and their observed result.

Exit the VM when done:

```bash
exit
```
