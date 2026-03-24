# Joystick Pipeline Critique

This document reviews the recent joystick-related changes in:

- `/start_joy.sh`
- `/src/slvrov_nodes_python/launch/launch.py`
- `/src/slvrov_nodes_python/slvrov_nodes_python/multi_joy_logic.py`
- `/src/slvrov_nodes_python/slvrov_nodes_python/thruster_bridge_node.py`
- `/src/slvrov_nodes_python/slvrov_nodes_python/joystick_calibrator.py`

The calibrator changes are small and mostly harmless. The bigger concerns are
in launch integration, packaging, and the new bridge path.

## What Is Good About The Direction

- Splitting joystick input into `/joy_left` and `/joy_right` is a good move.
  It makes the operator interface explicit and matches the multi-topic support
  already built into the calibrator and logic node.
- Introducing a bridge node is a reasonable architectural idea.
  A thin adapter between control logic and hardware routing can be a clean
  boundary if the interfaces are stable.
- Moving toward a single launch entry point is also a good idea.
  The stack is easier to operate when joystick nodes, logic, and downstream
  routing come up together.

## Problem 1: `launch.py` Contains a Stray Runtime Statement

### Where

- `/src/slvrov_nodes_python/launch/launch.py:2`

### Why It Is A Problem

The file contains a top-level `Copy` token on its own line.

That line is syntactically valid Python, so a compile-only check does not catch
it. But importing the launch file executes that statement, which raises a
`NameError` immediately. In practice, this means the launch file is likely to
fail before ROS ever gets to `generate_launch_description()`.

This is especially dangerous because it looks harmless during a quick scan and
can slip past simple static checks.

### Recommended Solution

Remove the stray line and add a very small validation step for launch files.

The broader lesson here is that launch files should be treated as executable
code, not as static configuration. A quick import test is more valuable than a
compile-only test for this kind of file.

### Basic Implementation Idea

1. Delete the `Copy` line.
2. Add a lightweight test or CI command that imports the module and calls
   `generate_launch_description()`.
3. If the team has a ROS test suite, include a smoke test that runs
   `ros2 launch ... --show-args` or an equivalent non-hardware validation step.

## Problem 2: The Launch File Uses the Wrong Executable Name

### Where

- `/src/slvrov_nodes_python/launch/launch.py:44-49`
- `/src/slvrov_nodes_python/setup.py:25-33`

### Why It Is A Problem

The launch file tries to start:

- `executable="joystick_logic_node"`

But the installed console script declared in `setup.py` is:

- `joystick_logic=slvrov_nodes_python.multi_joy_logic:main`

Unless there is another executable being generated elsewhere, the launch system
will not find `joystick_logic_node`. That means the new launch file is pointing
at a node name, not the package executable that actually exists.

This is a classic ROS packaging mismatch:

- node runtime name: `joystick_logic_node`
- executable name: `joystick_logic`

Those are different concepts and both need to be correct.

### Recommended Solution

Use the installed executable name in launch files and keep the node runtime
name as a separate concern.

The clean convention is:

- `executable` should match the `console_scripts` entry
- `name` should match the ROS node name you want on the graph

### Basic Implementation Idea

1. Change the launch file to use `executable="joystick_logic"`.
2. Keep `name="joystick_logic_node"` if that graph name is desired.
3. Add a quick packaging check after install:
   `ros2 pkg executables slvrov_nodes_python`
4. Optionally standardize naming so executable and node names are closer
   together and easier to reason about.

## Problem 3: The Mapping File Is Wired Into Launch Incorrectly

### Where

- `/src/slvrov_nodes_python/launch/launch.py:18-24`
- `/src/slvrov_nodes_python/launch/launch.py:44-49`
- `/src/slvrov_nodes_python/slvrov_nodes_python/multi_joy_logic.py:326-345`
- `/joy_mappings.yaml`

### Why It Is A Problem

The launch file declares a `mappings_yaml` argument and then passes it as a
parameter file:

```python
parameters=[cfg, LaunchConfiguration("mappings_yaml")]
```

But `JoystickLogicNode` does not expect that file to be loaded as a ROS
parameter file. Instead, it expects the path to be assigned to the `mapping_file`
parameter, and then it manually opens that file inside `_load_mapping_file()`.

That means there is a contract mismatch:

- launch treats `mappings_yaml` like a ROS parameter YAML
- the node treats it like an application data file

Those are not the same format.

The calibrator output at `/joy_mappings.yaml` is a simple payload:

```yaml
joy_topics: []
mappings: []
```

That file is not structured like a ROS parameter file with node names and
`ros__parameters`. The node can load it directly, but only if the launch file
passes it into the `mapping_file` parameter explicitly.

As written, the logic node may start without actual mappings or topics and then
raise configuration errors.

### Recommended Solution

Pick one configuration model and follow it consistently.

The simplest option is:

- keep the calibrator output as plain application YAML
- pass its path into `mapping_file`
- reserve ROS parameter YAML files for true ROS parameters only

This keeps the calibrator output simple and keeps the logic node in control of
how that file is parsed.

### Basic Implementation Idea

1. Change the launch file to pass:

   ```python
   parameters=[
       cfg,
       {"mapping_file": LaunchConfiguration("mappings_yaml")},
   ]
   ```

2. Make sure `cfg` is a real ROS parameter YAML and `joy_mappings.yaml` is a
   plain mapping file.
3. Add startup logging that prints:
   - the resolved `mapping_file` path
   - the number of loaded topics
   - the number of loaded mappings
4. Add a failure message that clearly distinguishes:
   - missing ROS parameter file
   - missing joystick mapping file
   - malformed mapping payload

## Problem 4: Packaging And Installed Resource Layout Are Incomplete

### Where

- `/src/slvrov_nodes_python/setup.py:9-13`
- `/src/slvrov_nodes_python/launch/launch.py:12-24`

### Why It Is A Problem

The launch file relies on package-share resources:

- a launch file discoverable through the package
- a `config` directory returned by `get_package_share_directory()`
- `slvrov_config.yaml`
- `joy_mappings.yaml`

But `setup.py` only installs:

- `package.xml`
- the package resource marker

It does not install:

- `launch/*.py`
- any `config/*.yaml`

So even if the code inside the launch file were perfect, the installed package
layout still would not support the assumptions the launch file makes.

There is also no `config` directory in this package right now, which means the
default `_cfg()` path does not line up with the current repository structure.

This makes the feature fragile in two ways:

- `ros2 launch slvrov_nodes_python launch.py` may fail because the launch file
  is not installed into the package share
- even if it launches, the default config paths may not exist

### Recommended Solution

Make the package share layout explicit and treat launch/config assets as part of
the delivered interface, not as optional side files.

The package should install:

- launch files
- runtime YAML configuration
- example or default joystick mapping payloads if needed

### Basic Implementation Idea

1. Add package data installation entries for:
   - `share/slvrov_nodes_python/launch`
   - `share/slvrov_nodes_python/config`
2. Create an actual `config/` directory in the package source tree.
3. Move or create:
   - `slvrov_config.yaml`
   - a default or example `joy_mappings.yaml`
4. Update the launch file defaults to point only at files that are guaranteed
   to be installed.
5. Add a post-install smoke check that verifies:
   - `ros2 pkg prefix slvrov_nodes_python`
   - expected files exist under the share directory

## Problem 5: `thruster_bridge_node` Does Not Retain Its Subscription Object

### Where

- `/src/slvrov_nodes_python/slvrov_nodes_python/thruster_bridge_node.py:47-52`

### Why It Is A Problem

The node creates a subscription but does not store the returned subscription
object on `self`.

In `rclpy`, publishers and subscriptions should generally be stored as object
attributes. If the Python object is garbage collected, the callback path can
stop working unexpectedly. That risk is subtle because the node may appear to
work for a while and then fail in a way that looks unrelated.

For a bridge node, that would be especially confusing:

- `joystick_logic_node` appears healthy
- `pca9685_node` appears healthy
- but commands silently stop crossing the bridge

### Recommended Solution

Store the subscription on the node instance, just like the publisher.

This is a small change, but it makes the node lifecycle explicit and reliable.

### Basic Implementation Idea

1. Replace the bare `self.create_subscription(...)` call with:

   ```python
   self.sub = self.create_subscription(...)
   ```

2. Use clear names such as:
   - `self.command_sub`
   - `self.command_pub`
3. Add a small test that instantiates the node and verifies both publisher and
   subscription attributes exist.

## Problem 6: The New Command Path Does An Unnecessary Encode/Decode Round Trip

### Where

- `/src/slvrov_nodes_python/slvrov_nodes_python/multi_joy_logic.py:394-414`
- `/src/slvrov_nodes_python/slvrov_nodes_python/multi_joy_logic.py:546-566`
- `/src/slvrov_nodes_python/slvrov_nodes_python/pca9685_node.py:127-145`

### Why It Is A Problem

`JoystickLogicNode` already computes normalized thruster outputs in the range
`[-1.0, 1.0]`.

Then it:

1. converts thrusters to PWM-like integers such as `1100..1900`
2. converts those integers back into normalized values in `_build_command()`
3. publishes those normalized values in `PCA9685Command`
4. lets `pca9685_node` convert them into hardware-specific duty cycles again

That round trip is unnecessary and makes the data flow harder to reason about.

It also hardcodes assumptions:

- midpoint `1500`
- half-span `400`

Those assumptions may be fine for some ESCs or servos, but they do not belong
in the generic command publication path if the downstream hardware node already
has per-device min/default/max configuration.

The result is not necessarily incorrect today, but it is over-coupled and easy
to break later.

### Recommended Solution

Choose one of these two designs and stick to it:

1. Publish normalized actuator commands end to end.
   This is the cleaner design for the current codebase.
2. Publish real pulse widths end to end.
   This only makes sense if every downstream consumer truly expects pulse
   widths, which the current `pca9685_node` does not.

Given the current `PCA9685Command` contract, the better solution is to publish
normalized values directly and remove the fake-PWM conversion from the logic
node.

### Basic Implementation Idea

1. Build the outgoing message directly from:
   - `thrusters`
   - normalized claw values
2. Move the ID labeling into a helper such as:

   ```python
   def _build_command(self, thrusters: np.ndarray, state: ControlState) -> PCA9685Command:
   ```

3. Remove the dependence on `map_to_pwm()` for publication.
4. Keep PWM conversion only for:
   - debug logging
   - operator visualization
   - any future consumer that explicitly needs pulse widths
5. If PWM integers are still useful for observability, publish them on a
   separate debug topic or log them only.

## Problem 7: `start_joy.sh` Is Useful But Too Machine-Specific

### Where

- `/start_joy.sh:2-6`

### Why It Is A Problem

The script is convenient, but it hardcodes several assumptions:

- ROS is installed at `/opt/ros/jazzy`
- the workspace overlay is at `~/slvrov_ros/install/setup.bash`
- `gnome-terminal` exists
- joystick device IDs `0` and `1` are always the desired controllers

This makes the script handy for one workstation and brittle for everyone else.
Even on the same machine, device numbering can shift after reboot or reconnect.

Because joystick topic identity now matters to the calibrator and logic node,
device selection should ideally be more stable than "whatever Linux calls
device 0 today."

### Recommended Solution

Keep the convenience script, but turn it into a small, configurable launcher
instead of a hardcoded personal helper.

The simplest robust version would:

- discover the workspace relative to the script location
- allow overriding left and right device IDs with environment variables
- fail with a clear message when `gnome-terminal` is unavailable

An even better version would identify controllers by device path or name rather
than raw integer order.

### Basic Implementation Idea

1. Resolve the workspace path relative to the script:

   ```bash
   SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
   source "$SCRIPT_DIR/install/setup.bash"
   ```

2. Allow overrides:

   ```bash
   LEFT_DEVICE_ID="${LEFT_DEVICE_ID:-0}"
   RIGHT_DEVICE_ID="${RIGHT_DEVICE_ID:-1}"
   ```

3. Check dependencies before launching:
   - `command -v gnome-terminal`
   - `command -v ros2`
4. Print the resolved left/right device IDs and topic names before opening
   terminals.
5. Longer term, replace the shell helper with a ROS launch file or a small
   Python launcher that can match controllers by identity.

## Suggested Refactor Order

If these issues are addressed, this order will produce the fastest path to a
working system:

1. Fix `launch.py` import/runtime issues.
2. Fix launch executable naming and mapping file wiring.
3. Fix package installation of launch/config resources.
4. Make `thruster_bridge_node` lifecycle-safe.
5. Clean up the command contract in `multi_joy_logic.py`.
6. Harden `start_joy.sh` for repeatable operator use.

## Final Assessment

The overall architecture is moving in a good direction, but the implementation
is not production-safe yet because the new launch path, resource layout, and
runtime contracts are not fully aligned.

The most important theme is consistency:

- launch files should reference installed executables
- installed packages should include the assets launch files depend on
- mapping files should be passed according to the format the node actually
  expects
- message payloads should represent one clear abstraction without needless
  conversion layers

Once those pieces line up, the new left/right joystick pipeline should be much
easier to operate and maintain.
