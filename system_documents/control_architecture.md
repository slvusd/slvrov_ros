# Control Architecture

This document describes the current joystick/control architecture and the safety
choices behind it.

## Node Boundaries

- `joystick_calibrator`
  - discovers bindings
  - writes `joy_topics` and `mappings` to disk
- `joystick_logic_node`
  - loads saved mappings
  - reads live joystick messages
  - reconstructs logical motion values
  - keeps missing actions neutral-safe
  - publishes `/thruster_command`
- `thruster_bridge_node`
  - forwards `/thruster_command` to `/pca9685_command`
- `pca9685_node`
  - resolves logical IDs to configured PCA9685 outputs
  - writes duty cycles to hardware

## Topic and File Roles

- `sensor_msgs/msg/Joy`
  - raw operator input from joystick drivers
- `joy_mappings.yaml`
  - saved calibration profile
- `/thruster_command`
  - normalized thruster output bundle from joystick logic
- `/pca9685_command`
  - same bundle forwarded toward hardware translation

## Safety Model

The current safety model is:

- stale joystick inputs time out to neutral
- unmapped actions stay neutral
- startup warns instead of failing for missing mappings
- the mixer and hardware command format remain unchanged

This is a conservative choice. Unknown or absent operator input should not
invent motion.

## Deferred Redundancy Layers

Planned but not active in this pass:

- button-based movement fallback
- text-based operator redundancy
- claw control pipeline
- extended YAML profile schema

Those features are tracked centrally in
`/src/slvrov_nodes_python/slvrov_nodes_python/unimplemented_features.py`.

## Rationale

- Neutral-safe defaults reduce the chance that an incomplete calibration profile
  turns into unintended thrust.
- Keeping the current command contract avoids forcing simultaneous changes in
  logic, bridge, PCA9685 routing, and documentation.
- Deferred redundancy layers are intentionally separated so future work can add
  them through the same logical control path instead of branching the control
  stack.
