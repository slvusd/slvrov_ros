# ROV Config Directory

This directory is the starter home for JSON/YAML configuration shared by
`slvrov_core_python`, `slvrov_launch`, and owner-authored ROS2 node code.

For the MVP, persistent configuration changes should eventually be made
from Setup Mode and saved here or to a target-machine path with the same
shape. Developer Mode should treat edits as temporary unless a later task
adds an explicit save flow through Setup Mode.

Suggested runtime grouping:

- `motors/`: thruster configuration and motor controller pin/channel mappings.
- `actions/`: action definitions and action-type mapping data.
- `controls/`: joystick/control profiles and related mappings.
- `rovs/`: complete ROV selections and launch parameter files.

The current files are examples only. Future tasks should define detailed
fields, validation rules, and load/save helpers before treating them as
final runtime config.
