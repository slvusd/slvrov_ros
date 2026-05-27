# MVP Shared Config Directory

This directory is the starter home for JSON config files shared by the
MVP web UI, science tools, and ROS2 bridge code.

For the MVP, persistent configuration changes should eventually be made
from Setup Mode and saved here or to a target-machine path with the same
shape. Developer Mode should treat edits as temporary unless a later task
adds an explicit save flow through Setup Mode.

The files in `mvp/` are examples only. Task 2 will define the detailed
fields, validation rules, and load/save helpers.

