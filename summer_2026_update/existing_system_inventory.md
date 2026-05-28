# Existing System Inventory

Task 1 inventory for the ROV Web UI MVP. This is a repo snapshot before new
MVP implementation begins; no code changes are included in this task.

## Package Snapshot

### `src/slvrov_core_python`

Core Python package for vehicle control and hardware-facing behavior.

Important files:

- `src/slvrov_core_python/slvrov_core_python/joystick_mapper.py`
- `src/slvrov_core_python/slvrov_core_python/joystick_logic.py`
- `src/slvrov_core_python/slvrov_core_python/pca9685_node.py`
- `src/slvrov_core_python/slvrov_core_python/pin_mappings_server.py`
- `src/slvrov_core_python/slvrov_core_python/pin_mappings_client.py`
- `src/slvrov_core_python/slvrov_core_python/json_crud.py`
- `src/slvrov_core_python/slvrov_core_python/control_objects.py`
- `src/slvrov_core_python/config/joy_mappings.yaml`
- `src/slvrov_core_python/config/pca9685_pin_configs.json`

What exists:

- `joystick_mapper.py` exposes ROS2 services for mapper activation, action
  selection, mapping runs, saving mappings, status, add/edit/delete mapping,
  and viewing mappings.
- `joystick_logic.py` merges `sensor_msgs/Joy` topics into a control state,
  applies deadzones/gains, calculates thruster/claw outputs, and publishes
  `PCA9685Command` on `thruster_command`.
- `pca9685_node.py` consumes logical PWM commands and writes to PCA9685 pins
  through `slvrov_tools.pca9685`.
- `pin_mappings_server.py` and `pin_mappings_client.py` provide service/client
  flows for PCA9685 pin config storage.
- `json_crud.py` and `control_objects.py` are reusable helper/value-object
  modules.

Can reuse:

- Joystick mapper services and value objects for future Setup Mode control
  mapping work.
- `json_crud.py` for simple top-level JSON object files.
- Existing PCA9685 config shape as reference material for future config
  decisions.

Should not touch casually:

- `joystick_logic.py`, `pca9685_node.py`, and pin-mapping persistence are
  control/hardware paths and should stay owner-reviewed.
- `submersed_globals.py` uses current-working-directory-based config paths, so
  future MVP persistence should not copy that behavior without a decision.
- `joystick_logic.py` still mentions a `thruster_bridge_node`, while
  `summer_2026_update/slvrov_ros_nodes.md` says that bridge should likely be
  removed.

Likely future owner:

- Core drive, joystick, thruster, PCA9685, and safety-critical control behavior.

### `src/slvrov_interfaces`

Custom ROS2 interface package.

Important files:

- `src/slvrov_interfaces/msg/PCA9685Command.msg`
- `src/slvrov_interfaces/srv/AddPCA9685PinConfigs.srv`
- `src/slvrov_interfaces/srv/GetPCA9685PinConfigs.srv`
- `src/slvrov_interfaces/srv/String.srv`
- `src/slvrov_interfaces/srv/JoystickMapper/*.srv`
- `src/slvrov_interfaces/CMakeLists.txt`

What exists:

- `PCA9685Command.msg` has `string[] id` and `float32[] pwm`.
- PCA9685 services add/fetch pin configs.
- Joystick mapper services cover set mapper state, set action, send mapping,
  delete mapping, and fetch status.

Can reuse:

- Existing service contracts for web bridge adapters when exposing current
  joystick/PCA9685 behavior to Flask.

Should not touch casually:

- `.msg` and `.srv` changes affect generated ROS2 code and all callers. Future
  interface edits should wait for an approved boundary/API task.

Likely future owner:

- Shared contracts for web bridge, safety, preflight, developer monitor,
  storage monitor, and optional media/capture ROS2 nodes.

### `src/slvrov_launch`

Launch package.

Important files:

- `src/slvrov_launch/launch/two_joystick_launch.py`
- `src/slvrov_launch/launch/test_launch.py`
- `src/slvrov_launch/config/two_joystick_launch.yaml`
- `src/slvrov_launch/config/test_launch.yaml`

What exists:

- `two_joystick_launch.py` starts two `joy` nodes and tries to start
  `joystick_logic`.
- `two_joystick_launch.yaml` includes joystick device IDs, joystick topics,
  loop rate, timeout, mapping file, axis gains, mixing matrix, and thruster
  inversions.

Can reuse:

- Launch/config patterns for future MVP launch orchestration.

Should not touch casually:

- `two_joystick_launch.py` references package `slvrov_nodes_python`, but this
  repo now contains `slvrov_core_python`. Fixing that should be a focused
  future task, not part of inventory.

Likely future owner:

- Startup orchestration for core nodes, web UI, bridge/safety/preflight/monitor
  nodes, and science/media support.

### `src/slvrov_web_ui`

MVP Flask and browser UI skeleton.

Important files:

- `src/slvrov_web_ui/slvrov_web_ui/docs/setup_note.md`
- `src/slvrov_web_ui/slvrov_web_ui/routes/README.md`
- `src/slvrov_web_ui/slvrov_web_ui/ros_adapters/README.md`
- `src/slvrov_web_ui/slvrov_web_ui/nodes/README.md`
- `src/slvrov_web_ui/slvrov_web_ui/static/README.md`
- `src/slvrov_web_ui/slvrov_web_ui/templates/base.html`
- `src/slvrov_web_ui/slvrov_web_ui/static/css/base.css`
- `src/slvrov_web_ui/slvrov_web_ui/static/js/app.js`

What exists:

- Package skeleton exists with `python3-flask` and `rclpy` dependencies.
- Placeholder docs already reserve locations for Flask routes, static assets,
  templates, fake ROS adapters, and owner-authored node placeholders.
- No real Flask app, API route, adapter implementation, or mode UI exists yet.

Can reuse:

- This should be the main home for Flask routes, API helpers, UI files, fake
  adapters, route tests, and web-facing docs.

Should not touch casually:

- Do not put safety-critical ROS2 node internals directly in Flask routes.
- Do not implement owner-authored ROS2 node logic here unless explicitly asked.

Likely future owner:

- Main page, Pilot Mode, Scientist Mode web surface, Setup Mode, Developer
  Mode, web API helpers, fake adapters, and route-level tests.

### `src/slvrov_science_python`

Science/camera/data-collection skeleton.

Important files:

- `src/slvrov_science_python/slvrov_science_python/docs/setup_note.md`
- `src/slvrov_science_python/slvrov_science_python/capture/README.md`
- `src/slvrov_science_python/slvrov_science_python/media/README.md`
- `src/slvrov_science_python/slvrov_science_python/data_logging/README.md`

What exists:

- Package skeleton exists with placeholder directories for capture, media, and
  data logging.
- No camera, media, or sensor logging behavior exists yet.

Can reuse:

- Likely home for non-ROS camera/media helpers, filename/metadata helpers, and
  future data logging helpers.

Should not touch casually:

- Camera capture ownership is not decided yet. It may belong in ROS2, Flask
  adapters, or this package depending on the boundary decision.
- Do not store generated media inside this source package.

Likely future owner:

- Scientist Mode support code when it is not better owned by a ROS2 node.

### `src/slvrov_tools_vendor`

Vendored `slvrov_tools` package.

Important files:

- `src/slvrov_tools_vendor/slvrov_tools/src/slvrov_tools/pca9685.py`
- `src/slvrov_tools_vendor/slvrov_tools/src/slvrov_tools/gst_tools.py`
- `src/slvrov_tools_vendor/slvrov_tools/src/slvrov_tools/cv2_tools.py`
- `src/slvrov_tools_vendor/slvrov_tools/src/slvrov_tools/network_tools.py`
- `src/slvrov_tools_vendor/env_hook/pythonpath.sh`

What exists:

- Core hardware code imports PCA9685 helpers from this package.
- Camera/GStreamer/OpenCV/network helpers also exist.
- `summer_2026_update/slvrov_ros_structure.md` says this vendored dependency
  may eventually be removed in favor of normal installation.

Can reuse:

- Existing PCA9685 dependency path for current hardware code.
- Camera helper files as planning references after the camera strategy is
  approved.

Should not touch casually:

- Treat this as external-ish vendored code. Avoid editing during MVP work unless
  the owner decides to keep and maintain it.

## Existing Web UI Concepts

Important files:

- `ui_static/joystick_mapper_ui_v1/*`
- `ui_static/joystick_mapper_ui_v2/*`
- `ui_static/joystick_mapper_ui_v3/*`
- `ui_static/joystick_mapper_ui_v3/requests.md`
- `ui_static/joystick_mapper_ui_v3/layout.md`

What exists:

- Three static joystick mapper UI concepts exist outside the packaged
  `slvrov_web_ui` skeleton.
- V3 documents possible HTTP routes:
  `/joystick_mapper/set_mapper_state`, `/joystick_mapper/set_action`,
  `/joystick_mapper/set_mapping_state`, and `/joystick_mapper/status`.
- The concepts use `fetch()` and local UI state; no Flask backend was found.

Can reuse:

- Visual direction, flow ideas, and route notes for future Setup Mode mapping
  screens.

Should not touch casually:

- These are concepts, not the final packaged MVP frontend. Route names differ
  between versions and should be normalized in a later route-map task.

## Config, Data, And Docs

Important files:

- `config/README.md`
- `config/schemas/README.md`
- `config/mvp/*.example.json`
- `docs/data_directory.md`
- `summer_2026_update/summer_2026_updates_MVP.md`
- `summer_2026_update/mvp_agent_tasks.md`
- `summer_2026_update/slvrov_ros_structure.md`
- `summer_2026_update/slvrov_ros_nodes.md`
- `system_documents/ros2_python_style_guide.md`
- `system_documents/testing.md`

What exists:

- `config/mvp/` contains starter JSON examples for cameras, control profiles,
  pin mappings, recording/storage, ROS requirements, safety thresholds,
  thrusters, and UI preferences.
- The config examples are placeholders, not approved schemas.
- `docs/data_directory.md` says collected photos, videos, CSV files, logs, and
  metadata should live outside the source repo in a sibling `data/` directory.
- `system_documents/ros2_python_style_guide.md` is the standing style guide for
  ROS2 Python work.
- Some older docs still reference `slvrov_nodes_python`, `thruster_bridge`, and
  other names that do not fully match the current repo.

Can reuse:

- `summer_2026_updates_MVP.md` as the source of confirmed MVP requirements.
- `config/mvp/` and `config/schemas/` as the future shared config area.
- `docs/data_directory.md` as the baseline storage policy.

Should not touch casually:

- Do not treat placeholder JSON examples as final schemas.
- Do not silently rewrite legacy docs during unrelated MVP tasks.

## Tests Found

Important files:

- `src/slvrov_core_python/test/test_flake8.py`
- `src/slvrov_core_python/test/test_pep257.py`
- `src/slvrov_launch/test/test_flake8.py`
- `src/slvrov_web_ui/test/test_import_skeleton.py`
- `src/slvrov_science_python/test/test_import_skeleton.py`

What exists:

- Packages mostly have generated ament lint/copyright/pep257 tests.
- `slvrov_web_ui` and `slvrov_science_python` have skeleton import tests.
- No focused behavior tests were found for Flask routes, config validation,
  camera/media helpers, JSON helpers, joystick logic, or PCA9685 services.

Can reuse:

- Existing import tests as a pattern for early package checks.

Should not touch casually:

- Broad lint cleanup of older code should not be bundled into unrelated MVP
  feature tasks.

## High-Level Future Ownership Map

- Flask app, routes, API response helpers, frontend, fake adapters:
  `src/slvrov_web_ui`.
- Web bridge/safety/preflight/developer monitor placeholders:
  `src/slvrov_web_ui/slvrov_web_ui/nodes`.
- Science/media helpers when not ROS2-owned: `src/slvrov_science_python`.
- Shared config examples/schemas: `config/`.
- Runtime media/data policy: `docs/data_directory.md`.
- Core joystick/thruster/PCA9685 behavior: `src/slvrov_core_python`.
- ROS2 contracts: `src/slvrov_interfaces`.
- Startup orchestration: `src/slvrov_launch`.

## Owner Decision Needed Next

Before implementation tasks change code, confirm:

- Can `src/slvrov_web_ui` be the primary Flask/UI implementation package?
- Can `src/slvrov_science_python` own non-ROS camera/media helpers?
- Should `src/slvrov_core_python` remain read-mostly unless a task targets
  existing joystick/PCA9685 behavior?
- Should a future task update `src/slvrov_launch` to replace stale
  `slvrov_nodes_python` references?
- Should vendored `slvrov_tools` stay untouched during MVP work?

## Checks Performed

- Read Task 1 in `summer_2026_update/mvp_agent_tasks.md`.
- Listed repo files with `rg --files`.
- Inspected package manifests, setup files, launch files, interfaces, configs,
  tests, web UI skeletons, science skeletons, static UI concepts, and planning
  docs.
- No runtime tests were run because Task 1 does not require code changes or
  build checks.
