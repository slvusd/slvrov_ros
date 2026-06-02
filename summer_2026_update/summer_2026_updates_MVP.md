# ROV Web UI MVP Specification

## 0. Project Summary

The project is a ROS2-based driveable ROV with a simple but full browser-based UI. The UI should be usable by amateur scientists and roboticists, while the codebase should remain readable, structured, and maintainable by amateur programmers.

The project repository/package workspace is named:

- `slvrov_ros`

The target codebase structure is defined in `summer_2026_update/slvrov_ros_structure.md`. The MVP should move toward this structure:

- `slvrov_core_python` — ROS2 control nodes, Flask server code, MediaMTX support files, and production UI for Setup, Pilot, and Developer modes
- `slvrov_interfaces` — custom messages, services, and actions
- `slvrov_launch` — launch files for each approved ROV configuration
- `rov_config` — JSON/YAML configuration grouped by motors, actions, controls, and ROV definitions

Older MVP scaffold packages such as `slvrov_web_ui` and `slvrov_science_python` have been consolidated into `slvrov_core_python`. Future web/science work should continue inside the core package unless the owner explicitly creates a new package.

## 1. Target Users

### Amateur operators/scientists

These users need a clear, low-friction UI for driving, camera viewing, photo/video capture, and science data collection.

### Amateur programmers/maintainers

These users need a codebase that is simple, readable, well-commented, and split into understandable packages, nodes, services, topics, and frontend files.

## 2. Base Technical Decisions

### Confirmed

- ROS2 distribution: **Jazzy Jalisco**
- Main target operating system: **Ubuntu**
- Future testing possibility: Docker image on macOS
- Network model: nodes/server communicate over Ethernet
- Authentication: **not required for MVP**
- Access model: local network / LAN only
- Backend: Flask
- Config file format: JSON
- Cameras: USB cameras
- Camera count: support up to 6 cameras
- Camera streaming stack: MediaMTX with WebRTC
- Scientist media formats:
  - Video: MP4
  - Photos: JPEG
- Physical joystick input: Linux `js#` joystick devices through ROS2 joystick nodes
- Emergency stop: visible globally in all modes
- UI theme: dark theme by default, based on the existing project theme
- Primary display target: large displays such as monitors or laptops, especially 4:3 and 16:10 layouts
- Touch-specific UI: not required for MVP
- Main page should not remember the last selected mode
- Main page does not need detailed live connection status for MVP unless it is simple to implement

### Confirmed MVP Implementation Choices

- Frontend stack: plain HTML, CSS, and JavaScript.
- Frontend organization: split code by mode and by reusable UI utilities/components where useful.
- Camera layout customization: fixed/preset camera layouts only for MVP.
- Web joystick/controller controls: not part of MVP; add in a future version.
- Vehicle state model: use a simple software `control enabled / disabled` state rather than a full arming/disarming model for MVP.
- Backend live-update strategy: REST polling first; upgrade to WebSockets or Server-Sent Events only if polling becomes insufficient.
- File history: include a simple recent photos gallery.
- Visual style: hybrid by mode. Pilot should feel like a simple vehicle control panel; Scientist should feel more data/capture focused; Developer can be more technical. Create visual mockups before final implementation choices are locked.
- Agent workflow: strict one-sub-feature-at-a-time prompts/checkpoints.
- ROS2 node implementation: the project owner wants to write ROS2 nodes, or at least the important parts of them, personally. Coding agents should therefore prefer scaffolding, interfaces, adapters, fake implementations, tests, and clear TODO notes over fully implementing new ROS2 node internals unless the project owner explicitly asks for that node logic.
- Owner control: use small decision packets before implementation when architecture, route design, UI layout, safety behavior, or ROS2 boundaries are still open. The agent should show options, tradeoffs, and recommended defaults, then stop for owner review before building the selected option.

## 3. MVP Navigation

The main page acts as a mode selector. The minimum version includes four top-level modes:

1. Pilot Mode
2. Scientist Mode
3. Setup Mode
4. Developer Mode

Each mode should be visually distinct and focused on its user goal.

## 4. Global UI Requirements

These requirements apply across all modes.

### Emergency Stop

- Emergency stop must be visible in all modes.
- Emergency stop should be treated as a global safety control.
- If triggered, it should command the system to stop/disable motion as safely and quickly as possible.

### Critical Alerts

Critical alerts should appear globally across all modes.

Examples:

- Camera failure
- Backend disconnection
- ROS2 communication failure
- Motor/thruster safety fault
- Low disk space while recording
- Loss of expected ROS2 messages
- Emergency stop active

### Browser Disconnect / ROS2 Failure Behavior

- If the browser disconnects while motors are active, motors should turn off.
- If ROS2 stops publishing expected safety-critical messages, motors should turn off.
- The system should fail safe by stopping thrusters in relevant fault conditions.

## 5. Main Page

### Purpose

Give the user a clear entry point into the major workflows of the ROV web UI.

### MVP Features

- Four large navigation cards or buttons:
  - Pilot
  - Scientist
  - Setup
  - Developer
- Short description under each mode
- Keep the page simple; detailed live status is not required for MVP unless it is trivial to implement.
- Do not require login/authentication.
- Do not remember the last selected mode.

### Design Goal

The main page should be clean, readable, and approachable.

## 6. Pilot Mode

### Purpose

Provide the simplest and clearest driving interface possible.

### MVP Features

- Dynamic camera view using MediaMTX/WebRTC
  - Single-camera view
  - Multi-camera view
  - Fixed/preset layout choices
  - Up to 6 USB cameras
- Minimal extra information
  - No regular telemetry required for MVP
  - No control input visualization required for MVP
- Global emergency stop visible
- Physical joystick/controller input only for MVP
- Web UI controls that mimic a joystick/controller are a future-version feature
  - Future web controls should send data that can be published by a dummy joystick node or similar abstraction

### Layout Guidance

Pilot Mode should prioritize camera visibility above all else.

Potential camera layouts:

- One large selected camera
- Grid view for multiple cameras
- Fixed/preset layouts only for MVP
- Drag/resize layout customization is deferred

### Design Goal

Pilot Mode should be the simplest of all modes. It should prioritize low distraction, clear visual feedback, and safe operation.

## 7. Scientist Mode

### Purpose

Provide a camera-centered science interface for photo/video capture and future data collection.

### MVP Scope

For the current MVP, Scientist Mode only needs to handle cameras and media capture. Sensor data display/logging can be designed into the architecture but does not need to be implemented until sensors are ready.

### MVP Features

- Dedicated camera view with the same camera features as Pilot Mode:
  - Single-camera view
  - Multi-camera view
  - Fixed/preset layout choices
  - Up to 6 USB cameras
- Take photos from each camera
- Record video from each camera
- Save media filenames using timestamps by default
- Allow the scientist to override filenames when desired
- Use JPEG for photos
- Use MP4 for videos
- Choose the lowest-CPU recording strategy that works reliably with MediaMTX/WebRTC
  - If individual per-camera recording is more efficient and robust, prefer that.
  - If combined-layout recording is simpler or more useful later, document it as a future option.

### Future Sensor/Data Features

These are planned but not necessarily MVP implementation items until sensor details are available:

- Live sensor readings
- CSV data logging
- Save data at specified intervals
- Manual save button for current values
- Timestamped data rows
- Optional notes/annotations
- Media/data timestamp synchronization

### Design Goal

Scientist Mode should be data/capture oriented, but for MVP it should focus on reliable camera capture and file organization.

## 8. Setup Mode

### Purpose

Configure and test the ROV before normal operation.

### MVP Features

- Physical joystick setup
  - Joysticks exposed on Linux as `js#`
  - Existing ROS2 joystick nodes remain supported
- Web joystick/gamepad-style control option is deferred to a future version
  - When added, it should publish through a dummy joystick node or equivalent so downstream code can treat it like joystick input
- Control mapping
  - Map joystick axes
  - Map joystick buttons
  - Show live interpreted input values
  - Save multiple control profiles
- Thruster tuning
  - Store thruster configuration in JSON
  - Store pin mappings in JSON
  - Generate or update relevant ROS2 launch/config files where appropriate
- System Test / Preflight section
  - Automated tests started by the user pressing explicit test buttons
  - Do not use a single "Run All Tests" button for MVP
  - Save preflight/test results to a log file
  - Failed preflight tests should warn the user, but should not block Pilot Mode for MVP

### MVP Thruster Tuning Parameters

The MVP should include the following tuning/configuration values:

- Direction inversion per thruster
- Deadzone
- Minimum command / startup power
- Maximum command / power limit
- Neutral PWM value
- Minimum PWM value
- Maximum PWM value
- Trim offset
- Scale/gain multiplier
- Ramp rate / slew limit
- Test pulse duration
- Test pulse power
- Thruster physical location/name
- Motor controller channel/pin mapping

### MVP Motor Test Safety Defaults

- Use preset test sequences rather than arbitrary manual sliders.
- Require explicit confirmation before starting any motor/thruster test.
- Keep test power low by default.
- Use short timed pulses.
- Show which thruster is being tested.
- Provide a cancel/stop button during tests.
- Use the simple `control enabled / disabled` state rather than a complicated armed/disarmed model for MVP.

### MVP Camera Setup

Setup Mode should include basic camera configuration:

- Enable/disable camera
- Camera display name
- Camera index/device path
- Stream URL or MediaMTX path
- Test preview
- Resolution/FPS display if available

Camera configuration should be saved in JSON.

### Sensor Calibration

Sensor calibration is deferred for MVP because current MVP science functionality only uses cameras.

Potential future calibration tools:

- IMU zeroing
- Pressure/depth offset
- Temperature sensor check
- Leak sensor check
- Battery/voltage calibration

### Design Goal

Setup Mode should help users prepare the ROV without requiring developer knowledge.

## 9. System Test / Preflight

### Purpose

Let users verify basic functions such as cameras, motors, controls, and ROS2 communication before operation.

### Placement

For MVP, System Test / Preflight should live inside Setup Mode rather than becoming a fifth top-level mode.

### Test Model

- Tests are automated after the user starts each individual test.
- The tester must press explicit buttons to start tests.
- No single "Run All Tests" button for MVP.
- Test results should be saved to a log file.
- Failed checks should issue warnings but not block Pilot Mode.

### Camera Test

Verify:

- Stream exists
- Frame rate
- Resolution
- Latency, if practical

### Motor/Thruster Test

Verify:

- Each motor can receive PWM/test command
- Each thruster movement sequence runs as expected
- Test should send PWM to each motor using a preset sequence
- Tests should be safe, low-power, and cancelable

### Controller Test

Provide separate tests for:

- Physical joystick input
- Web joystick/gamepad input

Each test should verify that input is received and interpreted correctly.

### Sensor/ROS2 Test

Verify:

- Topic exists
- Recent message is received
- Value is within sane range where applicable
- Required nodes/services are reachable

### Safe Test Mode Clarification

A "safe test mode" does not need to be a complex new vehicle state for MVP. It can simply mean:

- Normal driving commands are disabled during a test.
- Only the active test command is allowed.
- Test commands are limited in power and duration.
- Emergency stop always overrides the test.
- The UI clearly shows that a test is active.

## 10. Developer Mode

### Purpose

Give maintainers and advanced users access to system, ROS2, and backend debugging tools without overwhelming normal users.

### MVP Features

- Customizable computer stats display
  - Developer can choose which stats are shown.
  - Each selected stat appears in its own box.
- General system stats should use the lowest-CPU approach available.
  - Prefer Python libraries like `psutil` over repeatedly spawning subprocess commands when practical.
- ROS2 node monitoring
  - Prefer a dedicated monitor node that observes required nodes and reports status to the frontend.
- ROS2 developer tools
  - View allowlisted topics/services/parameters first.
  - Avoid arbitrary publish/service-call tools in MVP unless clearly marked as dangerous developer tools.
  - Parameter changes in Developer Mode are temporary debugging changes only.
  - Persistent changes should be made through Setup Mode and saved into JSON config / launch configuration.
- Logs
  - Show only important frontend-facing logs such as errors and warnings.
  - Full logs remain on the computer for developers to inspect directly.
- Node/service control
  - Include restart and shutdown buttons for selected developer nodes/services.

### Recommended System Stats

Potential stat boxes:

- CPU usage
- Memory usage
- Disk usage
- Disk remaining
- CPU temperature, if available
- Network throughput/status
- Camera recording/storage usage
- Process status for key services
- ROS2 monitor status

### Recommended Node Health Model

Use a simple health model for MVP:

- Required node is visible
- Required topic is publishing recently
- Required service is available
- Optional heartbeat/diagnostic message is fresh, if available
- Node is marked healthy/warning/error based on these checks

### Recommended Topic/Service Policy

For MVP, use allowlists:

- Allowlist safe topics to view.
- Allowlist safe services to call.
- Allowlist parameters that can be edited.
- Mark dangerous actions with confirmation prompts.
- Do not include arbitrary message publishing unless specifically needed for debugging.

### Design Goal

Developer Mode can be technical, but it should be organized, readable, and safe by default.

## 11. Safety and Permissions

### Confirmed

Dangerous actions should require confirmation, including:

- Emergency stop
- Developer node shutdown
- Potentially dangerous parameter edits

Motor tests do not necessarily need a special developer permission level for MVP, but they should require clear confirmation.

### Permission Recommendation

Avoid a complex user/role permission system for MVP unless it becomes necessary. Since the UI is LAN-only with no authentication, use simple confirmations and mode separation instead.

Potential future permission layers:

- Normal operator
- Developer unlock
- Admin/setup mode

### Fail-Safe Scenarios

Thrusters should stop automatically when:

- Browser control connection is lost during active web control
- Backend loses contact with the active control source
- ROS2 safety-critical messages stop unexpectedly
- Emergency stop is pressed
- Motor test is canceled or times out
- Preflight/test command exceeds its allowed duration
- A safety monitor reports a critical fault
- Disk space margin is crossed while recording and recording shutdown requires preventing further unsafe operation

## 12. Backend/API Architecture
### Requests and Responses

Many web requests and responses should mirror their ROS2 counterparts in naming and structure, if possible.

### MVP Architecture

Use a split architecture:

- Flask code lives in `slvrov_core_python` and serves the web UI through a single understandable server entry point.
- UI assets for Setup, Pilot, Developer, and any MVP Scientist/capture screens should be served by that Flask server from clearly separated static/template directories inside `slvrov_core_python`.
- MediaMTX support files live under `slvrov_core_python` unless a later owner decision creates a dedicated runtime/config location.
- A ROS2 bridge node or service layer handles ROS2-specific communication.
- Shared objects or clearly defined interfaces can connect Flask to ROS2 logic where appropriate, but keep the boundary understandable.
- Avoid putting too much ROS2 complexity directly inside route handlers.
- Treat ROS2 nodes needed by the web UI as project-owner-authored code by default. A coding agent can create the package structure, interface definitions, adapter contracts, fake adapters, tests, and documentation, but should leave meaningful node behavior for the owner unless explicitly told otherwise.

### Likely Web-Facing ROS2 Nodes

Some web UI features may need small ROS2 nodes or service layers inside `slvrov_core_python`. These should be planned as clear extension points so the project owner can write the ROS2 logic.

Potential nodes or service layers include:

- `web_ros_bridge_node`: allowlisted bridge for web-facing ROS2 service calls, topic reads, and status summaries.
- `web_safety_bridge_node` or `safety_monitor_node`: emergency stop, control enabled/disabled state, browser disconnect handling, and critical safety fault reporting.
- `web_preflight_bridge_node` or `preflight_test_node`: Setup Mode test orchestration, test progress, cancel/stop behavior, and test result reporting.
- `web_developer_monitor_node` or `ros_health_monitor_node`: required-node visibility, topic freshness, service availability, and allowlisted developer diagnostics.
- `media_capture_node` or `science_capture_node`: optional camera capture/recording command handling if Scientist Mode capture is better kept in ROS2 than in the Flask/MediaMTX layer.
- `storage_monitor_node`: optional storage warning/stop-threshold reporting if storage state should be published into ROS2/global alerts.

These names are suggestions, not final requirements. During implementation, coding agents should document which node boundary they expect and provide fakes or tests so the owner can fill in the ROS2 node internals.

### REST / Live Update Strategy

Use REST polling first for MVP because it is simpler to understand and maintain.

REST endpoints should cover:

- Start/stop recording
- Take photo
- Save settings
- Start a test
- Call an allowlisted service
- Update config
- Fetch camera/control status
- Fetch test progress
- Fetch developer stats
- Fetch ROS2 node health
- Fetch alerts/warnings

WebSockets or Server-Sent Events can be added later only if REST polling becomes insufficient for responsiveness or efficiency.

### Flask Route Planning

Before implementing feature routes, the agent should create a route map for owner review. The route map should list each planned endpoint, HTTP method, request body, response shape, backing adapter/service, expected fake implementation, and whether it may eventually call an owner-authored ROS2 node.

Route planning should happen before major backend work for:

- Global safety and alerts
- Camera configuration and status
- Media capture and recent file history
- Setup configuration screens
- Control mapping
- Thruster configuration
- Preflight/test actions
- Developer system stats
- ROS2 health monitoring
- Storage warnings and stop thresholds

### Error Response Recommendation

Use structured JSON error responses:

```json
{
  "ok": false,
  "error_code": "CAMERA_STREAM_UNAVAILABLE",
  "message": "Camera 1 stream is not available.",
  "details": {
    "camera_id": "camera_1"
  },
  "suggested_action": "Check that the camera is plugged in and MediaMTX is running."
}
```

Successful responses should use a similarly predictable shape:

```json
{
  "ok": true,
  "data": {}
}
```

### MVP Config Files

Use JSON files under `rov_config/` for:

- `rov_config/motors/`: thruster configuration and motor controller channel/pin mappings
- `rov_config/actions/`: action definitions and action-type mapping data
- `rov_config/controls/`: joystick/control profiles and related mappings
- `rov_config/rovs/`: complete ROV configuration files that select the motor/action/control/camera/safety/storage profiles used by launch files

Camera configuration, UI preferences/layout presets, required ROS2 nodes/topics/services, safety thresholds, and recording/storage settings should either live in the relevant `rov_config/rovs/` definition or in owner-approved companion files referenced by the ROV definition.

Setup Mode should be the place where persistent config changes are made.

## 13. Data and File Storage

### Directory Layout

Store collected data outside the source code repository.

Recommended high-level layout:

```text
slvrov_workspace/
├── slvrov_ros/
└── data/
    ├── photos/
    ├── videos/
    ├── csv/
    ├── preflight_logs/
    ├── test_logs/
    └── metadata/
```

### File Access

- Photos should be viewable in the web UI.
- Video and CSV downloads are not required for MVP.
- Include a simple recent photos gallery.
- Full file browser and downloads are not required for MVP.

### File Size and Storage Limits

- Each data type should have a maximum size/usage policy based roughly on around 60 minutes of data.
- Remaining storage should be monitored.
- When remaining storage crosses a warning threshold, show warnings such as:
  - "Only about 2 minutes of video recording remaining on camera 1."
  - "Only about 30 more data points can be stored for sensor 5."
- When storage reaches the stop threshold:
  - Automatically stop recording.
  - Save the current file cleanly.
  - Warn the user.

## 14. UI/UX

### Theme

- Dark theme by default.
- Use the existing project theme as the starting point.
- A light theme can be considered later if it does not add significant complexity.

### Display Target

- Optimize for large displays such as monitors or laptops.
- Target 4:3 and 16:10 layouts.
- No touch-specific UI for MVP.

### Style Direction

Use a hybrid style by mode, and create visual mockups before committing to final implementation details.

- Pilot Mode should lean toward a simple vehicle control panel: clear camera view, minimal distraction, obvious safety controls.
- Scientist Mode should lean toward a scientific/data-capture dashboard: camera capture controls, recent photos, recording state, storage warnings.
- Setup Mode should be structured and step-by-step.
- Developer Mode can be more technical, with box-based stats and allowlisted tools.

Before implementation, create at least two static mockups:

1. Simple vehicle-control style
2. Scientific/data dashboard style

Then combine the useful parts into the mode-specific hybrid design.

### UI Demo Selection Workflow

Before implementing final UI screens, the agent should create small static demos for owner review. These demos should be quick to inspect, disposable, and focused on decisions rather than production polish.

Recommended demos:

- Main page: at least 2 mode-selection approaches.
- Global safety: at least 2 emergency-stop/alert placements.
- Pilot Mode: at least 3 camera layout/control-density approaches.
- Scientist Mode: at least 2 capture-focused layouts.
- Setup Mode: at least 2 step-by-step configuration flows.
- Developer Mode: at least 2 dense technical dashboard layouts.

After the owner selects a direction, the agent should implement only the selected direction and remove or clearly archive unselected demos.

### Status Bar

- A global top status bar is not required in every mode.
- Setup Mode may benefit from a status bar to help users keep track of setup state.
- Critical alerts should still be global across all modes.

### Layout Customization

- Use fixed/preset layouts for MVP.
- Avoid draggable/resizable layout customization for MVP.
- Avoid adding backend complexity just to support custom layouts.

## 15. Agentic AI Implementation Plan

An agent should not be given uncontrolled authority to build the whole system at once. Instead, the project should be split into clear, reviewable implementation units.

### Agent Workflow Requirements

The agent should:

1. Create the initial structure:
   - Directories
   - Files
   - Package layout
   - Starter configs
2. Build each mode independently while keeping the whole system architecture in mind.
3. Split each mode into sub-features.
4. Write tests for each sub-feature.
5. Test from service/topic-level functionality upward to UI-level behavior.
6. Avoid complex dependencies where practical.
7. Include clear comments explaining ROS2/Flask/frontend flow for amateur maintainers.
8. Provide setup instructions.
9. Use strict one-sub-feature-at-a-time prompts or review checkpoints so the project owner decides when each sub-feature is created.
10. Keep new ROS2 node internals as project-owner-authored work unless explicitly instructed otherwise.
11. When a feature needs ROS2 node behavior, define the interface, config, fake adapter, test expectations, and maintainer comments first, then leave a clear owner-facing TODO or minimal stub for the node logic.
12. Create decision packets before implementation for UI demos, Flask routes, config schema choices, safety behavior, ROS2 node boundaries, storage policy, and hardware-dependent behavior.
13. Stop after each decision packet with a short recommendation, clear tradeoffs, and the exact owner decision needed before implementation.

### Suggested Agent Build Order

1. Inventory existing packages and create the decision log.
2. Decide the shared API response shape.
3. Plan ROS2/web boundaries and owner-authored node TODOs.
4. Plan JSON config files and ownership rules.
5. Plan Flask routes after response, ROS2 boundary, and config decisions are known.
6. Plan the structural migration from the current repo layout to the target `slvrov_core_python` / `slvrov_interfaces` / `slvrov_launch` / `rov_config` layout.
7. Create or update workspace/package skeletons and placeholder docs.
8. Implement shared API response and config helpers.
9. Create UI demo shell, main page demos, global safety demos, Setup flow options, and mode demos.
10. Implement the selected theme, frontend utilities, Flask shell, and main page inside `slvrov_core_python`.
11. Implement global safety API/UI shell with fake ROS2 adapter.
12. Implement camera config/status API, one-stream WebRTC prototype, and selected Pilot layouts.
13. Decide storage policy before media capture behavior depends on it.
14. Plan and implement Scientist media capture in backend and UI sub-steps, with package placement decided before implementation.
15. Plan and implement Setup camera, control mapping, and thruster configuration in separate sub-steps.
16. Plan and implement Preflight/test API/UI with fake ROS2 adapters.
17. Plan and implement Developer stats and ROS2 monitor in separate sub-steps.
18. Implement storage limit handling and global alert integration.
19. Audit old scaffold packages/config paths before removing anything.
20. Audit routes, add end-to-end tests, and write docs/manual hardware checklists.

## 16. Resolved MVP Decisions

The following decisions have been resolved for the MVP.

| Area | MVP Decision | Future Notes |
|---|---|---|
| Frontend stack | Plain HTML, CSS, and JavaScript | Reconsider a framework only if frontend complexity becomes hard to manage. |
| Web UI joystick controls | Physical joystick only for MVP | Add web joystick/controller controls in a future version. |
| Camera layouts | Fixed/preset layouts only | Draggable/resizable layouts can be explored later. |
| Vehicle state | Simple `control enabled / disabled` state | Full arming/disarming can be added later if needed by the motor-control architecture. |
| Thruster tuning | Use the recommended MVP parameter set | Refine exact values after hardware testing. |
| Motor test safety | Use recommended low-power, timed, preset pulse policy | Exact pulse duration and power should be chosen during hardware testing. |
| Camera setup | Include basic camera setup in Setup Mode | Keep saved configuration in JSON. |
| Sensor calibration | Defer all sensor calibration | Add when non-camera science sensors are introduced. |
| Developer ROS2 access | Allowlisted topics, services, and parameters only | Avoid arbitrary ROS2 access from the browser for MVP. |
| Backend live updates | REST polling first | Upgrade to WebSockets/SSE later only if needed. |
| File history | Simple recent photos gallery | Full file browser and downloads are future features. |
| Visual style | Hybrid by mode, with mockups first | Pilot simple/control-focused; Scientist capture/data-focused; Developer technical. |
| Agent workflow | Strict one-sub-feature-at-a-time prompts/checkpoints | Each sub-feature should include tests before moving on. |
| ROS2 node authorship | Project owner writes new ROS2 node internals by default | Coding agents should provide scaffolds, contracts, fakes, tests, and TODOs unless asked to implement node behavior. |
| Owner control | Decision packets before implementation | Use demos, route maps, schemas, and tradeoff notes so the owner chooses before agents build. |
| Target structure | Consolidate web/UI/MediaMTX support into `slvrov_core_python`; keep interfaces and launch separate; use `rov_config/` for ROV configuration | The old web/science scaffolds and root `config/` tree have been migrated into the new structure. |

## 17. Remaining Details To Decide During Implementation

These are not blocking architectural decisions, but they should be finalized when the relevant sub-feature is implemented.

### Camera Details

- Exact USB camera device naming strategy.
- Exact MediaMTX path naming convention for up to 6 cameras.
- Whether camera previews should auto-detect available cameras or only use JSON-defined cameras.

### Motor/Thruster Details

- Exact thruster count, physical layout, and naming convention.
- Exact PWM range, neutral value, and low-power test pulse values.
- Exact motor controller hardware and pin/channel mapping.

### Control Mapping Details

- Exact joystick axes/buttons to support in the first control profile.
- Whether profiles are selected manually or auto-selected by joystick device name.
- Exact JSON schema for saved control profiles.

### Developer Mode Details

- Exact required-node list.
- Exact allowlisted topics, services, and parameters.
- Exact restart/shutdown actions that should be exposed.

### Storage Details

- Exact storage directory path on the target machine.
- Exact warning and stop thresholds for remaining disk space.
- Approximate video/photo storage usage for 60 minutes of data per type.
