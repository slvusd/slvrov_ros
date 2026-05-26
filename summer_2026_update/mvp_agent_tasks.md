# ROV Web UI MVP Agent Tasks

This task list splits the MVP from `summer_2026_updates_MVP.md` into reviewable coding-agent prompts. Each task is intended to be completed independently, reviewed, and tested before the next task begins.

Every coding agent prompt below includes the following standing requirements:

- Follow `system_documents/ros2_python_style_guide.md` for ROS2 Python code.
- Add comments where amateur maintainers or maintainers-to-be would need help understanding ROS2, Flask, frontend, safety, or data-flow decisions.
- Keep implementation simple and readable.
- Add focused tests for the sub-feature.
- Test from service/topic-level behavior upward to API/UI behavior.
- Do not add broad unrelated refactors.
- Stop at the end of the task and report what changed, what was tested, and what remains.

## Task 1: Workspace And Package Skeleton

### Scope

Create the MVP package and directory structure needed for:

- `slvrov_web_ui`
- `slvrov_science_python`
- shared JSON config files
- data directory documentation
- starter Flask/frontend layout

### Coding Agent Prompt

Implement the initial MVP workspace structure described in `summer_2026_update/summer_2026_updates_MVP.md`. Create package skeletons for `slvrov_web_ui` and `slvrov_science_python` if they do not already exist, and add starter directories for Flask routes, static frontend files, templates if needed, config schemas/examples, tests, and docs.

Follow `system_documents/ros2_python_style_guide.md` for any ROS2 Python code. Add comments where amateur maintainers or maintainers-to-be would need help understanding package layout, ROS2 package conventions, Flask/frontend boundaries, or config-file purpose.

Do not implement full features yet. This task is only for structure, starter files, and a short setup note that explains where future MVP pieces belong.

### Tests And Checks

- Verify package manifests and setup files are syntactically valid.
- Run available package discovery/build checks if `colcon` is available.
- Add minimal tests that prove importable Python package skeletons import cleanly.

### Acceptance Criteria

- New packages/directories have clear names and expected ownership.
- Starter configs are valid JSON.
- Docs explain the structure for future maintainers.
- No feature-specific behavior is implemented beyond placeholders.

## Task 2: Shared Config Structure

### Scope

Define JSON config structure for:

- cameras
- control profiles
- thruster configuration
- pin mappings
- UI preferences/layout presets
- required ROS2 nodes/topics/services
- safety thresholds
- recording/storage settings

### Coding Agent Prompt

Create shared JSON config examples and Python helpers for loading, validating, and saving MVP web UI config files. Use simple JSON schemas or documented validation functions, whichever best fits the current repo style. Keep config ownership understandable: Setup Mode should own persistent changes, while Developer Mode changes should be temporary unless explicitly saved through Setup Mode later.

Follow `system_documents/ros2_python_style_guide.md` for ROS2 Python code. Add comments where amateur maintainers or maintainers-to-be would need help understanding config ownership, validation rules, or why a field exists.

Do not build UI screens yet. This task should only establish config files, helper functions, and tests.

### Tests And Checks

- Unit test valid config loading.
- Unit test missing files and invalid JSON errors.
- Unit test required fields and default values.
- Verify sample JSON files parse cleanly.

### Acceptance Criteria

- Config files cover all MVP areas listed in scope.
- Config helpers use clear errors and avoid silent fallback for malformed files.
- Tests document expected config behavior.

## Task 3: Flask App Shell And Static Frontend Shell

### Scope

Create the basic web app shell:

- Flask app factory or simple app entrypoint
- route organization
- static HTML/CSS/JS structure
- shared frontend utilities
- health endpoint
- development run instructions

### Coding Agent Prompt

Build the minimal `slvrov_web_ui` Flask application shell and static frontend shell. The app should serve the main page and mode pages as simple static experiences for now. Add a lightweight `/api/health` endpoint that returns structured JSON in the MVP response shape.

Follow `system_documents/ros2_python_style_guide.md` for Python code. Add comments where amateur maintainers or maintainers-to-be would need help understanding the Flask app boundary, static file layout, or API response convention.

Do not connect to ROS2 yet. Keep this task focused on the web shell.

### Tests And Checks

- Unit test Flask app creation.
- Test `/api/health` response shape.
- Verify static files are served.
- Run a local dev server if practical and inspect the rendered shell.

### Acceptance Criteria

- Main page and placeholder mode pages load.
- `/api/health` returns `{ "ok": true, "data": ... }`.
- Frontend structure is split clearly enough for future mode-specific code.

## Task 4: Main Mode-Selection Page

### Scope

Implement the main page with four mode entry points:

- Pilot
- Scientist
- Setup
- Developer

### Coding Agent Prompt

Implement the main mode-selection page for the MVP web UI. It should show four large navigation cards or buttons with short descriptions. It should not require login and should not remember the last selected mode.

Follow the visual direction in `summer_2026_update/summer_2026_updates_MVP.md`: clean, readable, approachable, dark theme by default, optimized for large displays. Add comments where amateur maintainers or maintainers-to-be would need help understanding shared frontend utilities or navigation behavior.

Do not implement mode internals in this task.

### Tests And Checks

- Frontend smoke test that all four navigation targets are present.
- Manual or browser-automation check at 4:3 and 16:10-ish viewport sizes.
- Verify no local storage/session behavior remembers the last selected mode.

### Acceptance Criteria

- Four mode entry points are visible and easy to select.
- Descriptions are concise and understandable.
- Page uses the shared dark theme.

## Task 5: Global Alerts And Emergency Stop Shell

### Scope

Add global safety UI and API shell for:

- visible emergency stop in all modes
- global critical alerts
- simple control enabled/disabled state
- backend endpoints for alert state and emergency stop state

### Coding Agent Prompt

Implement the global emergency stop and critical-alert UI shell. The emergency stop must be visible across all mode pages. Add backend API endpoints for reading alert state, triggering emergency stop, clearing emergency stop if safe, and reading/updating the simple `control enabled / disabled` state.

Follow `system_documents/ros2_python_style_guide.md` for Python code. Add comments where amateur maintainers or maintainers-to-be would need help understanding safety flow, why emergency stop is global, and where future ROS2 stop commands should connect.

Do not wire real motor shutdown yet unless an existing safe service already exists. Stub the ROS2 side clearly and safely.

### Tests And Checks

- Unit test API response shapes.
- UI test that emergency stop appears on all mode pages.
- Test that emergency stop state can be triggered and reflected in the UI.
- Test disabled control state is shown clearly.

### Acceptance Criteria

- Emergency stop is globally visible.
- Critical alerts can be displayed globally.
- API uses structured success/error responses.
- Real motor-control integration points are clearly marked.

## Task 6: Camera Configuration And WebRTC Display Prototype

### Scope

Prototype camera config and display:

- JSON camera config
- up to 6 USB cameras
- MediaMTX/WebRTC stream URL/path handling
- single test preview
- camera status endpoint

### Coding Agent Prompt

Implement the first camera configuration and WebRTC display prototype. Use JSON camera configuration to define enabled cameras, display names, device paths or indexes, and MediaMTX/WebRTC paths. Add backend endpoints to fetch camera config/status and a frontend preview component that can display one configured stream.

Follow `system_documents/ros2_python_style_guide.md` for Python code. Add comments where amateur maintainers or maintainers-to-be would need help understanding MediaMTX path naming, WebRTC embedding, and how camera config flows from JSON to UI.

Do not implement recording or multi-camera layouts yet.

### Tests And Checks

- Unit test camera config loading and validation.
- API test for camera config/status.
- Frontend smoke test that a configured camera appears in the preview UI.
- Document how to point a camera entry at a MediaMTX stream.

### Acceptance Criteria

- Up to 6 camera entries can be represented.
- Disabled cameras are not shown as active previews.
- Missing stream/config errors produce clear frontend-facing messages.

## Task 7: Pilot Mode Camera Layouts

### Scope

Build Pilot Mode camera viewing:

- single-camera view
- multi-camera grid view
- fixed/preset layout choices
- minimal distraction
- emergency stop remains visible

### Coding Agent Prompt

Implement Pilot Mode camera layouts using the camera configuration and WebRTC prototype. Support a selected single-camera view and fixed/preset multi-camera layouts for up to 6 cameras. Pilot Mode should prioritize camera visibility and keep extra information minimal.

Follow `system_documents/ros2_python_style_guide.md` for Python code. Add comments where amateur maintainers or maintainers-to-be would need help understanding camera layout presets, frontend state, or the boundary between camera config and display.

Do not add joystick visualization, telemetry, or web joystick controls.

### Tests And Checks

- Frontend tests for layout switching.
- Browser/manual checks at large 4:3 and 16:10 viewport sizes.
- Verify emergency stop is still visible in all layouts.
- Verify disabled cameras are not rendered as active streams.

### Acceptance Criteria

- Pilot Mode has clear single and multi-camera layouts.
- Layouts are fixed/preset only.
- No non-MVP controls are introduced.

## Task 8: Scientist Mode Media Capture

### Scope

Build camera-centered capture features:

- photo capture from each camera
- video recording from each camera
- timestamped default filenames
- optional filename override
- JPEG photos
- MP4 videos
- recent photos gallery

### Coding Agent Prompt

Implement Scientist Mode media capture for configured cameras. Add backend endpoints for taking photos, starting/stopping per-camera recording, reporting recording state, and listing recent photos. Use timestamped filenames by default and allow optional filename overrides with safe filename validation.

Choose the lowest-CPU recording strategy that works reliably with MediaMTX/WebRTC or document the chosen placeholder if hardware/MediaMTX integration cannot be fully tested locally. Use JPEG for photos and MP4 for videos.

Follow `system_documents/ros2_python_style_guide.md` for Python code. Add comments where amateur maintainers or maintainers-to-be would need help understanding capture flow, file naming, storage paths, or why a recording strategy was chosen.

Do not implement sensor data logging yet.

### Tests And Checks

- Unit test filename generation and override validation.
- API tests for photo and recording endpoints.
- Test recent photo listing.
- Use fake camera/media adapters where hardware is unavailable.

### Acceptance Criteria

- Capture endpoints have predictable structured responses.
- Media filenames are safe and timestamped by default.
- Recent photos gallery can render saved photo metadata.
- Hardware-dependent gaps are documented.

## Task 9: Setup Mode Control Mapping

### Scope

Build Setup Mode control mapping:

- physical joystick setup using Linux `js#`
- joystick axes/buttons mapping
- live interpreted input values
- save multiple control profiles

### Coding Agent Prompt

Implement Setup Mode control mapping for physical joystick input. Use existing joystick mapper/service concepts where practical, but keep the web-facing flow understandable. The UI should let users see live interpreted input values, map axes and buttons, and save multiple control profiles to JSON.

Follow `system_documents/ros2_python_style_guide.md` for ROS2 Python code. Add comments where amateur maintainers or maintainers-to-be would need help understanding joystick topics, `js#` devices, mapping services, profile JSON, or the flow from joystick input to saved mapping.

Do not implement web joystick/gamepad controls.

### Tests And Checks

- Unit test control profile JSON validation.
- Service/API tests for mapping state and profile save/load.
- Fake joystick-message tests for interpreted input values.
- UI smoke test for mapping workflow steps.

### Acceptance Criteria

- Users can create and save named control profiles.
- Axis/button action types use the `ROVActionType` string values.
- Live interpreted input display is clear enough for setup.

## Task 10: Setup Mode Thruster Configuration

### Scope

Build thruster tuning/config:

- thruster JSON config
- pin/channel mapping JSON
- MVP tuning parameters
- generated or updated ROS2 launch/config files where appropriate

### Coding Agent Prompt

Implement Setup Mode thruster configuration. Store thruster configuration and motor controller channel/pin mappings in JSON. Include all MVP tuning parameters from the specification: inversion, deadzone, startup power, max power, neutral/min/max PWM, trim, scale/gain, ramp rate, test pulse duration, test pulse power, physical location/name, and channel/pin mapping.

If launch/config generation is appropriate, implement it in a small, explicit helper with tests. Avoid hidden magic.

Follow `system_documents/ros2_python_style_guide.md` for ROS2 Python code. Add comments where amateur maintainers or maintainers-to-be would need help understanding tuning parameters, PWM safety, pin mappings, or generated file behavior.

Do not implement motor test execution in this task.

### Tests And Checks

- Unit test thruster config validation.
- Unit test pin/channel mapping validation.
- Test config save/load round trips.
- Test generated config/launch output if implemented.

### Acceptance Criteria

- Every MVP tuning parameter is represented.
- Invalid PWM/range values fail with clear errors.
- Config is saved as readable JSON.

## Task 11: System Test / Preflight Services And UI

### Scope

Build Setup Mode preflight:

- explicit individual test buttons
- camera test
- motor/thruster test shell
- controller test
- ROS2 node/topic/service test
- logs saved to file
- no single Run All Tests button

### Coding Agent Prompt

Implement the System Test / Preflight section inside Setup Mode. Add individual test controls for camera checks, motor/thruster test sequences, controller input checks, and ROS2 node/topic/service checks. Save test results to a log file. Failed tests should warn users but should not block Pilot Mode for MVP.

Use safe test mode behavior: disable normal driving commands during an active test, allow only the active test command, keep motor tests low-power and timed, provide cancel/stop, and ensure emergency stop overrides tests.

Follow `system_documents/ros2_python_style_guide.md` for ROS2 Python code. Add comments where amateur maintainers or maintainers-to-be would need help understanding preflight flow, safety limits, log format, or ROS2 test checks.

Do not add a single "Run All Tests" button.

### Tests And Checks

- Unit test test-result log writing.
- API tests for starting/canceling individual tests.
- Fake ROS2/topic/service checks for healthy/warning/error states.
- UI test that no Run All Tests button exists.

### Acceptance Criteria

- Each test can be started individually.
- Active tests are cancelable.
- Results are saved to logs.
- Failures show warnings but do not block Pilot Mode.

## Task 12: Developer Mode System Stats

### Scope

Build customizable stats:

- CPU usage
- memory usage
- disk usage/free space
- CPU temperature if available
- network throughput/status if practical
- process status for key services

### Coding Agent Prompt

Implement Developer Mode system stats with a customizable set of stat boxes. Prefer `psutil` or similarly low-CPU Python APIs over repeatedly spawning subprocesses. Let selected stats appear in their own boxes and keep the UI technical but readable.

Follow `system_documents/ros2_python_style_guide.md` for Python code. Add comments where amateur maintainers or maintainers-to-be would need help understanding stat collection, platform-specific fallbacks, polling frequency, or performance tradeoffs.

Do not implement ROS2 node health monitoring in this task.

### Tests And Checks

- Unit test stat collector with mocked system data.
- API test for stats response shape.
- Frontend test for selecting visible stat boxes.
- Confirm polling interval is not too aggressive.

### Acceptance Criteria

- Stats are returned in a predictable JSON shape.
- UI allows choosing displayed stats.
- Missing platform-specific stats degrade gracefully.

## Task 13: Developer Mode ROS2 Monitor

### Scope

Build allowlisted ROS2 monitoring:

- required node visibility
- recent topic publishing
- required service availability
- optional heartbeat freshness
- allowlisted topics/services/parameters

### Coding Agent Prompt

Implement Developer Mode ROS2 monitoring using a dedicated monitor node or clear ROS2 service layer. Report required node visibility, recent topic publishing, required service availability, and optional heartbeat/diagnostic freshness. Use allowlists for topics, services, and parameters.

Follow `system_documents/ros2_python_style_guide.md` for ROS2 Python code. Add comments where amateur maintainers or maintainers-to-be would need help understanding allowlists, ROS2 graph checks, freshness thresholds, or why arbitrary publishing/service calls are avoided.

Do not add arbitrary ROS2 publish tools in MVP.

### Tests And Checks

- Unit test health classification rules.
- Fake ROS2 graph/service/topic tests.
- API test for monitor status response.
- UI smoke test for healthy/warning/error display.

### Acceptance Criteria

- Health state is clear and simple: healthy/warning/error.
- Only allowlisted items are exposed.
- Dangerous actions are not available by default.

## Task 14: Storage Limit Handling

### Scope

Implement storage policy:

- storage warning threshold
- storage stop threshold
- remaining media estimate
- automatic recording stop at stop threshold
- warnings in global alerts

### Coding Agent Prompt

Implement MVP storage limit handling for media/data directories. Track disk remaining and estimate remaining photo/video capacity using a simple documented policy. Show warnings when warning thresholds are crossed and automatically stop recording cleanly when stop thresholds are reached.

Follow `system_documents/ros2_python_style_guide.md` for Python code. Add comments where amateur maintainers or maintainers-to-be would need help understanding storage thresholds, estimates, or recording shutdown flow.

Do not implement a full file browser or downloads.

### Tests And Checks

- Unit test storage threshold classification.
- Unit test remaining media estimate formatting.
- API test for storage status.
- Test recording stop hook with a fake recorder.

### Acceptance Criteria

- Warnings are clear and user-facing.
- Stop threshold cleanly stops active recordings.
- Storage config is saved/read from JSON.

## Task 15: End-To-End Tests And Documentation

### Scope

Add final MVP verification:

- setup instructions
- run instructions
- API summary
- config summary
- end-to-end smoke tests
- mode-by-mode manual checklist

### Coding Agent Prompt

Create the MVP end-to-end test and documentation package. Add setup/run instructions for the web UI and ROS2 pieces, document config files and important API endpoints, and add a mode-by-mode manual checklist for Pilot, Scientist, Setup, and Developer Mode.

Add automated end-to-end smoke tests where practical. Tests should verify the app can start, the main page loads, each mode route loads, health/status endpoints respond, and safety UI is visible.

Follow `system_documents/ros2_python_style_guide.md` for Python code. Add comments where amateur maintainers or maintainers-to-be would need help understanding test setup, fake adapters, or how to run the MVP without hardware.

### Tests And Checks

- Run all available unit and integration tests.
- Run browser smoke tests if the frontend can be served locally.
- Verify docs match actual commands and file paths.
- Confirm hardware-dependent behavior has clear fake/manual test notes.

### Acceptance Criteria

- A maintainer can follow docs to run the MVP shell.
- Tests cover the major API/UI paths.
- Manual checklist identifies what requires hardware.

