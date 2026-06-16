# MVP Decision Log And Open Questions

Task 2 decision log for owner-controlled MVP choices.

Use this file to keep confirmed decisions separate from recommended defaults and
open questions. Later decision packets should update this log after owner
review, not silently promote recommendations into decisions.

Sources:

- [MVP spec](summer_2026_updates_MVP.md)
- [Agent task list](mvp_agent_tasks.md)
- [Target repo structure](slvrov_ros_structure.md)
- [Task 1 inventory](existing_system_inventory.md)
- [Shared config README](../config/README.md)
- [Data directory note](../docs/data_directory.md)
- [ROS2 Python style guide](../system_documents/ros2_python_style_guide.md)
- [SLVROV Web UI design guide](../system_documents/ui_design)

## Status Key

- `Confirmed`: Decided in the MVP spec or by owner review.
- `Recommended`: A proposed default, not yet owner-approved unless also listed
  as confirmed.
- `Open`: Needs owner decision before related implementation work continues.

## Decision Records

| ID | Area | Decision | Status | Source / Next Step |
|---|---|---|---|---|
| D-001 | Frontend stack | Use plain HTML, CSS, and JavaScript. | Confirmed | MVP spec section 16 |
| D-002 | Backend | Use Flask. | Confirmed | MVP spec section 2 |
| D-003 | Config format | Use JSON for MVP config files. | Confirmed | MVP spec sections 2 and 12 |
| D-004 | Camera stack | Use USB cameras, MediaMTX, and WebRTC; support up to 6 cameras. | Confirmed | MVP spec section 2 |
| D-005 | Live updates | Start with REST polling; add WebSockets/SSE only if polling is insufficient. | Confirmed | MVP spec sections 2, 12, 16 |
| D-006 | Joystick controls | Physical joystick input only for MVP; web joystick controls are future work. | Confirmed | MVP spec sections 2, 6, 8, 16 |
| D-007 | Vehicle state | Use simple `control enabled / disabled`, not full arming/disarming. | Confirmed | MVP spec sections 2 and 16 |
| D-008 | Camera layouts | Use fixed/preset camera layouts only for MVP. | Confirmed | MVP spec sections 2, 6, 14, 16 |
| D-009 | File history | Runtime logs only for MVP; photo/video/CSV browsing is deferred with science/capture scope. | Confirmed | MVP spec sections 2, 13, 16 |
| D-010 | ROS2 node authorship | Owner writes new ROS2 node internals by default; agents provide scaffolds, interfaces, fakes, tests, and TODOs. | Confirmed | MVP spec sections 2, 12, 15, 16 |
| D-011 | Preflight placement | System Test / Preflight lives inside Setup Mode, not as a fifth top-level mode. | Confirmed | MVP spec section 9 |
| D-012 | Persistent settings | Persistent config changes should be made through Setup Mode. | Confirmed | MVP spec section 12 |
| D-013 | UI demo workflow | Create a broad static UI direction demo before detailed page/mode demos so owner can choose a base visual direction. | Confirmed | `mvp_agent_tasks.md` Task 10A |
| D-013A | UI demo backend policy | UI demos are static-only and backend-free: no Flask routes, API calls, ROS2, MediaMTX/WebRTC, runtime config reads, or backend adapters. Use local fixtures/mock state only. | Confirmed | Owner instruction, 2026-06-15 |
| D-014 | Target structure | Use `src/slvrov_core_python`, `src/slvrov_interfaces`, `src/slvrov_launch`, and root `rov_config/`; keep core control code under `slvrov_core_python.control`, Flask/UI code under `slvrov_core_python.web`, and MediaMTX support under `slvrov_core_python.mediamtx`. | Confirmed | `slvrov_ros_structure.md`; owner structural update |
| D-015 | Tools dependency | Remove the `slvrov_tools_vendor` ROS package/submodule; install the real `slvrov-tools` repo into the active Python virtual environment. | Confirmed | Owner structural update; `system_documents/setup.md` |
| D-016 | Science scope | Remove the `slvrov_core_python.science` scaffold from the MVP package; defer Scientist Mode, photo/video capture, and sensor data workflows. | Confirmed | Owner structural update |

## Structural Migration

Confirmed:

- `slvrov_core_python` should contain ROS2 control nodes/helpers under
  `slvrov_core_python.control`, Flask server/UI code under
  `slvrov_core_python.web`, MediaMTX support files under
  `slvrov_core_python.mediamtx`, and production UI for Setup, Pilot, and
  Developer modes.
- `slvrov_interfaces` stores custom interfaces and should split services by
  node or feature area where useful.
- `slvrov_launch` stores launch files for each ROV configuration in
  `rov_config/rovs/`.
- `rov_config/` should contain grouped configuration under `motors/`,
  `actions/`, `controls/`, and `rovs/`.

Current repo layout:

- `src/slvrov_core_python/` already exists and contains control nodes,
  joystick mapper logic, JSON helpers, hardware-related code, web/UI scaffold
  directories, and MediaMTX support notes.
- Control code is grouped under
  `src/slvrov_core_python/slvrov_core_python/control/`.
- `src/slvrov_web_ui/`, `src/slvrov_science_python/`, and
  `src/slvrov_tools_vendor/` have been removed.
- `slvrov_tools` is installed into the active Python virtual environment from
  the real upstream repository instead of being vendored as a ROS package.
- `rov_config/` contains MVP example configs and schema notes grouped by
  `motors/`, `actions/`, `controls/`, and `rovs/`.
- `docs/` currently contains data-directory documentation; docs that support
  runtime structure can remain root-level if useful, but package-specific docs
  should move under the owning package.

Recommended defaults, not owner decisions:

- Continue using `src/slvrov_core_python/slvrov_core_python/web/` with
  `routes/`, `templates/`, `static/`, and `adapters/` subdirectories for
  Flask/UI work.
- Continue using `src/slvrov_core_python/slvrov_core_python/mediamtx/` for
  MediaMTX config templates, helper scripts, and maintainer notes.
- Keep control nodes, control data objects, control JSON helpers, and
  hardware-control helpers under
  `src/slvrov_core_python/slvrov_core_python/control/`.
- Keep reviewed runtime config examples in `rov_config/` by category.

Open questions:

| Question | Owner | Next-step task |
|---|---|---|
| Should root `config/` stay removed, or should a future docs-only config example area be recreated? | Project owner | Config schema task |
| Should root `docs/` remain for cross-package documentation, or should more docs move into package-specific folders? | Project owner | Structural migration task |

## UI Direction

Confirmed:

- Three top-level modes: Pilot, Setup, Developer.
- Dark theme by default, based on the existing project theme.
- Optimize for large monitor/laptop displays, especially 4:3 and 16:10.
- Touch-specific UI is not required for MVP.
- Main page should not remember the last selected mode.
- Pilot should prioritize camera visibility and low distraction.
- Setup should be structured and step-by-step.
- Developer can be technical, with organized stats and allowlisted tools.
- UI demos and production UI should follow `system_documents/ui_design`.
- UI should use clear affordances, semantic colors, stable spacing, readable
  typography, and visible hover/pressed/disabled/loading/error states.
- Demos should include normal, empty/unavailable, warning, and critical states.
- UI demos should use static fixtures or in-page mock state only; backend
  integration begins only after owner approval.
- Create static mockups before final implementation choices are locked.

Recommended defaults, not owner decisions:

- Use `src/slvrov_core_python/slvrov_core_python/web/static/` for final packaged frontend
  files.
- Use existing `ui_static/joystick_mapper_ui_v3/` only as reference material for
  Setup Mode mapping flow, not as the final UI.
- Include apple/macOS-inspired, existing-project, hybrid, and agent-recommended
  style families in the static UI direction demo, interpreted through the ROV
  safety/operator context rather than as generic SaaS or landing-page styles.
- Keep unselected demos disposable; remove or clearly archive them after owner
  selection.

Open questions:

| Question | Owner | Next-step task |
|---|---|---|
| Which base visual style should guide the MVP UI: apple/macOS-inspired, existing project style, hybrid, or agent-recommended? | Project owner | Task 10A |
| Which main page mode-selector layout should be implemented? | Project owner | Task 10A, then Task 11 |
| Which global emergency-stop and critical-alert placement should be used? | Project owner | Task 10A, then Task 12 |
| Which Pilot Mode camera layout presets should be MVP defaults? | Project owner | Task 10A, then Task 14 |
| Should Setup Mode always show a status bar, or only on specific setup screens? | Project owner | Task 10A, then Task 13/14 |
| Which Developer Mode stat dashboard density is preferred? | Project owner | Task 10A, then Task 14 |
| Which parts of `system_documents/ui_design` should become hard production rules versus demo-only guidance? | Project owner | Task 10A |

## Flask Route Design

Confirmed:

- Flask serves the web UI and exposes simple HTTP endpoints.
- Routes should avoid embedding ROS2 complexity directly in handlers.
- A route map must be reviewed before feature routes are implemented.
- Successful responses should use a predictable JSON shape like
  `{ "ok": true, "data": {} }`.
- Error responses should use structured JSON with `ok`, `error_code`,
  `message`, `details`, and `suggested_action`.
- Planned route areas include health, safety, alerts, camera status/config,
  setup config, control mapping, thruster config, preflight tests, developer
  stats, ROS2 health, and storage status.

Recommended defaults, not owner decisions:

- Keep route modules under `src/slvrov_core_python/slvrov_core_python/web/routes/`.
- Put ROS2, MediaMTX, or filesystem work behind adapter interfaces under
  `src/slvrov_core_python/slvrov_core_python/web/adapters/`.
- Keep response helper design in a small shared module after Task 3 approves the
  response shape.

Open questions:

| Question | Owner | Next-step task |
|---|---|---|
| What exact shared API response shape should all routes use? | Project owner | Task 3 |
| What route naming conventions should the MVP use? | Project owner | Task 6 |
| Which routes are MVP versus future-only? | Project owner | Task 6 |
| Which route groups may call real ROS2 services immediately, and which must use fakes first? | Project owner | Tasks 4 and 6 |
| Should health/status endpoints expose detailed live status on the main page, or keep it minimal? | Project owner | Task 6 or future main page task |

## Config Schema Choices

Confirmed:

- Config files use JSON.
- Setup Mode is the place for persistent config changes.
- MVP config areas include control mapping, thrusters, pin mappings, cameras,
  UI preferences/layout presets, ROS2 requirements, safety thresholds, and
  runtime log/storage settings.
- Starter examples now live under `../rov_config/`.

Recommended defaults, not owner decisions:

- Keep reviewed runtime config files under `rov_config/motors/`,
  `rov_config/actions/`, `rov_config/controls/`, and `rov_config/rovs/`.
- Put formal JSON Schema files or documented validation contracts in
  `rov_config/schemas/` or package-local validator docs if the owner chooses
  schema files.
- Keep runtime generated logs and future media outside the repo, following
  `../docs/data_directory.md`.
- Avoid reusing `submersed_globals.py` current-working-directory persistence
  behavior for new MVP web config paths.

Open questions:

| Question | Owner | Next-step task |
|---|---|---|
| Should validation use formal JSON Schema files, Python validation helpers, or both? | Project owner | Task 5 |
| What exact top-level fields should each MVP config file use? | Project owner | Task 5 |
| Which Setup Mode screen owns saving each config file? | Project owner | Task 5 |
| Should control profiles be selected manually or auto-selected by joystick device name? | Project owner | Task 5 or control mapping task |
| Should ROS2 launch/config files be generated from JSON, manually maintained, or both? | Project owner | Task 5 |

## ROS2 Node Boundaries

Confirmed:

- New ROS2 node internals are owner-authored by default.
- Agents should provide interfaces, fake adapters, tests, docs, and TODO notes
  before real node behavior.
- Existing interfaces live in `src/slvrov_interfaces/`.
- Existing core control/hardware code lives in `src/slvrov_core_python/`.
- New web-facing Flask/UI work should live in `slvrov_core_python`.

Recommended defaults, not owner decisions:

- Use `web_ros_bridge_node` as the broad allowlisted service/topic bridge name
  unless Task 4 chooses a better name.
- Keep safety, preflight, developer monitoring, and storage
  monitoring as separate boundaries when that makes ownership easier.
- Use fake adapters for early Flask/UI tests until owner-authored ROS2 nodes are
  ready.

Open questions:

| Question | Owner | Next-step task |
|---|---|---|
| Which MVP features are Flask-only, fake-adapter-backed, existing-ROS-backed, or new-node-backed? | Project owner | Task 4 |
| Which suggested node names should become final? | Project owner | Task 4 |
| Which owner-authored nodes should be written first? | Project owner | Task 4 |
| Should storage warnings be Flask-only or published through a ROS2 storage monitor/global alert path? | Project owner | Task 4 and storage policy |
| Which additional launch files should be updated to use `rov_config/rovs/` as new ROV configs are approved? | Project owner | Future launch task |

## Safety Behavior

Confirmed:

- Emergency stop must be visible globally.
- Critical alerts should appear globally across modes.
- Browser disconnect, backend control-source loss, ROS2 safety-critical message
  loss, emergency stop, canceled/timed-out motor tests, test duration overruns,
  critical safety monitor faults, and storage stop-threshold conditions should
  fail safe by stopping thrusters where relevant.
- Dangerous actions require confirmations.
- Motor tests use low-power, timed, preset pulses with cancel/stop controls.
- Failed preflight checks warn the user but do not block Pilot Mode for MVP.
- MVP does not use a complex role/permission system.

Recommended defaults, not owner decisions:

- Model global motion state as `control enabled / disabled`.
- Make emergency stop override every test workflow that could
  affect safety.
- Keep dangerous Developer Mode actions allowlisted and confirmation-gated.

Open questions:

| Question | Owner | Next-step task |
|---|---|---|
| What exact emergency-stop backend path should Flask call first? | Project owner | Task 4 and future safety task |
| Which ROS2 messages/services count as safety-critical for disconnect/freshness checks? | Project owner | Task 4 |
| What exact motor test pulse power and duration should be used on hardware? | Project owner | Hardware-dependent decision |
| What critical alert categories are MVP versus future-only? | Project owner | Task 6 or future safety task |
| Should disk stop-threshold behavior only stop recording, or also disable motion in some cases? | Project owner | Storage policy task |

## Camera Strategy

Confirmed:

- Use USB cameras.
- Support up to 6 cameras.
- Use MediaMTX with WebRTC for streaming.
- Pilot Mode needs single-camera and multi-camera fixed/preset layout support.
- Science capture, photo/video recording, and sensor display/logging are
  deferred until the owner reopens that scope.

Recommended defaults, not owner decisions:

- Keep basic camera configuration in JSON and editable from Setup Mode.

Open questions:

| Question | Owner | Next-step task |
|---|---|---|
| What USB camera device naming strategy should be used? | Project owner | Task 5 or camera setup task |
| What MediaMTX path convention should be used for up to 6 cameras? | Project owner | Camera strategy/config task |
| Should camera previews auto-detect available cameras or only use JSON-defined cameras? | Project owner | Task 5 or camera setup task |

## Storage Policy

Confirmed:

- Store runtime data outside the source repository.
- Recommended layout is a sibling `data/` directory with `preflight_logs/`,
  `test_logs/`, and `metadata/`.
- Photo galleries, video downloads, CSV downloads, and full file browsing are
  future science/capture features.
- Remaining storage should be monitored.
- Warning and stop thresholds should exist.
- At the stop threshold, active logging or future capture should stop cleanly
  and the user should be warned.

Recommended defaults, not owner decisions:

- Use `../data/` relative to `slvrov_ros/` as the initial target-machine layout
  unless the owner chooses an absolute runtime path.
- Keep runtime storage settings in JSON under the future approved config
  shape.
- Show storage warnings globally when they affect active runtime behavior.

Open questions:

| Question | Owner | Next-step task |
|---|---|---|
| What exact storage directory path should the target machine use? | Project owner | Storage policy task or Task 5 |
| What warning and stop thresholds should be used for disk remaining? | Project owner | Storage policy task |
| Should storage status be Flask-only or backed by a ROS2 `storage_monitor_node`? | Project owner | Task 4 |

## Hardware-Dependent Decisions

Confirmed:

- ROS2 distribution is Jazzy Jalisco.
- Main target OS is Ubuntu.
- Network model is Ethernet between nodes/server.
- Physical joystick input uses Linux `js#` devices through ROS2 joystick nodes.
- Motor/thruster testing should use preset, confirmed, low-power, timed pulses.

Recommended defaults, not owner decisions:

- Keep hardware-specific values in JSON examples and owner-reviewed config,
  not hard-coded into Flask routes.
- Treat `src/slvrov_core_python/` hardware/control code as read-mostly until a
  task specifically targets it.

Open questions:

| Question | Owner | Next-step task |
|---|---|---|
| What exact thruster count, physical layout, and naming convention should MVP use? | Project owner | Task 5 or thruster config task |
| What PWM range, neutral value, and low-power test pulse values should be used? | Project owner | Hardware test task |
| What motor controller channel/pin mapping should be the approved baseline? | Project owner | Task 5 |
| Which joystick axes/buttons should be included in the first control profile? | Project owner | Control mapping task |
| Which required ROS2 nodes, topics, services, and parameters should Developer Mode monitor? | Project owner | Task 4 and Task 5 |
| Which Developer Mode restart/shutdown actions are safe enough to expose? | Project owner | Developer Mode task |

## Immediate Next Owner Review

Owner should confirm whether this log format is useful before Task 3 begins.
Task 3 should then decide the shared API response shape and update `D-013` (or
the next available decision ID) once approved.

No implementation should proceed from a `Recommended` item until the relevant
owner decision is recorded here or in a later decision packet.
