# ROV Web UI MVP Agent Tasks

This task list splits the MVP from `summer_2026_updates_MVP.md` into small, reviewable coding-agent prompts. The order is dependency-driven: decisions that shape later work come before implementation tasks that depend on them.

Each task should be completed independently, reviewed, and tested before the next task begins. Some tasks are decision packets rather than implementation work. A decision packet should produce options, tradeoffs, a recommendation, and the exact owner decision needed before implementation continues.

Every coding agent prompt below includes the following standing requirements:

- Follow `system_documents/ros2_python_style_guide.md` for ROS2 Python code.
- Follow `system_documents/ui_design` for UI demos and frontend implementation.
- Add comments where amateur maintainers or maintainers-to-be would need help understanding ROS2, Flask, frontend, safety, or data-flow decisions.
- Keep implementation simple and readable.
- Add focused tests for implementation tasks.
- Test from service/topic-level behavior upward to API/UI behavior when the task includes backend or ROS2-facing behavior.
- For implementation tasks, validate the finished branch on the Ubuntu VM using `system_documents/agent_vm_testing.md`: log in with `ssh codex-tester`, pull the branch being tested, build with `colcon`, run automated tests, and run relevant ROS2 smoke tests.
- Treat new ROS2 node internals as project-owner-authored work unless the owner explicitly asks the coding agent to implement them.
- For ROS2 node-related features, provide interfaces, adapters, fake implementations, tests, and clear TODO notes so the owner can write or finish the node logic.
- Do not add broad unrelated refactors.
- Stop at the end of the task and report what changed, what was tested, the VM branch/commit tested when VM testing applies, what remains, and what owner decision is needed next.

Implementation-task testing should follow the VM workflow in [Agent VM Testing Guide](../system_documents/agent_vm_testing.md). Decision packets and documentation-only tasks do not need VM testing unless they include runnable examples or generated files that should be verified in the Ubuntu ROS2 environment.

UI demo tasks are intentionally backend-free. Tasks 10, 10A, 11, 12, 14, and
any owner-requested demo variants must use static HTML/CSS/JavaScript and local
fixture data only. They must not create Flask routes, start ROS2 nodes, call
ROS2 services/topics, connect to MediaMTX/WebRTC streams, fetch from API
endpoints, or introduce backend adapter code. A local static file server is
allowed only to preview static files in the browser.

Owner-selected UI direction as of 2026-06-16:

- Continue from the Apple/macOS-inspired direction in
  `ui_static/mvp_demo_shell/prototypes/apple_macos_direction/`.
- Support dark and light mode.
- Do not use different colors for Pilot, Setup, and Developer modes.
- Use notification/state colors consistently: error red, warning yellow/orange,
  and status/active/connected/healthy light blue.
- Treat archived demos under `ui_static/archive/` and
  `ui_archive/legacy_ui_examples/` as reference-only, not active directions.

Target structure from `summer_2026_update/slvrov_ros_structure.md`:

- `src/slvrov_core_python/`: ROS2 control nodes/helpers under `slvrov_core_python.control`, Flask server/UI code under `slvrov_core_python.web`, MediaMTX support files under `slvrov_core_python.mediamtx`, and production UI for Setup, Pilot, and Developer modes.
- `src/slvrov_interfaces/`: custom messages, services, and actions, with services split by node or feature area where useful.
- `src/slvrov_launch/`: launch files for each approved ROV configuration.
- `rov_config/`: ROV configuration grouped by `motors/`, `actions/`, `controls/`, and `rovs/`.

Older MVP scaffolds from `src/slvrov_web_ui/`, `src/slvrov_science_python/`, `src/slvrov_tools_vendor/`, and root `config/` have been removed or consolidated into `slvrov_core_python`, `rov_config/`, and the virtual-environment setup flow. Future structural tasks should avoid recreating those old standalone package paths unless the owner explicitly asks for them.

Likely web-facing `slvrov_core_python` ROS2 node boundaries to keep in mind:

- `web_ros_bridge_node`: allowlisted bridge for web-facing ROS2 service calls, topic reads, and status summaries.
- `web_safety_bridge_node` or `safety_monitor_node`: emergency stop, control enabled/disabled state, browser disconnect handling, and critical safety fault reporting.
- `web_preflight_bridge_node` or `preflight_test_node`: Setup Mode test orchestration, progress, cancel/stop behavior, and result reporting.
- `web_developer_monitor_node` or `ros_health_monitor_node`: required-node visibility, topic freshness, service availability, and allowlisted diagnostics.
- `storage_monitor_node`: optional storage warning/stop-threshold reporting if storage state should feed ROS2/global alerts.

These are planning names, not final API requirements. When a task touches one of these areas, the coding agent should make the boundary easy for the project owner to complete.

## Structural Change Implementation Plan

Use this plan after the owner approves the structural migration decision packet:

1. Snapshot the current layout before any future structural work:
   - `src/slvrov_core_python/` contains current control nodes, joystick mapper code, JSON helpers, and hardware-related config.
   - `src/slvrov_core_python/slvrov_core_python/web/` contains web/UI scaffold directories.
   - `src/slvrov_core_python/slvrov_core_python/control/` contains control nodes and local control helpers.
   - `rov_config/` contains MVP config examples and schema notes.
   - `docs/` contains cross-package documentation that can remain root-level unless it becomes package-specific.
2. Create target directories without moving behavior yet:
   - `src/slvrov_core_python/slvrov_core_python/web/routes/`
   - `src/slvrov_core_python/slvrov_core_python/web/templates/`
   - `src/slvrov_core_python/slvrov_core_python/web/static/`
   - `src/slvrov_core_python/slvrov_core_python/web/adapters/`
   - `src/slvrov_core_python/slvrov_core_python/mediamtx/`
   - `rov_config/motors/`, `rov_config/actions/`, `rov_config/controls/`, and `rov_config/rovs/`
3. Move or copy only low-risk scaffold files first, preserving imports and tests.
4. Update `setup.py`, package data, tests, and docs to point at the new locations.
5. Run local import/tests, then validate implementation branches on the Ubuntu VM using the VM testing guide.
6. Remove or recreate structure only after the owner explicitly approves the exact path-level change.

## Task 1: Existing System Inventory

### Scope

Inventory the current repo before adding MVP code.

### Coding Agent Prompt

Review the existing packages, launch files, interfaces, joystick mapper code, JSON helpers, tests, root docs, `rov_config/`, and the current `slvrov_core_python` control/web/MediaMTX scaffolds. Produce a short inventory document that explains what already exists, what can be reused, what should not be touched, and which files/packages are likely owners of future MVP work under the new target structure.

Do not implement code in this task.

### Decision Checkpoint

Owner reviews the inventory and confirms which existing packages may be changed by future tasks.

### Tests And Checks

- No tests required unless import/build checks are already trivial to run.
- Verify the inventory names real files and packages.

### Acceptance Criteria

- The owner can see what the agent found before implementation begins.
- Existing reusable code and risky areas are clearly called out.

## Task 2: MVP Decision Log And Open Questions

### Scope

Create a lightweight decision log for owner-controlled MVP choices.

### Coding Agent Prompt

Create or update an MVP decision log in `summer_2026_update/`. Include sections for target structure, UI direction, Flask route design, config schema choices, ROS2 node boundaries, safety behavior, camera strategy, storage policy, and hardware-dependent decisions. Seed it with the unresolved details from `summer_2026_updates_MVP.md` and `slvrov_ros_structure.md`.

Do not decide on behalf of the owner unless a default is already confirmed in the MVP spec. Mark recommended defaults separately from owner decisions.

### Decision Checkpoint

Owner reviews the decision log format and confirms it is useful before later tasks add to it.

### Tests And Checks

- No runtime tests required.
- Verify links and file paths in the decision log are accurate.

### Acceptance Criteria

- Open questions have owners and next-step tasks.
- Recommended defaults are visibly separate from confirmed decisions.

## Task 3: API Response Shape Decision Packet

### Scope

Choose the shared success/error response model before planning routes.

### Coding Agent Prompt

Propose the API response shape for all Flask routes. Start from the MVP spec's `{ "ok": true, "data": ... }` and structured error response examples. Include field names, error-code naming, `details` conventions, `suggested_action` usage, and how validation errors should be represented.

Do not implement response helper code in this task.

### Decision Checkpoint

Owner approves the response shape before the Flask route map uses it.

### Tests And Checks

- No runtime tests required.
- Verify examples are valid JSON.

### Acceptance Criteria

- Later route planning can use one approved response convention.
- Errors are readable for amateur maintainers and operators.

## Task 4: ROS2 Web Boundary Decision Packet

### Scope

Plan web-to-ROS boundaries before route and config details harden.

### Coding Agent Prompt

Create a boundary document that explains which MVP features are Flask-only, which use fake adapters, which may call existing ROS2 services/topics, and which likely need owner-authored ROS2 nodes. Include expected inputs/outputs for `web_ros_bridge_node`, `web_safety_bridge_node` or `safety_monitor_node`, `web_preflight_bridge_node` or `preflight_test_node`, `web_developer_monitor_node` or `ros_health_monitor_node`, and optional `storage_monitor_node`.

Do not implement ROS2 nodes in this task.

### Decision Checkpoint

Owner chooses which node boundaries they want to write personally first and which can stay fake for early UI work.

### Tests And Checks

- No runtime tests required.
- Verify the document references real interfaces where they already exist.

### Acceptance Criteria

- Future tasks know where to put fake adapters.
- Owner-authored node TODOs are clear and scoped.

## Task 5: Shared Config Schema Decision Packet

### Scope

Plan JSON config files before writing helpers or routes.

### Coding Agent Prompt

Propose JSON config structures under `rov_config/` for cameras, control profiles, thruster configuration, pin mappings, UI preferences/layout presets, required ROS2 nodes/topics/services, safety thresholds, and recording/storage settings. Include examples, validation rules, ownership notes, and which Setup Mode screens can persist each config. Group runtime config into `motors/`, `actions/`, `controls/`, and `rovs/` unless the owner approves a different split.

Use the ROS2/web boundary decisions from Task 4 so required-node, topic, service, and fake-adapter config fields match the planned architecture.

Do not implement config helper code in this task.

### Decision Checkpoint

Owner approves config file names, top-level fields, and persistence ownership.

### Tests And Checks

- Validate example JSON manually or with a JSON parser if examples are written as files.

### Acceptance Criteria

- Every MVP config area has a proposed `rov_config/` file/schema.
- Persistent versus temporary config ownership is clear.

## Task 6: Flask Route Map Decision Packet

### Scope

Plan Flask routes after response shape, ROS2 boundaries, and config ownership are known.

### Coding Agent Prompt

Create a Flask route map for owner review. Include planned routes for health, mode pages, global alerts, emergency stop, control enabled/disabled state, camera config/status, setup config save/load, control mapping, thruster config, preflight tests, developer stats, ROS2 monitor status, and storage status. Assume the route modules will live under `slvrov_core_python.web` unless the owner approves a different structure.

For each route, list method, path, request body, response shape, backing service/adapter, fake adapter behavior, possible ROS2 owner-authored node dependency, config files touched, and whether it is MVP or future.

Do not implement routes in this task.

### Decision Checkpoint

Owner chooses route naming conventions and approves the first API surface.

### Tests And Checks

- No runtime tests required.
- Verify route names are consistent and use the approved structured success/error response shapes.

### Acceptance Criteria

- The route map is specific enough for implementation tasks.
- Routes that may call owner-authored ROS2 nodes are clearly marked.
- Config dependencies are visible before route implementation begins.

## Task 7: Target Structure Migration Decision Packet

### Scope

Plan the migration from the current repo layout to the target structure before moving files.

### Coding Agent Prompt

Compare the current repo layout against `summer_2026_update/slvrov_ros_structure.md`. Produce a structural migration plan that covers:

- keeping Flask/UI scaffold content in `src/slvrov_core_python/slvrov_core_python/web/`
- keeping control nodes/helpers in `src/slvrov_core_python/slvrov_core_python/control/`
- removing the science/capture scaffold from MVP scope
- removing `src/slvrov_tools_vendor/` and using a venv-installed `slvrov-tools`
- maintaining `src/slvrov_core_python/slvrov_core_python/mediamtx/`
- keeping reviewed runtime config in `rov_config/`
- keeping `slvrov_interfaces` service grouping aligned with node/feature ownership
- ensuring `slvrov_launch` launch files reference `rov_config/rovs/`
- preserving tests and package metadata during migration

Do not move files in this task.

### Decision Checkpoint

Owner approves which current scaffolds are migrated, retired, or preserved.

### Tests And Checks

- No runtime tests required.
- Verify the plan names real current files/directories and exact target locations.

### Acceptance Criteria

- The owner can see exactly what will move, what will stay, and what will be removed later.
- The plan has a safe phase order and validation steps.

## Task 7A: Apply Target Structure Skeleton

### Scope

Create the approved target directories and low-risk placeholders.

### Coding Agent Prompt

Implement the approved target structure skeleton. Create or update directories for `slvrov_core_python` web routes/templates/static/adapters, MediaMTX support files, and `rov_config/motors/`, `rov_config/actions/`, `rov_config/controls/`, and `rov_config/rovs/`. Move only owner-approved low-risk scaffold files. Do not remove old packages until a later cleanup task confirms their useful content has been migrated or intentionally discarded.

Use the approved route map, config plan, and ROS2 boundary document to name directories and placeholder files.

### Decision Checkpoint

Owner reviews the new skeleton before feature code is added.

### Tests And Checks

- Verify package manifests and setup files are syntactically valid.
- Run available package discovery/build checks if `colcon` is available.
- Add or update minimal tests that prove importable Python package skeletons import cleanly.

### Acceptance Criteria

- Target directories exist with clear ownership.
- Starter configs are valid JSON.
- Docs explain the structure for future maintainers.
- Old scaffold packages remain untouched unless the owner approved a specific move.

## Task 8: API Response Helpers

### Scope

Implement the approved API response helpers.

### Coding Agent Prompt

Implement shared Flask response helpers for predictable success and error responses, based on the approved response shape from Task 3. Include error code, message, details, and suggested action fields for failures. Add tests for response shape and common error cases.

Do not implement feature-specific routes beyond a simple health/example route if needed for tests.

### Decision Checkpoint

Owner reviews helper names and example output before feature endpoints use them everywhere.

### Tests And Checks

- Unit test success response shape.
- Unit test error response shape.
- Test JSON serialization of details.

### Acceptance Criteria

- Feature tasks can reuse one response convention.
- Errors are readable for amateur maintainers and users.

## Task 9: Shared Config Helpers

### Scope

Implement approved config helpers.

### Coding Agent Prompt

Create Python helpers for loading, validating, saving, and reporting errors for the approved MVP config files under `rov_config/`. Use simple validation functions or JSON schemas, whichever best fits the current repo style. Keep malformed files from silently falling back to defaults.

Do not build UI screens yet.

### Decision Checkpoint

Owner reviews helper names and config error behavior.

### Tests And Checks

- Unit test valid config loading.
- Unit test missing files and invalid JSON errors.
- Unit test required fields and default values.
- Verify sample JSON files parse cleanly.

### Acceptance Criteria

- Config files cover approved MVP areas.
- Config helpers produce clear errors.
- Tests document expected config behavior.

## Task 10: UI Demo Shell

### Scope

Create a disposable demo area for UI prototypes.

### Coding Agent Prompt

Create a simple static UI demo shell where multiple HTML/CSS/JS prototypes can be viewed without committing to production structure. Include a demo index page and instructions for starting/viewing demos. Place demos in the owner-approved `slvrov_core_python` web/static structure or another clearly temporary demo location selected in Task 7.

Do not implement final UI screens in this task. Do not create or modify Flask
app code, Flask routes, ROS2 code, backend adapters, API clients, package entry
points, or production frontend files. Demo pages may load local fixture JSON or
hard-coded JavaScript objects, but they must not call backend endpoints.

### Decision Checkpoint

Owner confirms the demo review workflow.

### Tests And Checks

- Verify demo index loads locally.
- Check that demo files are clearly separated from production UI.

### Acceptance Criteria

- UI demos are easy to open and compare.
- Demo shell runs as static files, with no Flask/API/ROS2 dependency.
- Unselected demos can be removed or archived later.

## Task 10A: Static UI Direction Demo

### Scope

Create a thorough static UI demo set that helps answer the open UI direction
questions in `summer_2026_update/mvp_decision_log.md` before the more specific
main page, safety, and mode demos are built.

### Coding Agent Prompt

Using the disposable demo shell from Task 10, create a static UI direction demo
with multiple style families and enough representative screens to compare real
choices. The demo should include lightweight, non-production mockups for:

- Main mode selection.
- Global emergency stop and critical alerts.
- Pilot camera layout presets.
- Setup Mode status/navigation structure.
- Developer Mode stat/dashboard density.

Apply `system_documents/ui_design` to every option. Each option should show:

- Clear affordances and visible hover/pressed/disabled/loading/error states.
- Semantic safety colors for emergency, warning, healthy, and active states.
- Stable 4-point-grid spacing and fixed camera/control dimensions.
- Readable dark-mode contrast and camera overlays.
- At least one empty/unavailable state and one critical-alert state.

Historical Task 10A exploration included these style families:

- Apple/macOS-inspired: restrained, polished, high-clarity, with subtle depth
  and native-control feeling, adapted for a browser-based ROV tool.
- Existing project style: based on references now archived under
  `ui_static/archive/` and `ui_archive/legacy_ui_examples/`.
- Hybrid style: combine the best parts of the macOS-inspired and existing
  project styles.
- Agent-recommended style: a clearly labeled additional direction the coding
  agent thinks fits the ROV audience, safety needs, and MVP goals.

Each style family showed the same core decisions so the owner could compare
layout, information density, safety-control placement, camera emphasis, and
operator readability across styles. Use fake data and placeholder camera panels.

Do not implement production UI screens, Flask routes, ROS2 behavior, or final
frontend utilities in this task. Do not make `fetch()` calls or wire any
runtime API clients; all state should come from static fixtures or in-page mock
objects.

### Decision Checkpoint

Completed: owner selected the Apple/macOS-inspired direction on 2026-06-16.
Future UI demo tasks should not reopen broad style-family exploration unless
the owner explicitly asks.

### Tests And Checks

- Verify the demo index loads locally.
- Verify demos still work with backend, ROS2, and MediaMTX offline.
- Browser/manual check at 4:3 and 16:10-ish viewport sizes.
- Verify text, controls, alert banners, and emergency stop controls do not
  overlap.
- Verify every demo is clearly labeled as non-production.
- Verify each style option has a short tradeoff note.
- Verify each option avoids decorative emojis, decorative-only animations,
  nested cards, and text/control overlap.

### Acceptance Criteria

- The owner can compare at least four visual directions against the same MVP
  screens.
- The demo directly helps answer the UI questions listed in
  `summer_2026_update/mvp_decision_log.md`.
- Recommended defaults are visibly separate from owner-selected decisions.
- Later UI demo tasks can narrow from the selected direction instead of
  exploring unrelated styles again.

### Outcome

- Selected active demo path:
  `ui_static/mvp_demo_shell/prototypes/apple_macos_direction/`.
- Unselected legacy demos/examples archived under `ui_static/archive/` and
  `ui_archive/legacy_ui_examples/`.

## Task 11: Main Page UI Demos

### Scope

Generate alternate main mode-selection UI demos.

### Coding Agent Prompt

Create at least two static main-page demos for Pilot, Setup, and Developer mode selection. Use the selected Apple/macOS direction, dark/light theme support, and large-display target. Make the demos meaningfully different in layout, information density, and visual emphasis.

Do not implement the production main page yet. Do not connect demos to Flask,
API routes, local storage, backend state, or ROS2; links may navigate only
between static demo pages.

### Decision Checkpoint

Owner selects a main page direction or requests another demo.

### Tests And Checks

- Browser/manual check at 4:3 and 16:10-ish viewport sizes.
- Verify no demo uses login or remembered-mode behavior.
- Verify there are no backend/API calls.
- Verify each demo follows `system_documents/ui_design` for affordances,
  hierarchy, semantic colors, typography, and empty/unavailable states.
- Verify notification/state colors are consistent: error red, warning
  yellow/orange, and status/active/connected/healthy light blue.
- Verify Pilot, Setup, and Developer do not use different mode colors.

### Acceptance Criteria

- The owner can compare real visual options.
- Each option has a short tradeoff note.

## Task 12: Global Safety UI Demos

### Scope

Generate alternate emergency stop and alert placement demos.

### Coding Agent Prompt

Create at least two static demos showing how global emergency stop, critical alerts, and disabled-control state could appear across mode pages. Use the selected Apple/macOS direction and dark/light theme support. Include one low-distraction option for Pilot Mode and one more status-rich option for Setup/Developer contexts.

Do not wire real API calls, fake API clients, backend state, local storage, or
ROS2 commands. Safety state should be represented with static fixtures and
in-page mock state only.

### Decision Checkpoint

Owner selects the global safety layout and alert behavior direction.

### Tests And Checks

- Browser/manual check that emergency stop remains visible at target viewports.
- Verify emergency stop interactions do not call backend endpoints.
- Verify alerts do not hide critical controls.
- Verify error, warning, status/healthy, disabled, and loading states are
  visually distinct without relying on color alone.
- Verify notification/state colors are consistent: error red, warning
  yellow/orange, and status/active/connected/healthy light blue.
- Verify modes do not use different colors.

### Acceptance Criteria

- Safety controls are visible and understandable in every demo.
- Tradeoffs between visibility and distraction are documented.

## Task 13: Setup Mode Information Architecture Decision Packet

### Scope

Plan Setup Mode workflow before Setup demos and implementation.

### Coding Agent Prompt

Propose the Setup Mode information architecture. Include camera setup, control mapping, thruster configuration, and System Test/Preflight. Provide at least two flow options, such as tabs versus guided steps, and explain tradeoffs for amateur maintainers/operators.

Do not implement Setup Mode in this task.

### Decision Checkpoint

Owner selects the Setup Mode workflow structure.

### Tests And Checks

- No runtime tests required.
- Verify the proposed flow covers all MVP Setup requirements.

### Acceptance Criteria

- Setup Mode has an owner-approved structure before screens are designed or built.
- Future setup tasks know where each sub-feature belongs.

## Task 14: Mode UI Demos

### Scope

Generate alternate UI demos for each major mode.

### Coding Agent Prompt

Create static demos for Pilot, Setup, and Developer Mode using the selected Apple/macOS direction and dark/light theme support. Provide at least three Pilot camera layout/control-density options, at least two Setup flow demos based on the approved Setup information architecture, and at least two Developer dashboard layouts. Use fake data and placeholder camera panels, and include normal, empty/unavailable, warning, and critical states where relevant.

Do not implement production mode pages yet. Do not connect to Flask, API
routes, ROS2, MediaMTX/WebRTC, config files, local storage, or backend
adapters. Camera streams should be static placeholders or local mock panels.

### Decision Checkpoint

Owner selects a UI direction for each mode and notes any pieces to combine.

### Tests And Checks

- Browser/manual check at 4:3 and 16:10-ish viewport sizes.
- Verify demos still work with backend, ROS2, and MediaMTX offline.
- Verify text and controls do not overlap.
- Verify demos are clearly labeled as non-production.
- Verify notification/state colors are consistent: error red, warning
  yellow/orange, and status/active/connected/healthy light blue.
- Verify Pilot, Setup, and Developer do not use different mode colors.
- Verify camera tiles, toolbars, tabs, stat cards, and alert banners keep stable
  dimensions across state changes.

### Acceptance Criteria

- The owner can choose visual and workflow directions mode by mode.
- Selected and rejected ideas are documented.

## Task 15: Selected Theme And Frontend Utilities

### Scope

Implement shared frontend structure after UI direction is selected.

### Coding Agent Prompt

Based on the owner-selected demos, implement shared CSS variables, layout utilities, reusable frontend helpers, and production static file organization inside `slvrov_core_python`. Keep the production UI plain HTML/CSS/JavaScript.

Remove or archive unselected demo files if the owner approves.

If the owner has not approved backend work yet, keep this task frontend-only:
do not create Flask routes, backend adapters, ROS2 code, API clients, or live
`fetch()` calls. Use a small mock data module or static fixtures as the future
replacement point for backend data.

### Decision Checkpoint

Owner reviews the shared theme before feature pages depend on it.

### Tests And Checks

- Frontend smoke check that shared styles load.
- Browser/manual check at target viewports.
- Verify no backend/API/ROS2 calls are made unless the owner explicitly moved
  beyond the backend-free UI phase.
- Verify the shared theme encodes the approved subset of
  `system_documents/ui_design`, including semantic colors, focus states,
  interaction states, spacing tokens, type scale, and responsive constraints.

### Acceptance Criteria

- Production UI has a clear shared style foundation.
- Demo artifacts do not confuse production files.

## Task 16: Flask App Shell And Static Frontend Shell

### Scope

Create the basic web app shell after route and frontend structure decisions are approved.

### Coding Agent Prompt

Build the minimal `slvrov_core_python` Flask application shell and static frontend shell. The app should serve the main page and mode pages as simple static experiences from the approved `slvrov_core_python` web structure. Add a lightweight `/api/health` endpoint that returns the approved structured JSON response shape.

Do not connect to ROS2 yet. Use a bridge-client interface or fake adapter only if it helps preserve the approved future boundary.

### Decision Checkpoint

Owner reviews the basic app structure and run instructions.

### Tests And Checks

- Unit test Flask app creation.
- Test `/api/health` response shape.
- Verify static files are served.
- Run a local dev server if practical and inspect the rendered shell.

### Acceptance Criteria

- Main page and placeholder mode pages load.
- `/api/health` returns the approved success shape.
- Frontend structure is split clearly enough for future mode-specific code.

## Task 17: Selected Main Mode-Selection Page

### Scope

Implement the selected main page.

### Coding Agent Prompt

Implement the owner-selected main mode-selection page. It should show three large navigation cards or buttons with short descriptions. It should not require login and should not remember the last selected mode.

Do not implement mode internals in this task.

### Decision Checkpoint

Owner reviews the production main page before mode internals are added.

### Tests And Checks

- Frontend smoke test that all three navigation targets are present.
- Manual or browser-automation check at 4:3 and 16:10-ish viewport sizes.
- Verify no local storage/session behavior remembers the last selected mode.

### Acceptance Criteria

- Three mode entry points are visible and easy to select.
- Descriptions are concise and understandable.
- Page uses the selected shared theme.

## Task 18: Global Alerts And Emergency Stop API Contract

### Scope

Implement backend state and fake adapter for global safety.

### Coding Agent Prompt

Implement backend API endpoints for reading alert state, triggering emergency stop, clearing emergency stop if safe, and reading/updating the simple `control enabled / disabled` state. Use fake ROS2/safety adapters and mark the future owner-authored safety node boundary clearly.

Do not wire real motor shutdown unless an existing safe service already exists and the owner explicitly approves.

### Decision Checkpoint

Owner reviews safety route behavior and the fake-to-real adapter boundary.

### Tests And Checks

- Unit test API response shapes.
- Test emergency stop state transitions.
- Test disabled control state.
- Test fake safety adapter calls.

### Acceptance Criteria

- API uses structured success/error responses.
- Real motor-control integration points are clearly marked.
- Safety behavior fails closed where practical.

## Task 19: Global Alerts And Emergency Stop UI

### Scope

Implement selected global safety UI.

### Coding Agent Prompt

Implement the owner-selected global emergency stop and critical-alert UI. Emergency stop must be visible across all mode pages and must call the safety API shell from Task 18. Show critical alerts and disabled-control state clearly.

Do not wire real motor shutdown in frontend code.

### Decision Checkpoint

Owner reviews safety UI placement before camera and mode pages add more complexity.

### Tests And Checks

- UI test that emergency stop appears on all mode pages.
- Test that emergency stop state can be triggered and reflected in the UI.
- Browser/manual check that alerts do not block navigation or camera viewing.

### Acceptance Criteria

- Emergency stop is globally visible.
- Critical alerts can be displayed globally.
- Disabled control state is obvious.

## Task 20: Camera Config And Status API

### Scope

Implement camera config/status backend.

### Coding Agent Prompt

Implement camera configuration loading and backend endpoints to fetch camera config/status. Use the approved `rov_config/` camera configuration source for enabled cameras, display names, device paths or indexes, and MediaMTX/WebRTC paths. Keep status Flask/MediaMTX based for now unless an existing ROS2 camera status source exists.

If a ROS2 camera-status bridge seems needed, document the boundary and leave node internals for the owner.

### Decision Checkpoint

Owner reviews camera naming, MediaMTX path convention, and status fields.

### Tests And Checks

- Unit test camera config loading and validation.
- API test for camera config/status.
- Document how to point a camera entry at a MediaMTX stream.

### Acceptance Criteria

- Up to 6 camera entries can be represented.
- Disabled cameras are not reported as active previews.
- Missing stream/config errors produce clear frontend-facing messages.

## Task 21: WebRTC Single-Camera Prototype

### Scope

Prototype one configured camera preview.

### Coding Agent Prompt

Implement a frontend preview component that can display one configured MediaMTX/WebRTC stream using the camera API from Task 20. Use clear placeholder/error states when a stream is unavailable.

Do not implement recording or multi-camera layouts yet.

### Decision Checkpoint

Owner reviews the camera embed approach before Pilot and Setup preview pages depend on it.

### Tests And Checks

- Frontend smoke test that a configured camera appears in preview UI.
- Browser/manual check with fake or documented MediaMTX stream URL.
- Verify stream errors are visible and understandable.

### Acceptance Criteria

- One camera preview can be selected and displayed.
- The component can be reused by Pilot and Setup pages.

## Task 22: Pilot Mode Selected Camera Layouts

### Scope

Implement selected Pilot Mode camera layouts.

### Coding Agent Prompt

Implement the owner-selected Pilot Mode camera layouts using the camera configuration and WebRTC preview component. Support selected single-camera view and fixed/preset multi-camera layouts for up to 6 cameras. Keep extra information minimal.

Do not add joystick visualization, telemetry, or web joystick controls.

### Decision Checkpoint

Owner reviews Pilot Mode before Setup camera-preview wiring depends on the same component.

### Tests And Checks

- Frontend tests for layout switching.
- Browser/manual checks at large 4:3 and 16:10 viewport sizes.
- Verify emergency stop is still visible in all layouts.
- Verify disabled cameras are not rendered as active streams.

### Acceptance Criteria

- Pilot Mode has clear single and multi-camera layouts.
- Layouts are fixed/preset only.
- No non-MVP controls are introduced.

## Task 23: Storage Policy Decision Packet

### Scope

Plan runtime log/storage policy before storage warnings are implemented.

### Coding Agent Prompt

Propose the storage policy for runtime data directories. Include warning threshold, stop threshold, log/test-output capacity expectations, global alert behavior, and whether a future owner-authored `storage_monitor_node` is useful.

Do not implement storage code in this task.

### Decision Checkpoint

Owner approves thresholds, messages, and stop behavior before storage warnings are implemented.

### Tests And Checks

- No runtime tests required.
- Verify proposed policy supports roughly 60 minutes of data per type where practical.

### Acceptance Criteria

- Storage limits are understandable before implementation.
- Shutdown behavior is clear and testable.

## Task 24: Deferred Science Capture Route Decision Packet

### Scope

Future-only planning for science/media capture routes.

### Coding Agent Prompt

Do not run this task for the MVP. If the owner later reopens science/capture scope, propose the backend route design and adapter strategy for Scientist Mode media capture. Cover taking photos, starting/stopping per-camera video recording, reporting recording state, safe filename overrides, recent photos, storage checks, and hardware/MediaMTX assumptions.

Compare Flask/MediaMTX-only capture with a future owner-authored `media_capture_node` or `science_capture_node`. Use the approved storage policy when planning recording stop behavior.

Do not implement capture routes in this task.

### Decision Checkpoint

Owner decides whether science/capture returns to scope, then chooses capture route names and whether capture should stay Flask-side or use a ROS2 node boundary.

### Tests And Checks

- No runtime tests required.
- Verify planned response shapes match the shared API convention.

### Acceptance Criteria

- Capture behavior is planned before code exists.
- Hardware-dependent gaps are called out.

## Task 25: Deferred Scientist Capture Backend

### Scope

Future-only implementation of selected capture backend behavior.

### Coding Agent Prompt

Do not run this task for the MVP. If the owner later reopens science/capture scope, implement backend endpoints for taking photos, starting/stopping per-camera recording, reporting recording state, and listing recent photos. Use timestamped filenames by default and allow optional filename overrides with safe filename validation. Use fake camera/media adapters where hardware is unavailable. Implement this inside the owner-approved `slvrov_core_python` web/media structure unless a future structural decision creates a separate science package.

If capture commands need ROS2 ownership, define an adapter and fake implementation for the future owner-authored node instead of implementing node internals.

### Decision Checkpoint

Owner reviews backend capture behavior before final future Scientist UI wiring.

### Tests And Checks

- Unit test filename generation and override validation.
- API tests for photo and recording endpoints.
- Test recent photo listing.
- Test fake camera/media adapters.

### Acceptance Criteria

- Capture endpoints have predictable structured responses.
- Media filenames are safe and timestamped by default.
- Hardware-dependent gaps are documented.

## Task 26: Deferred Scientist Mode UI

### Scope

Future-only implementation of selected Scientist Mode UI.

### Coding Agent Prompt

Do not run this task for the MVP. If the owner later reopens science/capture scope, implement the owner-selected Scientist Mode camera/capture UI. Include camera viewing, photo capture, video recording controls, recording state, optional filename override, and a simple recent photos gallery.

Do not implement sensor data logging yet.

### Decision Checkpoint

Owner reviews Scientist Mode before future capture storage limits are integrated deeply.

### Tests And Checks

- UI smoke tests for capture buttons and recent gallery rendering.
- API integration tests with fake capture backend.
- Browser/manual check at target viewports.

### Acceptance Criteria

- Scientist Mode is camera/capture focused.
- Recent photos gallery can render saved photo metadata.
- Recording state is clear.

## Task 27: Setup Camera Configuration UI

### Scope

Implement camera setup controls.

### Coding Agent Prompt

Implement the Setup Mode camera configuration UI using the selected Setup structure. Allow enable/disable camera, display name, camera index/device path, stream URL or MediaMTX path, and test preview. Save changes through approved `rov_config/` config helpers.

Do not implement camera auto-detection unless the owner explicitly approves it.

### Decision Checkpoint

Owner reviews camera setup behavior and config persistence.

### Tests And Checks

- Unit/API tests for camera config save/load.
- UI smoke test for editing and saving a camera.
- Verify disabled cameras stop appearing as active previews.

### Acceptance Criteria

- Camera configuration can be edited and saved.
- Test preview helps validate a stream path.

## Task 28: Control Mapping Route And Service Decision Packet

### Scope

Plan control mapping backend and ROS2 service usage before UI/API implementation.

### Coding Agent Prompt

Propose the route/service flow for physical joystick control mapping. Cover Linux `js#` device display, live interpreted input values, axes/buttons mapping, profile save/load, existing joystick mapper/service concepts, and fake joystick data for UI testing.

Do not rewrite existing joystick mapper node behavior and do not implement routes in this task.

### Decision Checkpoint

Owner chooses how much of the control mapping flow should call existing services versus fake adapters until owner-written ROS2 code is ready.

### Tests And Checks

- No runtime tests required.
- Verify proposed action types use `ROVActionType` string values.

### Acceptance Criteria

- Control mapping has a clear owner-approved backend plan.
- Existing joystick mapper ownership is respected.

## Task 29: Setup Control Mapping UI And API

### Scope

Implement control mapping UI/API with approved adapters.

### Coding Agent Prompt

Implement Setup Mode control mapping for physical joystick input. The UI should let users see live interpreted input values, map axes and buttons, and save multiple control profiles to JSON. Use existing joystick mapper/service concepts where practical and fake adapters where hardware or owner-written ROS2 logic is unavailable.

Do not implement web joystick/gamepad controls.

### Decision Checkpoint

Owner reviews the mapping workflow before thruster configuration is added.

### Tests And Checks

- Unit test control profile JSON validation.
- Service/API tests for mapping state and profile save/load.
- Fake joystick-message tests for interpreted input values.
- UI smoke test for mapping workflow steps.

### Acceptance Criteria

- Users can create and save named control profiles.
- Axis/button action types use the `ROVActionType` string values.
- Live interpreted input display is clear enough for setup.

## Task 30: Thruster Config Schema And Safety Decision Packet

### Scope

Plan thruster config fields and safety constraints.

### Coding Agent Prompt

Propose the thruster configuration schema and validation rules. Include inversion, deadzone, startup power, max power, neutral/min/max PWM, trim, scale/gain, ramp rate, test pulse duration, test pulse power, physical location/name, and channel/pin mapping. Include safety constraints and any generated launch/config file strategy.

Do not implement config UI or motor-control node internals.

### Decision Checkpoint

Owner approves thruster fields, ranges, generated file behavior, and safety defaults.

### Tests And Checks

- Validate example JSON if written.
- Verify all MVP thruster parameters are represented.

### Acceptance Criteria

- Thruster config decisions are explicit before implementation.
- Motor-control ownership boundary is clear.

## Task 31: Setup Thruster Configuration UI And Helpers

### Scope

Implement approved thruster config behavior.

### Coding Agent Prompt

Implement Setup Mode thruster configuration. Store thruster configuration and motor controller channel/pin mappings in `rov_config/motors/` and connect complete ROV selections through `rov_config/rovs/` if approved. Implement approved validation and generated launch/config helpers if selected. Avoid hidden magic.

Do not implement motor test execution or motor-control ROS2 node internals.

### Decision Checkpoint

Owner reviews thruster config UI and generated output before preflight motor tests are planned.

### Tests And Checks

- Unit test thruster config validation.
- Unit test pin/channel mapping validation.
- Test config save/load round trips.
- Test generated config/launch output if implemented.

### Acceptance Criteria

- Every approved MVP tuning parameter is represented.
- Invalid PWM/range values fail with clear errors.
- Config is saved as readable JSON.

## Task 32: Preflight Test Plan And Route Decision Packet

### Scope

Plan System Test/Preflight after safety, camera, control mapping, and thruster config behavior are known.

### Coding Agent Prompt

Propose the preflight route design, UI actions, result log format, safety behavior, fake ROS2 adapters, and owner-authored node boundaries. Cover camera test, motor/thruster test shell, controller test, ROS2 node/topic/service test, cancel/stop behavior, and why there is no single "Run All Tests" button.

Do not implement preflight code in this task.

### Decision Checkpoint

Owner approves test names, route names, safety limits, and log format.

### Tests And Checks

- No runtime tests required.
- Verify each MVP preflight test has a proposed start/cancel/status/result flow.

### Acceptance Criteria

- Preflight behavior is owner-approved before safety-sensitive code is written.
- Owner-authored `preflight_test_node` or `thruster_test_node` TODOs are clear.

## Task 33: Preflight UI And API With Fakes

### Scope

Implement preflight UI/API using fake adapters.

### Coding Agent Prompt

Implement the System Test/Preflight section inside Setup Mode. Add individual test controls for camera checks, motor/thruster test sequences, controller input checks, and ROS2 node/topic/service checks. Save test results to a log file. Use fake ROS2 test adapters where owner-authored node logic is not ready.

Use safe test mode behavior: disable normal driving commands during an active test, allow only the active test command, keep motor tests low-power and timed, provide cancel/stop, and ensure emergency stop overrides tests.

Do not add a single "Run All Tests" button.

### Decision Checkpoint

Owner reviews preflight flow before any real hardware test integration.

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

## Task 34: Developer Stats Route Decision Packet

### Scope

Plan Developer Mode system stats.

### Coding Agent Prompt

Propose the Developer Mode system stats API and UI data model. Cover CPU usage, memory, disk, disk remaining, CPU temperature if available, network throughput/status if practical, camera recording/storage usage, and process status for key services. Include polling frequency and platform fallback recommendations.

Do not implement stats collection in this task.

### Decision Checkpoint

Owner selects which stat boxes are MVP and which are future.

### Tests And Checks

- No runtime tests required.
- Verify proposed stats use low-CPU collection approaches.

### Acceptance Criteria

- Stats are scoped before implementation.
- Platform-specific gaps are clear.

## Task 35: Developer System Stats Backend And UI

### Scope

Implement selected Developer Mode stats.

### Coding Agent Prompt

Implement Developer Mode system stats with a customizable set of stat boxes. Prefer `psutil` or similarly low-CPU Python APIs over repeatedly spawning subprocesses. Let selected stats appear in their own boxes and keep the UI technical but readable.

Do not implement ROS2 node health monitoring in this task.

### Decision Checkpoint

Owner reviews stat boxes before ROS2 monitor UI is added.

### Tests And Checks

- Unit test stat collector with mocked system data.
- API test for stats response shape.
- Frontend test for selecting visible stat boxes.
- Confirm polling interval is not too aggressive.

### Acceptance Criteria

- Stats are returned in a predictable JSON shape.
- UI allows choosing displayed stats.
- Missing platform-specific stats degrade gracefully.

## Task 36: ROS2 Monitor Contract Decision Packet

### Scope

Plan Developer Mode ROS2 monitoring after the ROS2 boundary and route map are known.

### Coding Agent Prompt

Propose the ROS2 monitor contract for Developer Mode. Cover required node visibility, recent topic publishing, required service availability, optional heartbeat/diagnostic freshness, allowlisted topics/services/parameters, dangerous action confirmations, and fake ROS2 graph data.

Leave meaningful internals of any `web_developer_monitor_node` or `ros_health_monitor_node` for the project owner unless explicitly asked to write them.

### Decision Checkpoint

Owner approves required-node list, allowlists, health thresholds, and owner-authored monitor node TODOs.

### Tests And Checks

- No runtime tests required.
- Verify arbitrary publishing is excluded from MVP.

### Acceptance Criteria

- ROS2 monitor behavior is scoped before implementation.
- Health state rules are clear: healthy/warning/error.

## Task 37: Developer ROS2 Monitor UI/API With Fakes

### Scope

Implement ROS2 monitor display and fake backend.

### Coding Agent Prompt

Implement Developer Mode ROS2 monitoring using the approved contract, fake ROS2 graph/service/topic data, and clear owner-authored node TODOs. Report required node visibility, recent topic publishing, required service availability, and optional heartbeat/diagnostic freshness. Use allowlists for topics, services, and parameters.

Do not add arbitrary ROS2 publish tools in MVP.

### Decision Checkpoint

Owner reviews monitor display before any real ROS2 graph integration.

### Tests And Checks

- Unit test health classification rules.
- Fake ROS2 graph/service/topic tests.
- API test for monitor status response.
- UI smoke test for healthy/warning/error display.

### Acceptance Criteria

- Health state is clear and simple.
- Only allowlisted items are exposed.
- Dangerous actions are not available by default.

## Task 38: Storage Limit Handling And Alert Integration

### Scope

Implement approved storage handling for runtime logs and test output.

### Coding Agent Prompt

Implement MVP storage limit handling for runtime data directories. Track disk remaining and estimate remaining log/test-output capacity using the approved policy. Show warnings when warning thresholds are crossed and cleanly stop optional logging behavior when stop thresholds are reached.

Storage checks can be Flask-side for MVP. If storage warnings need to feed ROS2/global alerts, define the contract for a future owner-authored `storage_monitor_node` and test it with a fake adapter.

### Decision Checkpoint

Owner reviews storage alerts and recording stop behavior.

### Tests And Checks

- Unit test storage threshold classification.
- Unit test remaining media estimate formatting.
- API test for storage status.
- Test recording stop hook with a fake recorder.

### Acceptance Criteria

- Warnings are clear and user-facing.
- Stop threshold cleanly stops active recordings.
- Storage config is saved/read from JSON.

## Task 38A: Structural Cleanup Audit

### Scope

Audit old scaffolds after target-structure migration and feature implementation have stabilized.

### Coding Agent Prompt

Compare any future recreated standalone web/science/config paths and root package-specific docs against the migrated `slvrov_core_python` and `rov_config/` structure. Identify which files have been migrated, which are still useful, which are duplicate/stale, and which can be safely removed after owner approval.

Do not delete files in this task unless the owner explicitly approves the exact removals.

### Decision Checkpoint

Owner approves which old scaffold files/packages are removed, preserved, or deferred.

### Tests And Checks

- Run import/build checks before and after any approved cleanup.
- Verify package manifests and launch files do not reference removed packages.
- Verify docs no longer point to removed paths.

### Acceptance Criteria

- No useful scaffold content is lost.
- Old package/config paths are either intentionally preserved or ready for removal.
- Cleanup risk is documented before destructive changes.

## Task 39: Final Integration Route Audit

### Scope

Audit implemented routes against the route map.

### Coding Agent Prompt

Compare all implemented Flask routes against the approved route map. Update the route map with final paths, methods, request/response shapes, fake adapters, and owner-authored ROS2 node TODOs. Identify route drift, missing tests, inconsistent error shapes, and feature gaps.

Do not add large new features in this task.

### Decision Checkpoint

Owner decides whether remaining route gaps are MVP blockers or follow-up work.

### Tests And Checks

- Run API tests if available.
- Verify route docs match actual code.

### Acceptance Criteria

- Route documentation and implementation agree.
- Remaining route gaps are visible.

## Task 40: End-To-End Smoke Tests

### Scope

Add final MVP verification tests.

### Coding Agent Prompt

Add automated end-to-end smoke tests where practical. Tests should verify the app can start, the main page loads, each mode route loads, health/status endpoints respond, safety UI is visible, fake adapters work, and hardware-dependent behavior has clear fake/manual notes.

Do not require real hardware for automated tests.

### Decision Checkpoint

Owner reviews what is covered by automation versus manual/hardware checks.

### Tests And Checks

- Run all available unit and integration tests.
- Run browser smoke tests if the frontend can be served locally.
- Verify tests do not require physical cameras, joysticks, or thrusters.

### Acceptance Criteria

- Tests cover the major API/UI paths.
- Hardware-dependent behavior is isolated behind fakes or manual checklist items.

## Task 41: MVP Documentation And Manual Checklists

### Scope

Create final setup/run docs and manual checklists.

### Coding Agent Prompt

Create the MVP documentation package. Add setup/run instructions for the `slvrov_core_python` Flask/UI pieces, ROS2 pieces, `rov_config/` files, and important API endpoints, and add a mode-by-mode manual checklist for Pilot, Setup, and Developer Mode. Document every owner-authored ROS2 node TODO created by earlier tasks, including expected interface, fake used in tests, and manual/hardware checks needed when the owner fills in the node logic.

### Decision Checkpoint

Owner reviews docs and chooses what is ready for implementation, merge, or follow-up.

### Tests And Checks

- Verify docs match actual commands and file paths.
- Confirm hardware-dependent behavior has clear fake/manual test notes.
- Link to the decision log and route map.

### Acceptance Criteria

- A maintainer can follow docs to run the MVP shell.
- Manual checklist identifies what requires hardware.
- Owner-authored ROS2 node work is easy to find and continue.
