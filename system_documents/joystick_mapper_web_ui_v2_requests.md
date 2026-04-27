# Joystick Mapper Web UI V2 Request Contract

This document describes the browser requests expected by the version 2 static UI.

The UI files live at:

- `/Users/caleb/repositories/slvusd/slvrov_ros/src/slvrov_core_python/slvrov_core_python/ui_static/joystick_mapper_ui_v2/index.html`
- `/Users/caleb/repositories/slvusd/slvrov_ros/src/slvrov_core_python/slvrov_core_python/ui_static/joystick_mapper_ui_v2/style.css`
- `/Users/caleb/repositories/slvusd/slvrov_ros/src/slvrov_core_python/slvrov_core_python/ui_static/joystick_mapper_ui_v2/app.js`

## Layout differences from V1

V2 changes the workflow in three main ways:

1. Mapping state and mapper state are combined into a single status strip at the top.
2. The request console moves into the middle workspace where the mapping and status panels used to be.
3. The action setup area now contains an action queue with:
   - `Add Action To Queue`
   - `Map Action Now`
   - `Map Next Action`
4. `Add Action To Queue` is frontend-only and does not send a backend request.
5. `Map Action Now` now serves the role of setting the current action and starting a mapping run.

## ROS service-backed requests

These requests map directly to the existing ROS service names exposed by `joystick_mapper.py`:

### `GET /joystick_mapper/status`

Purpose:
- Populate the top status strip.

Notes:
- This is still a bridge-only helper route.
- `joystick_mapper.py` does not expose a ROS status service yet.

Suggested response:

```json
{
  "backend": "reachable",
  "active": true,
  "mapping": false,
  "saved": true,
  "message": "optional status message"
}
```

### `POST /joystick_mapper/activation`

Purpose:
- Activate or deactivate the mapper, depending on `desired_state`.

Activation browser request:

```json
{
  "desired_state": "activate",
  "topics": ["/joy", "/pilot/joy"],
  "rosServiceData": "/joy,/pilot/joy"
}
```

Deactivation browser request:

```json
{
  "desired_state": "deactivate"
}
```

ROS call:

- Service: `joystick_mapper/activation`
- Request field: `data`

Important:
- This is a toggle-style ROS service.
- The bridge should check current state so it does not toggle the mapper in the wrong direction.

## V2 queue and mapping requests

The queue itself is frontend-only in V2. The browser keeps queued actions locally and only sends requests when the user chooses to map immediately or map the next queued action.

### `POST /joystick_mapper/action_queue/map_now`

Purpose:
- Immediately begin mapping the action currently shown in the action setup form.

Browser request:

```json
{
  "action": {
    "name": "forward",
    "type": "js_axis",
    "rosServiceData": "forward/js_axis/None/None"
  },
  "desired_state": "start",
  "orchestration": ["set_action", "toggle_mapping"]
}
```

Suggested bridge behavior:

1. Call `joystick_mapper/set_action`.
2. Call `joystick_mapper/toggle_mapping` to begin mapping.
3. Return one combined JSON response to the browser.

Suggested response:

```json
{
  "success": true,
  "message": "Started mapping action forward/js_axis/None/None.",
  "mapping": true,
  "current_action": "forward/js_axis/None/None"
}
```

### `POST /joystick_mapper/action_queue/map_next`

Purpose:
- Pop the next queued action and immediately begin mapping it.

Browser request:

```json
{
  "desired_state": "start",
  "action": {
    "name": "forward",
    "type": "js_axis",
    "rosServiceData": "forward/js_axis/None/None"
  },
  "queue_length": 2,
  "orchestration": ["set_action", "toggle_mapping"]
}
```

Suggested bridge behavior:

1. Read the next queued action from bridge-owned queue state.
2. Call `joystick_mapper/set_action`.
3. Call `joystick_mapper/toggle_mapping`.
4. Remove that action from queue state after successful start.

Suggested response:

```json
{
  "success": true,
  "message": "Started mapping next queued action forward/js_axis/None/None.",
  "mapping": true,
  "current_action": "forward/js_axis/None/None",
  "queue_length": 1
}
```

## V2 expected browser flow

1. User enters joystick topics.
2. Browser calls `POST /joystick_mapper/activation`.
3. User defines an action.
4. User either:
   - clicks `Add Action To Queue`, which updates only the frontend queue
   - clicks `Map Action Now`, which calls `POST /joystick_mapper/action_queue/map_now`
5. If the user queued actions, clicking `Map Next Action` calls `POST /joystick_mapper/action_queue/map_next`.
6. The bridge is responsible for orchestrating `set_action` and `toggle_mapping` during the queue-driven flows.

## Important implementation notes

- V2 intentionally removes the explicit start/stop mapping buttons from the page.
- `Add Action To Queue` does not hit the backend.
- Because of that, the queue helper routes are expected to own the "start mapping this action" orchestration.
- `joystick_mapper/toggle_mapping` is still part of the backend contract, even though the V2 UI no longer exposes it as a direct button.
- The bridge should continue to maintain mapper state, mapping state, and saved state.
