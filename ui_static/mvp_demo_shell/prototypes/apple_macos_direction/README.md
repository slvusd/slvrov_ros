# Apple/macOS Direction Demo

This is the selected static UI direction for the MVP demos.

Owner decisions captured here:

- Use the Apple/macOS-inspired visual direction.
- Provide dark and light mode.
- Do not assign different colors to Pilot, Setup, or Developer modes.
- Reserve notification colors for meaning:
  - Error: red.
  - Warning: yellow/orange.
  - Status, active, connected, or healthy state: light blue.

All data is local mock state in `app.js`. This demo must stay static: no Flask
routes, ROS2 calls, MediaMTX/WebRTC streams, API clients, or backend `fetch()`
calls.
