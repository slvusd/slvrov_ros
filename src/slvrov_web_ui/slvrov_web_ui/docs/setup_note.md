# `slvrov_web_ui` MVP Setup Note

This package will own the Flask backend, browser-facing static files, and
small adapter interfaces that let Flask routes talk to owner-authored ROS2
bridge or monitor nodes.

Task 1 intentionally does not start a real Flask app or connect to ROS2.
Future MVP tasks should add behavior in small layers:

- `routes/`: Flask blueprints and route helpers.
- `static/`: plain HTML/CSS/JavaScript assets served to browsers.
- `templates/`: shared Flask templates if later tasks need server-rendered
  pages.
- `ros_adapters/`: simple Python interfaces or fake clients used by route
  tests before real ROS2 bridge nodes are ready.
- `nodes/`: placeholder location and docs for project-owner-authored ROS2
  bridge, safety, preflight, and monitor nodes.

Keep route handlers small. When a route needs ROS2 information, prefer a
clear adapter or fake object that can be tested without running ROS2.

