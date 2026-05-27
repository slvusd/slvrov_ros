# Web UI ROS2 Node Placeholders

This directory documents likely future node boundaries for the web UI.
The project owner expects to write or finish meaningful ROS2 node logic,
so coding-agent work should add interfaces, fakes, tests, and TODO notes
before adding real node behavior.

Likely future nodes include:

- `web_ros_bridge_node`: allowlisted service/topic bridge for Flask routes.
- `web_safety_bridge_node` or `safety_monitor_node`: emergency stop,
  control-enabled state, disconnect handling, and critical faults.
- `web_preflight_bridge_node` or `preflight_test_node`: setup/preflight
  test orchestration, progress, cancel, and result reporting.
- `web_developer_monitor_node` or `ros_health_monitor_node`: node/topic/
  service freshness and allowlisted diagnostics for Developer Mode.

