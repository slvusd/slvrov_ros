#!/usr/bin/env python3
"""
thruster_telemetry.py
=====================
Subscribes to /thruster_command and serves the latest PWM state
over a small Flask HTTP API so a UI can poll it.

Runs two threads:
  1. Flask thread       — daemon, starts first
  2. ROS 2 spin thread  — main thread, owns the executor

Launched via ros2 run (add to setup.py console_scripts):
  'thruster_telemetry=slvrov_nodes_python.thruster_telemetry:main'

Then run with:
  ros2 run slvrov_nodes_python thruster_telemetry

Endpoints
─────────
  GET /thruster/latest      full latest snapshot  (JSON)
  GET /thruster/quick       minimal dict for fast UI polling (JSON)
  GET /thruster/history     rolling 60-second history (JSON)
  GET /thruster/health      staleness check (JSON)

Prerequisites
─────────────
  pip install flask --break-system-packages
"""

import collections
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from slvrov_interfaces.msg import PCA9685Command

try:
    from flask import Flask, jsonify
    FLASK_AVAILABLE = True
except ImportError:
    FLASK_AVAILABLE = False
    print("[thruster_telemetry] Flask not found. Install with:")
    print("  pip install flask --break-system-packages")

# ── Config ────────────────────────────────────────────────────
PORT          = 9001
HISTORY_S     = 60
SAMPLE_RATE_S = 0.1   # store a snapshot at most every 100 ms
MAX_POINTS    = int(HISTORY_S / SAMPLE_RATE_S)   # 600 points
# ─────────────────────────────────────────────────────────────


class ThrusterState:
    """Thread-safe store for the latest thruster command and rolling history."""

    def __init__(self) -> None:
        self._lock    = threading.Lock()
        self._latest  = {}               # last message as a plain dict
        self._history = collections.deque(maxlen=MAX_POINTS)
        self._last_stored_at = 0.0

    # ── write ─────────────────────────────────────────────────

    def update(self, ids: list[str], pwms: list[float]) -> None:
        """Store a new snapshot from an incoming PCA9685Command message."""
        now   = time.time()
        entry = {
            "ts":      now,
            "iso":     time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(now)),
            "outputs": dict(zip(ids, [round(float(p), 4) for p in pwms])),
        }
        with self._lock:
            self._latest = entry
            # Throttle history writes to SAMPLE_RATE_S so the deque doesn't
            # fill up when the joystick node is running at 50 Hz.
            if now - self._last_stored_at >= SAMPLE_RATE_S:
                self._history.append(entry)
                self._last_stored_at = now

    # ── read ──────────────────────────────────────────────────

    def latest(self) -> dict:
        with self._lock:
            return dict(self._latest)

    def history(self) -> list[dict]:
        with self._lock:
            return list(self._history)

    def quick(self) -> dict:
        """Return a minimal snapshot suitable for fast polling."""
        with self._lock:
            if not self._latest:
                return {"ts": None, "outputs": {}}
            return {
                "ts":      self._latest["ts"],
                "iso":     self._latest["iso"],
                "outputs": dict(self._latest.get("outputs", {})),
            }


# ── ROS node ──────────────────────────────────────────────────

class ThrusterTelemetryNode(Node):
    """Lightweight subscriber that feeds ThrusterState from /thruster_command."""

    def __init__(self, state: ThrusterState) -> None:
        super().__init__("thruster_telemetry_node")
        self._state = state
        self._sub = self.create_subscription(
            PCA9685Command,
            "thruster_command",
            self._callback,
            10,
        )
        self.get_logger().info(
            f"thruster_telemetry_node ready — subscribed to /thruster_command, "
            f"serving on :{PORT}"
        )

    def _callback(self, msg: PCA9685Command) -> None:
        self._state.update(list(msg.id), list(msg.pwm))


# ── Flask app ─────────────────────────────────────────────────

def make_flask_app(state: ThrusterState) -> "Flask":
    app = Flask(__name__)

    # Silence Flask's default request logger so ROS logs stay readable.
    import logging
    log = logging.getLogger("werkzeug")
    log.setLevel(logging.WARNING)

    @app.route("/thruster/latest")
    def route_latest():
        data = state.latest()
        if not data:
            return jsonify({"error": "No data received yet"}), 503
        return jsonify(data)

    @app.route("/thruster/quick")
    def route_quick():
        return jsonify(state.quick())

    @app.route("/thruster/history")
    def route_history():
        return jsonify(state.history())

    @app.route("/thruster/health")
    def route_health():
        data = state.latest()
        age  = time.time() - data.get("ts", 0) if data else None
        return jsonify({
            "ok":          bool(data),
            "age_sec":     round(age, 2) if age is not None else None,
            "stale":       age is None or age > 1.0,
        })

    return app


# ── Entry point ───────────────────────────────────────────────

def main(args=None) -> None:
    if not FLASK_AVAILABLE:
        return

    state = ThrusterState()

    # ── Flask in daemon thread ─────────────────────────────────
    # Flask must be a daemon so it exits automatically when the
    # main ROS thread shuts down (e.g. on Ctrl-C or ros2 node kill).
    app = make_flask_app(state)
    flask_thread = threading.Thread(
        target=lambda: app.run(host="0.0.0.0", port=PORT, threaded=True),
        daemon=True,
    )
    flask_thread.start()

    # ── ROS spin on main thread ────────────────────────────────
    # rclpy.spin() must run on the main thread when launched via
    # ros2 run so that signal handling (SIGINT / Ctrl-C) works correctly.
    node = None
    try:
        rclpy.init(args=args)
        node     = ThrusterTelemetryNode(state)
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()

    except (KeyboardInterrupt, SystemExit):
        pass

    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
