#!/usr/bin/env python3
"""joy_calibrator.py

Interactive joystick calibration tool for the SLVROV.

Requirements satisfied
──────────────────────
  ✔ Independent of number of joysticks  — auto-detects all connected sticks
  ✔ Asks user for each axis/button      — move the physical control to bind it
  ✔ Axes can be ignored/skipped         — press Enter or type 's' to skip
  ✔ Saves to YAML                       — writes joy_config.yaml on completion
  ✔ AI config review                    — calls Claude API to explain, flag
                                          issues, and suggest tweaks

Dependencies
────────────
    pip install pygame pyyaml anthropic

Run
───
    python3 joy_calibrator.py
    python3 joy_calibrator.py --out my_config.yaml   # custom output path
    python3 joy_calibrator.py --no-ai                # skip the AI review step
"""

from __future__ import annotations

import argparse
import select
import sys
import time
from dataclasses import dataclass, asdict, field
from pathlib import Path
from typing import List, Optional

import pygame
import yaml


# ── Config ────────────────────────────────────────────────────────────────────

DEFAULT_OUTPUT = "joy_config.yaml"
DEADZONE       = 0.1    # applied to every axis mapping
SCALE          = 1.0
AXIS_THRESHOLD = 0.50   # minimum deflection to register a bind
SETTLE_SECS    = 0.6    # seconds to hold still after a bind before confirming
POLL_HZ        = 60


# ── Actions list — edit order or add new ones here ────────────────────────────
# Format: (action_name, expected_source, prompt_text)

ACTIONS = [
    ("forward",     "axis",   "Push your FORWARD control forward"),
    ("strafe",      "axis",   "Push your STRAFE control right"),
    ("yaw",         "axis",   "Twist or push your YAW control"),
    ("heave",       "axis",   "Push your HEAVE (up/down) control up"),
    ("roll",        "axis",   "Push your ROLL control"),
    ("claw_rotate", "axis",   "Move your CLAW ROTATE control"),
    ("claw_tilt",   "axis",   "Move your CLAW TILT control"),
    ("claw_open",   "button", "Press your CLAW OPEN button"),
    ("claw_close",  "button", "Press your CLAW CLOSE button"),
]


# ── Dataclasses ───────────────────────────────────────────────────────────────

@dataclass
class JoystickMapping:
    action:   str
    topic:    str
    source:   str       # "axis" or "button"
    index:    int
    invert:   bool  = False
    deadzone: float = DEADZONE
    scale:    float = SCALE


@dataclass
class JoystickHardware:
    id:    int
    role:  str
    topic: str


@dataclass
class CalibrationResult:
    num_joysticks: int
    joysticks:     List[JoystickHardware] = field(default_factory=list)
    axes:          List[JoystickMapping]  = field(default_factory=list)
    buttons:       List[JoystickMapping]  = field(default_factory=list)


# ── Helpers ───────────────────────────────────────────────────────────────────

def _banner(text: str) -> None:
    print(f"\n{'─' * 60}")
    print(f"  {text}")
    print(f"{'─' * 60}")


def _topic_for(js_id: int) -> str:
    return f"/joy_{js_id}"


def _user_typed_skip() -> bool:
    """Return True if the user typed Enter or 's' on stdin (non-blocking)."""
    ready, _, _ = select.select([sys.stdin], [], [], 0)
    if ready:
        line = sys.stdin.readline().strip().lower()
        return line in ("", "s", "skip")
    return False


def _detect_axis(sticks: List[pygame.joystick.Joystick]) -> Optional[tuple]:
    """Return (js_id, axis_index, value) if any axis exceeds threshold."""
    pygame.event.pump()
    for js in sticks:
        for i in range(js.get_numaxes()):
            val = js.get_axis(i)
            if abs(val) >= AXIS_THRESHOLD:
                return (js.get_id(), i, val)
    return None


def _detect_button(sticks: List[pygame.joystick.Joystick]) -> Optional[tuple]:
    """Return (js_id, button_index) if any button is currently pressed."""
    pygame.event.pump()
    for js in sticks:
        for i in range(js.get_numbuttons()):
            if js.get_button(i):
                return (js.get_id(), i)
    return None


def _wait_release_all(sticks: List[pygame.joystick.Joystick]) -> None:
    """Block until all sticks are at rest and all buttons released."""
    while True:
        pygame.event.pump()
        active = any(
            abs(js.get_axis(i)) >= AXIS_THRESHOLD
            for js in sticks for i in range(js.get_numaxes())
        ) or any(
            js.get_button(i)
            for js in sticks for i in range(js.get_numbuttons())
        )
        if not active:
            break
        time.sleep(1 / POLL_HZ)


# ── Calibration loop ──────────────────────────────────────────────────────────

def calibrate() -> CalibrationResult:
    pygame.init()
    pygame.joystick.init()

    count = pygame.joystick.get_count()
    if count == 0:
        print("\n[ERROR] No joysticks detected. Plug one in and retry.\n")
        sys.exit(1)

    sticks: List[pygame.joystick.Joystick] = []
    for i in range(count):
        js = pygame.joystick.Joystick(i)
        js.init()
        sticks.append(js)

    _banner(f"Detected {count} joystick(s)")
    hardware: List[JoystickHardware] = []
    for js in sticks:
        role = "main" if js.get_id() == 0 else "aux"
        hw = JoystickHardware(id=js.get_id(), role=role, topic=_topic_for(js.get_id()))
        hardware.append(hw)
        print(f"  [{js.get_id()}] {js.get_name()}  →  topic={hw.topic}  role={hw.role}")

    result = CalibrationResult(num_joysticks=count, joysticks=hardware)

    _banner("Calibration — follow the prompts")
    print("  • Move / press the indicated control to bind it.")
    print("  • Press Enter (or type 's' + Enter) to SKIP an action.")
    print()

    for action, expected_source, instruction in ACTIONS:

        print(f"\n  ┌─ ACTION : {action.upper()}")
        print(f"  │  {instruction}")
        print(f"  │  (Enter / s = skip this action)")
        print(f"  └─ Waiting...", end="", flush=True)

        time.sleep(0.4)             # give the user time to read
        _wait_release_all(sticks)   # make sure nothing is still held

        detected    = None
        deadline    = None

        while True:
            if _user_typed_skip():
                print(f"\r  └─ SKIPPED {action}                              ")
                detected = None
                break

            if expected_source == "axis":
                hit = _detect_axis(sticks)

                if hit:
                    js_id, ax_idx, val = hit
                    if detected is None:
                        # First time we see this candidate — start settle timer
                        detected = hit
                        deadline = time.monotonic() + SETTLE_SECS
                        print(
                            f"\r  └─ Candidate: joy_{js_id} axis {ax_idx} "
                            f"val={val:+.2f}  hold still...",
                            end="", flush=True,
                        )
                    if time.monotonic() >= deadline:
                        print(
                            f"\r  └─ BOUND ✓  joy_{js_id} axis {ax_idx} "
                            f"val={val:+.2f}            "
                        )
                        break

                else:
                    # Stick released before settling — reset and wait again
                    if detected is not None:
                        detected = None
                        deadline = None
                        print(
                            f"\r  └─ Released too early — try again...        ",
                            end="", flush=True,
                        )

            elif expected_source == "button":
                hit = _detect_button(sticks)
                if hit:
                    js_id, btn_idx = hit
                    detected = hit
                    print(f"\r  └─ BOUND ✓  joy_{js_id} button {btn_idx}             ")
                    time.sleep(SETTLE_SECS)
                    break

            time.sleep(1 / POLL_HZ)

        if detected is None:
            continue  # skipped — do not add a mapping

        if expected_source == "axis":
            js_id, ax_idx, val = detected
            result.axes.append(JoystickMapping(
                action   = action,
                topic    = _topic_for(js_id),
                source   = "axis",
                index    = ax_idx,
                invert   = val < 0.0,   # auto-detect direction
                deadzone = DEADZONE,
                scale    = SCALE,
            ))

        elif expected_source == "button":
            js_id, btn_idx = detected
            result.buttons.append(JoystickMapping(
                action   = action,
                topic    = _topic_for(js_id),
                source   = "button",
                index    = btn_idx,
                deadzone = DEADZONE,
                scale    = SCALE,
            ))

    pygame.quit()
    return result


# ── YAML writer ───────────────────────────────────────────────────────────────

def save_yaml(result: CalibrationResult, path: str | Path) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    data = {
        "num_joysticks": result.num_joysticks,
        "joysticks": [asdict(js) for js in result.joysticks],
        "defaults": {"deadzone": DEADZONE, "scale": SCALE},
        "axes":    [asdict(m) for m in result.axes],
        "buttons": [asdict(m) for m in result.buttons],
    }

    with path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False)

    print(f"\n  ✔  Config saved → {path.resolve()}")


# ── AI review ─────────────────────────────────────────────────────────────────

def ai_review(yaml_path: str | Path) -> None:
    """Read the saved YAML, send it to Claude, print and save the analysis."""
    try:
        import anthropic
    except ImportError:
        print("\n[AI Review] Install the package first:  pip install anthropic")
        return

    yaml_text = Path(yaml_path).read_text(encoding="utf-8")

    prompt = f"""
You are a robotics software engineer reviewing a joystick calibration config for
an underwater ROV (remotely operated vehicle) called SLVROV.

The config is consumed by a ROS 2 node (multi_joy_logic.py) that:
  1. Subscribes to /joy_<id> topics (one per joystick).
  2. Loads each JoystickMapping from this YAML.
  3. Applies deadzone rescaling and scale multiplication to axis values.
  4. Flips the sign when `invert: true`.
  5. Routes the result to one of: forward, strafe, yaw, heave, roll,
     claw_open, claw_close, claw_rotate, claw_tilt.
  6. Feeds those values into a 6-thruster mixing matrix.

Here is the calibration config that was just generated:

```yaml
{yaml_text}
```

Please do ALL four of the following:

1. EXPLAIN — what each mapping does in plain English (one line per mapping).

2. FLAG ISSUES — missing critical actions (forward/strafe/yaw/heave are
   essential), duplicate axis indices on the same joystick, suspicious invert
   values, or anything else that looks wrong.

3. SUGGEST ADJUSTMENTS — deadzone or scale tweaks, axes that are conventionally
   inverted for ROV piloting, or any other improvements.

4. HOW TO TEST — step-by-step ROS 2 CLI commands (ros2 topic echo, ros2 run,
   etc.) to verify each axis before putting the ROV in the water.

Be specific and concise.
"""

    _banner("AI Config Review  (Claude)")
    print("  Sending config to Claude for analysis...\n")

    client   = anthropic.Anthropic()   # reads ANTHROPIC_API_KEY from environment
    response = client.messages.create(
        model      = "claude-sonnet-4-20250514",
        max_tokens = 1500,
        messages   = [{"role": "user", "content": prompt}],
    )

    review_text = response.content[0].text
    print(review_text)

    review_path = Path(yaml_path).with_suffix(".review.txt")
    review_path.write_text(review_text, encoding="utf-8")
    print(f"\n  ✔  Review saved → {review_path.resolve()}")


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="SLVROV joystick calibrator")
    parser.add_argument("--out",   default=DEFAULT_OUTPUT,
                        help="Output YAML path (default: joy_config.yaml)")
    parser.add_argument("--no-ai", action="store_true",
                        help="Skip the Claude AI review step")
    args = parser.parse_args()

    _banner("SLVROV Joystick Calibrator")

    result  = calibrate()
    total   = len(result.axes) + len(result.buttons)
    skipped = len(ACTIONS) - total

    _banner(f"Calibration complete — {total} bound, {skipped} skipped")
    save_yaml(result, args.out)

    if not args.no_ai:
        ai_review(args.out)
    else:
        print("\n  [AI Review skipped — omit --no-ai to enable it]")


if __name__ == "__main__":
    main()
