# Joystick Calibration

This document explains how joystick calibration works in the ROS 2 stack and
what each structure in the calibrator is responsible for.

## Purpose

The joystick calibrator exists so the ROV control system does not have to assume
that every controller uses the same axis numbers, button numbers, or even the
same number of connected joysticks.

It listens to one or more `sensor_msgs/msg/Joy` topics, asks the operator to
move the control they want for each logical action, detects which physical
axis/button moved the most, and saves that mapping to a YAML or JSON file.

The joystick logic node then reads that saved file and uses it to reconstruct
the logical control state for the ROV.

## Step-By-Step Calibration Flow

1. The `joystick_calibrator` node starts and reads its parameters.
2. It decides which logical actions should be calibrated.
   Skipped actions are removed from the calibration order.
3. It subscribes to joystick topics.
   If `joy_topics` is provided, it uses those topics.
   If `joy_topics` is empty, it discovers all active `Joy` topics.
4. It starts a background command thread for terminal commands such as
   `skip`, `undo`, and `quit`.
5. It waits until at least one joystick message has been received.
6. For the current logical action, it captures a baseline snapshot of every
   active joystick topic.
7. It prompts the operator to move the control they want for that action.
8. As new `Joy` messages arrive, it compares them with the baseline:
   axis movement is measured by absolute change from the baseline value, and
   buttons are detected on a rising edge from `0` to `1`.
9. It keeps the strongest candidate control for each physical axis/button.
10. Once movement has crossed the threshold and then gone quiet for a short
    period, it selects the strongest candidate as the binding.
11. It stores the binding as a `JoystickMapping`.
12. It waits briefly so the operator can release the previous control, then
    repeats the process for the next action.
13. When all requested actions are done, or when the operator quits, it saves
    the calibration result to disk.
14. The `joystick_logic_node` can then read that file and use the saved
    `joy_topics` and `mappings` to interpret joystick input.

## Structures

### `Action`

Purpose: Defines the list of logical control actions the vehicle understands.

Short explanation:
- It is an enum of actions such as `forward`, `strafe`, `yaw`, and claw
  controls.
- Each action also knows how to describe itself to the operator through a
  prompt string.

### `JoystickMapping`

Purpose: Stores one final calibrated binding from a physical control to a
logical action.

Short explanation:
- `action`: the logical function, such as `forward`.
- `topic`: which joystick topic produced the input.
- `source`: whether the input is an `axis` or `button`.
- `index`: the axis index or button index in the `Joy` message.
- `invert`: whether the logical direction should be flipped.
- `deadzone`: axis deadzone to apply later in the logic node.
- `scale`: multiplier used later in the logic node.

### `JoySnapshot`

Purpose: Captures the joystick state at the start of one calibration step.

Short explanation:
- It stores the baseline axis and button values before the operator moves
  anything.
- Later joystick messages are compared against this snapshot to measure motion.

### `Candidate`

Purpose: Tracks the strongest detected movement for one physical control during
the current calibration prompt.

Short explanation:
- It identifies a control by `topic`, `source`, and `index`.
- `score` measures how strong the movement was.
- `value` stores the signed axis change or button press value.
- The best candidate becomes the final mapping for the current action.

### `JoystickCalibrator`

Purpose: Orchestrates the full interactive calibration process.

Short explanation:
- It owns subscriptions to one or more joystick topics.
- It manages prompt timing, baseline capture, movement detection, and saving.
- It also manages the terminal command thread for operator commands.

## Important Internal Methods

### `_refresh_joy_subscriptions`

Purpose: Finds and subscribes to joystick topics.

Short explanation:
- Supports either explicit topic configuration or automatic discovery.
- Allows calibration to work with one or multiple joysticks.

### `_joy_callback`

Purpose: Receives live joystick messages.

Short explanation:
- Stores the latest message for each topic.
- Updates movement candidates while a calibration prompt is active.

### `_tick`

Purpose: Runs the calibration state machine.

Short explanation:
- Waits for joystick input.
- Starts prompts.
- Checks whether a candidate is strong enough and stable enough to bind.
- Finishes and saves when all actions are complete.

### `_update_candidates`

Purpose: Measures movement relative to the baseline snapshot.

Short explanation:
- For axes, it measures absolute deviation from baseline.
- For buttons, it detects a rising edge press.
- It keeps the strongest movement for each physical control.

### `_bind_candidate`

Purpose: Converts the best detected movement into a saved mapping.

Short explanation:
- It creates a `JoystickMapping`.
- It determines whether the axis should be inverted.
- It stores the binding and advances to the next action.

### `_save_output`

Purpose: Writes the calibration result to disk.

Short explanation:
- Saves `joy_topics` and `mappings`.
- Supports YAML or JSON based on file extension.
- The output file is meant to be loaded by the joystick logic node.

## Saved File Shape

The calibrator writes a simple structure like this:

```yaml
joy_topics:
  - /joy
  - /joy1
mappings:
  - action: forward
    topic: /joy
    source: axis
    index: 1
    invert: true
    deadzone: 0.1
    scale: 1.0
```

## Runtime Commands

While calibration is running, the operator can type:

- `skip`: skip the current logical action
- `undo`: remove the last saved binding
- `quit`: save current progress and exit

## Why This Design Works

- It supports different controller models without hardcoding axis numbers.
- It supports one joystick or multiple joystick topics.
- It allows optional actions such as `roll` to be skipped.
- It persists calibration so the rest of the control pipeline can stay simple.
