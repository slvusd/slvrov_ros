# Control Flow

This document traces the current control flow from operator input to PCA9685
hardware output.

## End-To-End Flow

```mermaid
flowchart TD
    A["Operator moves joystick"] --> B["joy_node publishes Joy"]
    B --> C["joystick_logic_node caches latest Joy by topic"]
    C --> D["Load saved mapping profile"]
    D --> E["Reconstruct logical ControlState"]
    E --> F["Apply thruster mixing"]
    F --> G["Build PCA9685Command"]
    G --> H["Publish /thruster_command"]
    H --> I["thruster_bridge_node forwards"]
    I --> J["Publish /pca9685_command"]
    J --> K["pca9685_node resolves logical IDs to pin configs"]
    K --> L["Write duty cycles to PCA9685"]
```

## Safety Branches

If joystick topics go stale:

- the logic node drops stale messages
- control returns to neutral

If calibration is incomplete:

- unmapped actions stay neutral
- warnings are logged
- mapped actions continue working

## File and Logic Relationship

- calibration determines the binding file
- the logic node owns interpretation and safety handling
- the bridge only routes messages
- the PCA9685 node owns hardware translation

## Deferred Paths

Not active yet, but reserved in comments and docs:

- button fallback arbitration before mixing
- text control feeding the same `ControlState`
- expanded YAML profile metadata
- claw control reintroduction through a future contract

## Rationale

- The control path is kept linear and easy to inspect so failures are easier to
  isolate.
- Safety handling lives in the joystick logic node because that is the first
  layer that understands the meaning of actions like `forward` or `yaw`.
- Redundancy features are deferred until they can reuse this same flow instead
  of creating parallel control pipelines with different safety behavior.
