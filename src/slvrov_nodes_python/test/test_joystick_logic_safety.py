from __future__ import annotations

import pytest

rclpy = pytest.importorskip("rclpy")

from slvrov_nodes_python.multi_joy_logic import (  # noqa: E402
    ALL_ACTIONS,
    JoystickLogicNode,
    JoystickMapping,
)


def _logic_node_shell() -> JoystickLogicNode:
    node = JoystickLogicNode.__new__(JoystickLogicNode)
    node.required_actions = {"forward", "strafe", "yaw", "heave", "roll"}
    node.optional_actions = set()
    return node


def test_detect_missing_actions_all_mapped() -> None:
    node = _logic_node_shell()
    mappings = [
        JoystickMapping(
            action=action,
            topic="/joy",
            source="axis",
            index=index,
        )
        for index, action in enumerate(ALL_ACTIONS)
    ]

    missing_required, missing_optional = node._detect_missing_actions(mappings)

    assert missing_required == set()
    assert missing_optional == set()


def test_detect_missing_actions_reports_required_and_optional() -> None:
    node = _logic_node_shell()
    mappings = [
        JoystickMapping(
            action="forward",
            topic="/joy",
            source="axis",
            index=1,
        ),
        JoystickMapping(
            action="yaw",
            topic="/joy",
            source="axis",
            index=2,
        ),
    ]

    missing_required, missing_optional = node._detect_missing_actions(mappings)

    assert missing_required == {"strafe", "heave", "roll"}
    assert missing_optional == set()


def test_filter_supported_mappings_ignores_deferred_actions() -> None:
    node = _logic_node_shell()

    class _Logger:
        def __init__(self) -> None:
            self.warning_messages = []

        def warning(self, message: str) -> None:
            self.warning_messages.append(message)

    logger = _Logger()
    node.get_logger = lambda: logger
    mappings = [
        JoystickMapping(
            action="forward",
            topic="/joy",
            source="axis",
            index=1,
        ),
        JoystickMapping(
            action="claw_open",
            topic="/joy",
            source="button",
            index=0,
        ),
    ]

    filtered = node._filter_supported_mappings(mappings)

    assert [mapping.action for mapping in filtered] == ["forward"]
    assert logger.warning_messages


def test_validate_mappings_still_rejects_duplicate_actions() -> None:
    node = _logic_node_shell()
    duplicate_mappings = [
        JoystickMapping(
            action="forward",
            topic="/joy_left",
            source="axis",
            index=1,
        ),
        JoystickMapping(
            action="forward",
            topic="/joy_right",
            source="axis",
            index=1,
        ),
    ]

    with pytest.raises(ValueError, match="Duplicate mapping for action: forward"):
        node._validate_mappings(duplicate_mappings)
