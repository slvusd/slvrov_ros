"""Central record of deferred control features.

This module intentionally keeps unimplemented feature names and planning notes
out of the active node files so the runtime code stays focused on features that
are actually live.
"""

from __future__ import annotations


DEFERRED_ACTIONS = (
    "claw_open",
    "claw_rotate",
    "claw_tilt",
)

DEFERRED_RUNTIME_FEATURES = (
    "button_fallback_arbitration",
    "text_redundancy_input",
    "claw_command_pipeline",
)

DEFERRED_MAPPING_FIELDS = (
    "skipped_actions",
    "multiple_bindings_per_action",
    "button_fallback_metadata",
)
