from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import rclpy  # type: ignore
from rclpy.executors import ExternalShutdownException  # type: ignore
from rclpy.node import Node  # type: ignore
from sensor_msgs.msg import Joy  # type: ignore
from slvrov_interfaces.srv import String
from std_srvs.srv import Trigger  # type: ignore

from ..control_objects import MappingCandidate, ROVActionMapping, ROVActionType
from ..json_crud import delete_from_json, load_from_json, save_to_json

# WILL CREATE LOGIC AFTER JSON NODES


# ACTIONS
# defaults defned in cntl objs in class (Enum? Dict? No idea?)
# stored for ROV in josn in array, with str of action ("action_name/action_type")
# user can extend actions with cutom using name and type


# AFFECT DEFINITION
# user will define how each motor obj affects each action
# stored in json as "action_name": affect (int, e.g. 1.0 <- default)
# stored in motor obj as dict

# FLOW
# 1. User creates custom actions (action_manager - TODO)
# -> custom action jsons hooked in
# -> json paths defined in ROV json param "custom_action_paths"
# * action json creation console
# * add action json button/page
#
# 2. User maps motor to action (motor_mapper - TODO)
# -> each motor has parameters and action array, defaults to 0.0 if user doesn't select
# * motor json creation console (define all parameters)
# * add motor json button/page <- IMPORTANT: can have multiple action json, but only 1 motor json
#
# 3. User maps control to action (joystick_mapper - completed)
# * control json creation console (define all parameters); map control to action
# * add control json button/page <- IMPORTANT: can have only 1 control json
#
# 4. ROV super-json is created (TODO)
# -> contains paths to all other jsons, and any global settings (e.g. global js deadzone)
# * ROV json creation console (define all parameters, including paths to other jsons)
#
# ROV JSON Creatin Console
# * add json buttons for actions, motors, and controls
# * has buttons that lead to corresponding json creation console, where json can be created
