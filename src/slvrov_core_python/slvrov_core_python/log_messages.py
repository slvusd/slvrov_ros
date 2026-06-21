from enum import Enum
from typing import Union


class BaseLogMessagesFunctions:
    def __str__(self):
        return f"<{self.value}> "

    def __add__(self, other):
        return str(self) + str(other)

    def __radd__(self, other):
        return str(other) + str(self)


class BaseLogMessages(BaseLogMessagesFunctions, Enum):
    SERVER_ERROR = "server error"
    UNCAUGHT_EXCEPTION = "uncaught exception"
    TOPIC_STALE = "topic might be stale"
    NO_TOPICS = "no topics"
    SAVE_DATA = "save data"
    SAVE_SUCCESSFUL = "save data successful"
    UNSAVED_DATA = "there are unsaved changes"
    START_TIMER = "started timer"
    STOP_TIMER = "stopped timer"
    PUBLISHER_CREATED = "publisher created"
    SEND_MSG = "publisher sent message"
    SUBSCRIPTION_CREATED = "subscription created"
    SUBSCRIPTION_DESTROYED = "subscription destroyed"
    RECIEVED_MSG = "subscription recieved message"
    SERVICE_CREATED = "service created"
    SERVICE_CALL_FAILED = "service call failed"
    SERVICE_CALL_SUCCEEDED = "service call succeeded"
    SERVICE_READY = "service is available"
    RECIEVED_REQ = "service recieved request"
    RECIEVED_RESP = "client recieved response"
    CLIENT_CREATED = "client created"
    CLIENT_WAIT_ON_SERVICE = "client waiting for service to become available"
    NODE_READY = "node ready"
    NO_PATH_PROVIDED = "no path provided"
    PATH_NOT_FOUND = "path not found"
    DIRECTORY_NOT_FOUND = "directory not found"
    PERMISSION_DENIED = "permission denied"
    INVALID_JSON = "invalid json"
    JSON_ROOT_INVALID = "json root must be an object"
    DELETE_FILE = "delete file"
    RENAME_FILE = "rename file"
    LIST_DIRECTORY = "list directory"
    MISSING_FIELD = "missing field"


class JoystickMapperLogMessages(BaseLogMessagesFunctions, Enum):
    MAPPER_IS_ACTIVE = "mapper is active"
    MAPPER_IS_INACTIVE = "mapper is inactive"
    MAPPER_SET_ACTIVE = "set mapper active"
    MAPPER_SET_INACTIVE = "set mapper inactive"
    ACTION_IS_SET = "action is set"
    ACTION_ALREADY_SET = "action already set"
    ACTION_SET = "set action"
    NO_ACTION_SET = "no action is set"
    ACTION_INVALID = "action invalid"
    MAPPING_IS_ACTIVE = "mapping is active"
    MAPPING_IS_INACTIVE = "mapping is inactive"
    MAPPING_SET_ACTIVE = "set mapping active"
    MAPPING_SET_INACTIVE = "set mapping inactive"
    MAPPING_FOUND_CANDIDATE = "mapping found candidate for action"
    MAPPING_NO_CANDIDATE = "mapping does not have candidate for action"
    MAPPING_CANDIDATE_TIE = "mapping has more than one candidate for action"
    MAPPING_FAILED = "mapping failed"
    MAPPING_SUCCEEDED = "mapping succeeded"
    SAVE_MAPPED_ACTIONS = "save mapped actions"
    DELETE_MAPPING = "delete mapping"
    EDIT_MAPPING = "edit mapping"
    ADD_MAPPING = "add mapping"
    VIEW_MAPPING = "view mapping"


LogMessage = Union[BaseLogMessagesFunctions, str]


def build_error_message(operation: LogMessage, reason: LogMessage) -> str:
    """Build a standard failure message from log fragments.

    Args:
        operation (BaseLogMessagesFunctions | str): Operation that failed.
        reason (BaseLogMessagesFunctions | str): Failure reason.

    Returns:
        str: Formatted failure message.
    """

    return operation + BaseLogMessages.SERVICE_CALL_FAILED + reason


class WebCrudLogMessages(BaseLogMessagesFunctions, Enum):
    """Log message fragments for shared web CRUD helpers."""

    RESOLVE_PATH = "resolve file path"
    LOAD_JSON = "load json"
    WRITE_JSON = "write json"
    LOAD_JSON_ARRAY = "load json array"
    DELETE_JSON_OBJECT = "delete json object"
    UPDATE_JSON_OBJECT = "update json object"


class ActionCrudLogMessages(BaseLogMessagesFunctions, Enum):
    """Log message fragments for action CRUD routes."""

    ADD_ACTIONS = "add actions"
    LOAD_ACTIONS = "load actions"
    DELETE_ACTION = "delete action"
    DELETE_ACTION_FILE = "delete action file"
    MERGE_ACTION_FILES = "merge action files"
    RENAME_ACTION_FILE = "rename action file"
    UPDATE_ACTION = "update action"
    CREATE_ACTIONS = "create actions"


class MotorCrudLogMessages(BaseLogMessagesFunctions, Enum):
    """Log message fragments for motor CRUD routes."""

    CREATE_MOTORS = "create motors"
    LOAD_MOTORS = "load motors"
    UPDATE_MOTOR = "update motor"
    DELETE_MOTOR = "delete motor"
    DELETE_MOTOR_FILE = "delete motor file"
    MERGE_MOTOR_FILES = "merge motor files"
    RENAME_MOTOR_FILE = "rename motor file"
