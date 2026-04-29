from dataclasses import dataclass
from enum import Enum


class LogMessages(Enum):
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
    SUBSCRIPTION_CREATED= "subscription created"
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
    MISSING_FIELD = "missing field"
    # TODO Are there destroy function for pub and srv and clients?

    def __str__(self):
        return f"<{self.value}> "

    def __eq__(self, value):
        return self.value == value


class JoystickMapperLogMessages(LogMessages):
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
    MAPPING_FOUND_CANDIDATE= "mapping found candidate for action"
    MAPPING_NO_CANDIDATE = "mapping does not have candidate for action"
    MAPPING_CANDIDATE_TIE = "mapping has more than one candidate for action"
    MAPPING_FAILED = "mapping failed"
    MAPPING_SUCCEEDED = "mapping succeeded"
    SAVE_MAPPED_ACTIONS = "save mapped actions"