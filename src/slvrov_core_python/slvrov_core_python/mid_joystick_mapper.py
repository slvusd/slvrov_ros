from dataclasses import dataclass

import rclpy  # type: ignore
from rclpy.executors import ExternalShutdownException  # type: ignore
from rclpy.node import Node  # type: ignore

from sensor_msgs.msg import Joy  # type: ignore
from std_srvs.srv import Trigger  # type: ignore
from slvrov_interfaces.srv import String

from .control_objects import *
from .json_crud import *


#TODO make these enums generic
class JoystickMapperMessages(Enum):
    ACTIVATED_WITHOUT_TOPICS = "<activated without topics> "
    ACTIVATION_FAILED = "<activation failed> "
    SET_ACTION_BEFORE_CLEARED = "<attempted set action before current action was cleared> "
    INVALID_ACTION = "<invalid action> "
    INVALID_ACTION_TYPE = "<invalid action type> "
    SET_ACTION_FAILED = "<set action failed> "
    MAPPED_WITHOUT_ACTION = "<attempted map without action> "
    MAPPED_WHEN_INACTIVE = "<attempted map when inactive> "
    STOP_MAPPING_FAILED = "<failed to stop mapping> "
    TIE_FOUND = "<tie found> "
    SERVER_ERROR = "<server error check log> "
    UNCAUGHT_EXCEPTION = "<uncaught exception> "
    DEACTIVATE_WHILE_UNSAVED = "<attempted deactivate before mappings were saved> "
    DEACTIVATE_WHILE_MAPPING = "<attempted deactivate while mapping was active> "
    CANDIDATE_SCORE_NONE = "<candidate score is None> "
    JS_TOPICS_STALE = "<joystick topics might be stale> "
    CANDIDATES_NONE_BUT_UPDATE = "<candidates are None during update loop> "

    ACTIVATION_SUCCESSFUL = "<activation successful> "
    SET_ACTION_SUCCESSFUL = "<successfully set action> "
    START_MAPPING_SUCCESSFUL = "<successfully started mapping> "
    MAP_SUCCESSFUL = "<action mapping successful> "
    SAVE_SUCCESSFUL = "<mappings successfully saved> "
    DEACTIVATION_SUCCESSFUL = "<deactivation successful> "

    STARTED_UPDATES = "<started update timer> "
    STOPPED_UPDATES = "<stopped update timer> "
    SUBSCRIBED_TO_TOPIC = "<subscribed to topic> "
    SUBSCRIPTION_DESTROYED = "<subscription destroyed> "

    def __str__(self):
        return self.value
    

# TODO make this logger generic
@dataclass
class JoystickMapperMessageLog:
    logger: object
    base: str | JoystickMapperMessages = ""
    level: str = "warning"
    success: bool | None = None
    stem_message: str | JoystickMapperMessages = ""
    client_message: str | JoystickMapperMessages = ""
    log_message: str | JoystickMapperMessages = ""

    def do(self) -> tuple[bool, str]:
        """
        Logs and crafts response
        """
        self.log()
        return self.response()

    def cliet_str(self) -> str:
        return self.base + self.stem_message + self.client_message
    
    def log_str(self) -> str:
        return self.base + self.stem_message + self.log_message

    def response(self) -> tuple[bool, str]:
        return self.success, self.client_str()
    
    def log(self) -> None:
        match self.level:
            case "info": self.logger.info(self.log_str())
            case "warning": self.logger.warning(self.log_str())
            case "error": self.logger.error(self.log_str())
            case _: raise ValueError(f"Invalid message level: {self.level}")
    
    def get_success(self) -> None:
        if self.success is not None: return
        if self.level == "info": self.success = True
        else: self.success = False


class JoystickMapper(Node):

    def __init__(self) -> None:
        # TODO add params for allow reuse controls and reuse actions
        super().__init__("joystick_mapper")

        self.declare_parameter("joystick_topics", [])
        self.declare_parameter("joystick_mappings_path", "")
        self.declare_parameter("update_hz", 10.0)

        self.js_topics = [str(topic) for topic in self.get_parameter("joystick_topics").value]
        self.js_mappings_path = str(self.get_parameter("joystick_mappings_path").value)
        self.update_time = 1.0 / float(self.get_parameter("update_hz").value)

        self.js_subscriptions = None
        self.js_mappings: list[ROVActionMapping] = list()

        self.latest_js_states: dict[str, Joy] = dict()
        self.candidates: dict[MappingCandidate, MappingCandidate] | None = None

        self.actions: list[ROVAction] = []  # TODO this should belong to the client

        self.mapping = False
        self.current_action_mapping: ROVActionMapping | None = None
        self.toggle_mapping_service = self.create_service(Trigger, "joystick_mapper/set_mapping_state", self.toggle_mapping_callback)
        self.set_action_service = self.create_service(String, "joystick_mapper/set_action", self.set_action_callback)
        self.save_mappings_service = self.create_service(String, "joystick_mapper/save_mappings", self.save_mappings_callback)  #TODO

        self.active = False
        self.activation_service = self.create_service(String, "joystick_mapper/set_mapper_state", self.activation_callback)

        self.get_logger().info("Joystick Mapper is up and ready for clients.")

    def js_callback(self, topic: str, msg: Joy) -> None:
        if self.current_action_mapping is None: return  # if no action is currently being mapped but mapper is active, ignore joystick inputs
        self.latest_js_states[topic] = msg

        # If the subscriptions are active but candidates haven't been created yet, create them
        if self.candidates is None: self.create_candidates()
        
    def toggle_mapping_callback(self, req, resp):
        # if user hasn't activated joystick mapper
        if not self.active:
            resp.success, resp.message = JoystickMapperMessageLog(
                logger=self.get_logger(),
                base=JoystickMapperMessages.MAPPED_WHEN_INACTIVE,
                ).do()
            return resp
        
        # activate mapping
        if not self.mapping:
            # if user hasn't set action to map
            if self.current_action_mapping is None:
                resp.success, resp.message = JoystickMapperMessageLog(
                    logger=self.get_logger(),
                    base=JoystickMapperMessages.MAPPED_WITHOUT_ACTION,
                ).do()
                return resp

            # will update mapping candidates on timer
            self.start_candidate_updates()
            self.mapping = True

            resp.success, resp.message = JoystickMapperMessageLog(
                logger=self.get_logger(),
                level="info",
                base=JoystickMapperMessages.START_MAPPING_SUCCESSFUL,
                ).do()
            
        # deactivate mapping
        else:
            self.mapping = False
            self.stop_candidate_updates()

            candidate = self.get_best_candidate()

            # if tie
            if candidate is None:
                resp.success, resp.message = JoystickMapperMessageLog(
                    logger=self.get_logger(),
                    base=JoystickMapperMessages.TIE_FOUND,
                ).do()

            else:
                self.current_action_mapping.topic = candidate.topic
                self.current_action_mapping.index = candidate.source_index
                self.js_mappings.append(self.current_action_mapping)

                resp.success, resp.message = JoystickMapperMessageLog(
                    logger=self.get_logger(),
                    level="info",
                    base=JoystickMapperMessages.MAP_SUCCESSFUL,
                    stem_message=f"{self.current_action_mapping.action}={candidate.source_type}/{candidate.topic}/{candidate.source_index}@{candidate.score}"
                    ).do()
                
            self.current_action_mapping = None
            self.clear_candidates()

        return resp
    
    def set_action_callback(self, req, resp):
        """
        Set the current action to be mapped based on the request data string.
        """
        if self.current_action_mapping is not None:
            resp.success, resp.message = JoystickMapperMessageLog(
                logger=self.get_logger(),
                base=JoystickMapperMessages.SET_ACTION_BEFORE_CLEARED,
                ).do()
            return resp

        try:
            self.current_action_mapping = ROVActionMapping.from_string(req.data)

        except ValueError as exception:
            # While this validation should be handled client-side, best practice to call it here, too
            resp.success, resp.message = JoystickMapperMessageLog(
                logger=self.get_logger(),
                base=JoystickMapperMessages.INVALID_ACTION,
                ).do()
            return resp

        except Exception as exception:
            resp.success, resp.message = JoystickMapperMessageLog(
                logger=self.get_logger(),
                level="error",
                base=JoystickMapperMessages.SET_ACTION_FAILED,
                ).do()
            return resp

        resp.success, resp.message = JoystickMapperMessageLog(
            logger=self.get_logger(),
            level="info",
            base=JoystickMapperMessages.SET_ACTION_SUCCESSFUL,
            stem_message=f"current action={self.current_action_mapping}"
            ).do()
        return resp

    def activation_callback(self, req, resp):
        """
        Activate or deactivate the joystick mapper based on current state. If not currently active, activate with provided parameters; otherwise, deactivate.
        When activating, the request data should be a comma-separated string of joystick topics to subscribe to (e.g. "/joy1,/joy2").
        """

        # if not currently active, activate with provided parameters; otherwise, deactivate
        if not self.active:
            try:
                topics = req.data.split(",")
                self.js_topics = topics

            except Exception as exception:
                resp.success, resp.message = JoystickMapperMessageLog(
                    logger=self.get_logger(),
                    level="error",
                    base=JoystickMapperMessages.ACTIVATION_FAILED,
                ).do()
                return resp

            resp.success, resp.message = JoystickMapperMessageLog(
                logger=self.get_logger(),
                level="info",
                base=JoystickMapperMessages.ACTIVATION_SUCCESSFUL,
                ).do()
            
            self.subscribe_to_joytsicks()
            self.active = True
        
        else:
            self.unsubscribe_from_joysticks()

            if len(self.js_mappings) > 0:
                resp.success, resp.message = JoystickMapperMessageLog(
                    logger=self.get_logger(),
                    base=JoystickMapperMessages.DEACTIVATE_WHILE_UNSAVED,
                ).do()
                return resp
                self.js_mappings = list()

            if self.mapping:
                resp.success, resp.message = JoystickMapperMessageLog(
                    logger=self.get_logger(),
                    base=JoystickMapperMessages.DEACTIVATE_WHILE_MAPPING,
                ).do()
                return resp                
                # stop candidate updates and clear candidate
                self.toggle_mapping_callback(None, None)

            self.active = False
            self.clear_activation_states()

            resp.success, resp.message = JoystickMapperMessageLog(
                logger=self.get_logger(),
                level="info",
                base=JoystickMapperMessages.DEACTIVATION_SUCCESSFUL,
                ).do()

        return resp

    def save_mappings(self) -> None:
        mappings_dict = dict()

        for mapping in self.js_mappings:
            mappings_dict.update(mapping.__json__())

        save_to_json(mappings_dict, self.js_mappings_path)

        # LOGFIX1
        JoystickMapperMessageLog(
            logger=self.get_logger(),
            level="info",
            base=JoystickMapperMessages.SAVE_SUCCESSFUL
            ).do()

    def load_mappings(self) -> None:
        ... # TODO

    def edit_mapping(self) -> None:
        ... # TODO

    def delete_mapping(self) -> None:
        ... # TODO

    def clear_candidates(self) -> None:
        """
        Clear candidate scores. 
        This should be run at the end of each action mapping process, so that current candidates don't interfere with the future mapping.
        """
        for candidate in self.candidates.values():
            candidate.score = None

    def create_candidates(self) -> None:
        """
        When activated, Joystick Mapper will create MappingCandidates that will later be used to map actions.
        This should be run once per activation.
        """
        self.candidates = dict()

        for topic, js_state in self.latest_js_states.items():
            for index, initial_score in enumerate(js_state.buttons):
                candidate = MappingCandidate(topic, ROVActionType.JS_BUTTON, index, initial_score)
                self.candidates[candidate] = candidate

            for index, initial_score in enumerate(js_state.axes):
                candidate = MappingCandidate(topic, ROVActionType.JS_AXIS, index, initial_score)
                self.candidates[candidate] = candidate

    def update_candidates(self) -> None:
        """
        Update candidate scores using latest joystick states, run on a timer based on the update_hz parameter.
        This should be run once per toggle.
        """
        if self.candidates is None: 
            JoystickMapperMessageLog(
                logger=self.get_logger(), 
                base=JoystickMapperMessages.CANDIDATES_NONE_BUT_UPDATE, 
                stem_message=JoystickMapperMessages.JS_TOPICS_STALE
                ).log()
            return  # if candidates haven't been created yet, don't update

        for candidate in self.candidates.values():
            source_type = candidate.source_type
            if source_type != self.current_action_mapping.action.type: continue  # skip candidates that are unrelated to current action

            js_state = self.latest_js_states[candidate.topic]

            if source_type == ROVActionType.JS_AXIS: scores = js_state.axes
            elif source_type == ROVActionType.JS_BUTTON: scores = js_state.buttons
            else: 
                JoystickMapperMessageLog(
                    self.get_logger(), 
                    base=JoystickMapperMessages.INVALID_ACTION_TYPE
                    ).log()

            new_score = scores[candidate.source_index]
            if candidate.score is None: candidate.score = new_score
            if candidate.score < new_score: candidate.score = new_score

    def get_best_candidate(self) -> MappingCandidate | None:
        """
        Get the candidate with the highest difference between its score and initial_score from self.candidates that matches the current action type.
        If there are multiple candidates with the same best score, return None to indicate that no clear winner was found.
        This should be run once per toggle, after candidate updates have been stopped.
        """
        best_candidates = []
        best_score = None

        for candidate in self.candidates.values():
            # skip candidates that are unrelated to current action
            if candidate.source_type != self.current_action_mapping.action.type: continue
            if candidate.score is None: 
                JoystickMapperMessageLog(
                    logger=self.get_logger(),
                    base=JoystickMapperMessages.CANDIDATE_SCORE_NONE,
                    stem_message=JoystickMapperMessages.JS_TOPICS_STALE
                ).log()
                continue

            score_diff = abs(candidate.score - candidate.initial_score)
            if best_score is None: 
                best_candidates.append(candidate)
                best_score = score_diff

            elif score_diff > best_score:
                best_candidates = [candidate]
                best_score = score_diff

            elif score_diff == best_score:
                best_candidates.append(candidate)
            
        # If there are multiple candidates with the same best score, return None to indicate that no clear winner was found
        return best_candidates[0] if len(best_candidates) == 1 else None

    def clear_activation_states(self) -> None:
        """
        Resets all activation-related states.
        This should be called during the deactivation process.

        Resests:
        - joystick topics
        - joystick subscriptions
        - latest joystick states
        - mapping candidates
        """

        # This is a simple function, but I wanted to make the activation callback more readable
        self.js_topics = []
        self.js_subscriptions = None
        self.latest_js_states = dict()
        self.candidates = None

        # LOGFIX
        self.get_logger().info("Cleared activation states.")
        
    def subscribe_to_joytsicks(self) -> None:
        """
        Subscribe to joystick topics to receive Joy messages.
        This should be called during the activation process.
        """

        # if activated without passing topic names
        if len(self.js_topics) == 0: raise ValueError(JoystickMapperMessages.ACTIVATED_WITHOUT_TOPICS)

        # This is a simple function, but I wanted to make the activation callback more readable
        for topic in self.js_topics:
            self.js_subscriptions.append(self.create_subscription(Joy, topic, lambda msg, bound_topic=topic: self.js_callback(bound_topic, msg), 10))

            JoystickMapperMessageLog(
                logger=self.get_logger(),
                level="info",
                base=JoystickMapperMessages.SUBSCRIBED_TO_TOPIC,
                stem_message=f"{topic}"
            ).log()

    def unsubscribe_from_joysticks(self) -> None:
        """
        Unsubscribe from joystick topics.
        This should be called during the deactivation process.
        """

        # This is a simple function, but I wanted to make the activation callback more readable
        for subscription in self.js_subscriptions:
            self.destroy_subscription(subscription)
            JoystickMapperMessageLog(
                logger=self.get_logger(),
                level="info",
                base=JoystickMapperMessages.SUBSCRIPTION_DESTROYED,
                stem_message=f"{subscription}"
            ).log()

        self.js_subscriptions = None

    def start_candidate_updates(self) -> None:
        """
        Start a timer that updates candidate scores based on latest joystick states.
        This should be called during the mapping toggle process.
        """
        # This is a one-liner, but I wanted to make the activation callback more readable
        self.update_timer = self.create_timer(self.update_time, self.update_candidates)
        
        JoystickMapperMessageLog(
            logger=self.get_logger(),
            level="info",
            base=JoystickMapperMessages.STARTED_UPDATES
        ).log()

    def stop_candidate_updates(self) -> None:
        """
        Stop the candidate update timer.
        This should be called during the deactivation process.
        """
        # This is a one-liner, but I wanted to make the activation callback more readable
        self.update_timer.cancel()

        JoystickMapperMessageLog(
            logger=self.get_logger(),
            level="info",
            base=JoystickMapperMessages.STOPPED_UPDATES
        ).log()


def main(args=None):
    try:
        rclpy.init(args=args)
        node = JoystickMapper()
        rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        print("Shutdown signal received, exiting...")

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
