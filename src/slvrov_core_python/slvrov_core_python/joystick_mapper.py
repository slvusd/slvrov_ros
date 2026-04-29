from dataclasses import dataclass

import rclpy  # type: ignore
from rclpy.executors import ExternalShutdownException  # type: ignore
from rclpy.node import Node  # type: ignore

from sensor_msgs.msg import Joy  # type: ignore
from std_srvs.srv import Trigger  # type: ignore
from slvrov_interfaces.srv import String

from .control_objects import *
from .json_crud import *
from .log_messages import JoystickMapperLogMessages

# Client Flow:
# 1. Activate joystick mapper with list of joystick topics to subscribe to
#  - succcessfull with one topic w/out namespaces w/out remappings (4/26)
# 2. Set action to map using set_action service
#  - successfully set action to myAction/js_button/None/None (4/26)
# 3. Toggle mapping on to start mapping process
#  - successfully toggled mapping on (4/26)
# - server returned error when i hadn't set action yet, which is expected behavior (4/26)
# 4. Toggle mapping off to end mapping process and save mapping results
# - successfully toggled mapping off and mapped to the button I was pressing (4/26)
# - successfully found button was king when i moved axis too (4/26)
# - successfully returned error when there was a tie for best candidate for both buttons and axis (4/26)
# - successfully mapped to axis when i only moved axis (4/26)
# 5. Deactivate joystick mapper to save mappings and unsubscribe from joystick topics


class JoystickMapper(Node):
    def __init__(self) -> None:
        # TODO add params for allow reuse controls and reuse actions
        # TODO make params default if not provided by user service calls (two variables, one is param, one is user req)
        super().__init__("joystick_mapper")

        self.declare_parameter("joystick_topics", [])
        self.declare_parameter("joystick_mappings_path", "")
        self.declare_parameter("update_hz", 20.0)

        self.js_topics = [str(topic) for topic in self.get_parameter("joystick_topics").value]
        self.js_mappings_path = str(self.get_parameter("joystick_mappings_path").value)
        self.update_time = 1.0 / float(self.get_parameter("update_hz").value)

        self.js_subscriptions: list = list()  # returns to empty list when mapper inactive
        self.js_mappings: list[ROVActionMapping] = list()  # returns to empty list after save

        self.latest_js_states: dict[str, Joy] = dict()  # topic: state, returns to empty dict when mapper is inactive
        self.candidates: dict[str, MappingCandidate] | None = None  # topic/type/index: MappingCandidate, returns to None when mapper is inactive

        self.mapper_active = False
        self.set_mapper_state_service = self.create_service(String, "joystick_mapper/set_mapper_state", self.set_mapper_state_callback)
        self.get_logger().info(JoystickMapperLogMessages.SERVICE_CREATED + "joystick_mapper/set_mapper_state")

        self.current_action_mapping: ROVActionMapping | None = None  # returns to None when mapping is inactive
        self.set_action_service = self.create_service(String, "joystick_mapper/set_action", self.set_action_callback)
        self.get_logger().info(JoystickMapperLogMessages.SERVICE_CREATED + "joystick_mapper/set_action")

        self.mapping_active = False
        self.set_mapping_state_service = self.create_service(Trigger, "joystick_mapper/set_mapping_state", self.set_mapping_state_callback)
        self.get_logger().info(JoystickMapperLogMessages.SERVICE_CREATED + "joystick_mapper/set_mapping_state")

        self.save_state: bool | None = None  # set to none here because there are neither unsaved or saved changes
        self.save_mapped_actions_service = self.create_service(String, "joystick_mapper/save_mapped_actions", self.save_mapped_actions_callback)
        self.get_logger().info(JoystickMapperLogMessages.SERVICE_CREATED + "joystick_mapper/save_mapped_actions")

        self.get_logger().info(JoystickMapperLogMessages.NODE_READY + "joystick_mapper")

    def js_callback(self, topic: str, msg: Joy) -> None:
        #self.get_logger().info(f"Received joystick message on topic {topic}: {msg}")
        if self.current_action_mapping is None: return  # if no action is currently being mapped but mapper is active, ignore joystick inputs
        self.latest_js_states[topic] = msg

        # If the subscriptions are active but candidates haven't been created yet, create them
        if self.candidates is None: self.create_candidates()
        
    def set_mapping_state_callback(self, req, resp):        
        if not self.mapping_active: resp = self.set_mapping_active(req, resp)  
        else: resp = self.set_mapping_inactive(req, resp)
        return resp
    
    def set_action_callback(self, req, resp):
        """
        Set the current action to be mapped based on the request data string.
        """
        if self.current_action_mapping is not None:
            msg = (JoystickMapperLogMessages.ACTION_SET +
                   JoystickMapperLogMessages.SERVICE_CALL_FAILED +
                   JoystickMapperLogMessages.ACTION_ALREADY_SET)
            
            self.get_logger().warning(msg)
            resp.success, resp.message = False, msg
            return resp

        try:
            self.current_action_mapping = ROVActionMapping.from_string(req.data)

        except ValueError as exception:  # While this validation should be handled client-side, best practice to call it here, too
            msg = (JoystickMapperLogMessages.ACTION_SET +
                   JoystickMapperLogMessages.SERVICE_CALL_FAILED +
                   JoystickMapperLogMessages.ACTION_INVALID +
                   req.data)
            
            self.get_logger().warning(msg)
            resp.success, resp.message = False, msg
            return resp

        except Exception as exception:
            msg = (JoystickMapperLogMessages.ACTION_SET +
                   JoystickMapperLogMessages.SERVICE_CALL_FAILED +
                   JoystickMapperLogMessages.UNCAUGHT_EXCEPTION +
                   str(exception))
            
            self.get_logger().error(msg)
            resp.success, resp.message = False, msg
            return resp

        msg = (JoystickMapperLogMessages.ACTION_SET +
               JoystickMapperLogMessages.SERVICE_CALL_SUCCEEDED +
               JoystickMapperLogMessages.ACTION_IS_SET +
               f"{self.current_action_mapping}")
        
        self.get_logger().info(msg)
        resp.success, resp.message = True, msg
        return resp

    def set_mapper_state_callback(self, req, resp):
        """
        Activate or deactivate the joystick mapper based on current state. If not currently active, activate with provided parameters; otherwise, deactivate.
        When activating, the request data should be a comma-separated string of joystick topics to subscribe to (e.g. "/joy1,/joy2").
        """

        if not self.mapper_active: resp = self.set_mapper_active(req, resp)
        else: resp = self.set_mapper_inactive(req, resp)
        return resp

    def save_mapped_actions_callback(self, req, resp):
        if req.data != "": mappings_path = req.data
        # if user hasn't sent new path and path parameter is empty
        elif self.js_mappings_path == "":
            msg = (JoystickMapperLogMessages.SAVE_MAPPED_ACTIONS +
                   JoystickMapperLogMessages.SERVICE_CALL_FAILED +
                   JoystickMapperLogMessages.NO_PATH_PROVIDED)
            
            self.get_logger().warning(msg)
            resp.success, resp.message = False, msg
            return resp
        
        else: mappings_path = self.js_mappings_path
        
        mappings_dict = dict()
        for mapping in self.js_mappings:
            mappings_dict.update(mapping.__json__())

        save_to_json(mappings_dict, mappings_path)
        self.js_mappings = list()  # return to pre-save state

        msg = (JoystickMapperLogMessages.SAVE_MAPPED_ACTIONS +
               JoystickMapperLogMessages.SERVICE_CALL_SUCCEEDED +
               JoystickMapperLogMessages.SAVE_SUCCESSFUL)
        
        self.get_logger().info(msg)
        resp.success, resp.message = True, msg

    def set_mapper_active(self, req: object, resp: object) -> object:
        # validate input
        try:
            if req.data != "":
                topics = req.data.split(",")
                self.js_topics = topics

            # if no topics were provided in parameters or request
            elif self.js_topics == []:
                msg = (JoystickMapperLogMessages.MAPPER_SET_ACTIVE +
                        JoystickMapperLogMessages.SERVICE_CALL_FAILED +
                        JoystickMapperLogMessages.NO_TOPICS)
            
                self.get_logger().warning(msg)
                resp.success, resp.message = False, msg
                return resp


        except Exception as exception:
            msg = (JoystickMapperLogMessages.MAPPER_SET_ACTIVE +
                    JoystickMapperLogMessages.SERVICE_CALL_FAILED +
                    JoystickMapperLogMessages.UNCAUGHT_EXCEPTION +
                    str(exception))
            
            self.get_logger().error(msg)
            resp.success, resp.message = False, msg
            return resp

        # activate mapper (subscribe to joystick topics)
        self.subscribe_to_joytsicks()
        self.mapper_active = True

        msg = (JoystickMapperLogMessages.MAPPER_SET_ACTIVE +
                JoystickMapperLogMessages.SERVICE_CALL_SUCCEEDED +
                JoystickMapperLogMessages.MAPPER_IS_ACTIVE)
        
        self.get_logger().info(msg)
        resp.success, resp.message = True, msg
        return resp

    def set_mapper_inactive(self, req: object, resp: object) -> object:
        if self.mapping_active:
            msg = (JoystickMapperLogMessages.MAPPER_SET_INACTIVE +
                    JoystickMapperLogMessages.SERVICE_CALL_FAILED +
                    JoystickMapperLogMessages.MAPPING_IS_ACTIVE)

            self.get_logger().warning(msg)
            resp.success, resp.message = False, msg
            return resp

        self.unsubscribe_from_joysticks()

        self.mapper_active = False
        self.js_subscriptions = list()
        self.latest_js_states = dict()
        self.candidates = None

        msg = (JoystickMapperLogMessages.MAPPER_SET_INACTIVE +
                JoystickMapperLogMessages.SERVICE_CALL_SUCCEEDED +
                JoystickMapperLogMessages.MAPPER_IS_INACTIVE)

        self.get_logger().info(msg)
        resp.success, resp.message = True, msg
        return resp

    def set_mapping_active(self, req: object, resp: object) -> object:
        # if user hasn't activated joystick mapper
        if not self.mapper_active:
            msg = (JoystickMapperLogMessages.MAPPER_SET_ACTIVE +
                    JoystickMapperLogMessages.SERVICE_CALL_FAILED + 
                    JoystickMapperLogMessages.MAPPER_IS_INACTIVE)
            
            self.get_logger().warning(msg)
            resp.success, resp.message = False, msg
            return resp
        
        # if user hasn't set action to map
        if self.current_action_mapping is None:
            msg = (JoystickMapperLogMessages.MAPPER_SET_ACTIVE +
                    JoystickMapperLogMessages.SERVICE_CALL_FAILED +
                    JoystickMapperLogMessages.NO_ACTION_SET)
            
            self.get_logger().warning(msg)
            resp.success = False, resp.message = msg
            return resp

        # will update mapping candidates on timer
        self.start_candidate_updates()
        self.mapping_active = True

        msg = (JoystickMapperLogMessages.MAPPER_SET_ACTIVE +
                JoystickMapperLogMessages.SERVICE_CALL_SUCCEEDED +
                JoystickMapperLogMessages.MAPPER_IS_ACTIVE)
        
        self.get_logger().info(msg)
        resp.success, resp.message = True, msg
        return resp

    def set_mapping_inactive(self, req: object, resp: object) -> object:
        self.mapping_active = False
        self.stop_candidate_updates()

        candidate = self.get_best_candidate()

        # if no candidates were found
        if candidate == JoystickMapperLogMessages.MAPPING_NO_CANDIDATE:
            msg = (JoystickMapperLogMessages.MAPPER_SET_INACTIVE +
                    JoystickMapperLogMessages.SERVICE_CALL_SUCCEEDED +
                    JoystickMapperLogMessages.MAPPING_IS_INACTIVE +
                    JoystickMapperLogMessages.MAPPING_FAILED + 
                    JoystickMapperLogMessages.MAPPING_NO_CANDIDATE +
                    JoystickMapperLogMessages.SERVER_ERROR)  # if no candidates were found, then it's a server error

            self.get_logger().warning(msg)
            resp.success, resp.message = False, msg
            return

        # if tie
        if candidate == JoystickMapperLogMessages.MAPPING_CANDIDATE_TIE:
            msg = (JoystickMapperLogMessages.MAPPER_SET_INACTIVE +
                    JoystickMapperLogMessages.SERVICE_CALL_SUCCEEDED +
                    JoystickMapperLogMessages.MAPPING_IS_INACTIVE +
                    JoystickMapperLogMessages.MAPPING_FAILED + 
                    JoystickMapperLogMessages.MAPPING_CANDIDATE_TIE)

            self.get_logger().warning(msg)
            resp.success, resp.message = False, msg
            return

        self.current_action_mapping.topic = candidate.topic
        self.current_action_mapping.index = candidate.source_index
        self.js_mappings.append(self.current_action_mapping)

        msg = (JoystickMapperLogMessages.MAPPER_SET_INACTIVE +
                JoystickMapperLogMessages.SERVICE_CALL_SUCCEEDED +
                JoystickMapperLogMessages.MAPPING_IS_INACTIVE +
                JoystickMapperLogMessages.MAPPING_SUCCEEDED +
                JoystickMapperLogMessages.MAPPING_FOUND_CANDIDATE + 
                f"{self.current_action_mapping.action}->" +
                f"{candidate.topic}/" +
                f"{candidate.source_type}/" +
                f"{candidate.source_index}={candidate.score}")
        
        self.get_logger().info(msg)
        resp.success, resp.message = True, msg

        self.current_action_mapping = None
        self.clear_candidates()

        return resp

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

        self.get_logger().info("Clearing candidates after mapping inactive")

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

        self.get_logger().info("Creating candidates")

    def update_candidates(self) -> None:
        """
        Update candidate scores using latest joystick states, run on a timer based on the update_hz parameter.
        This should be run once per toggle.
        """
        if self.candidates is None: 
            self.get_logger().warning("No candidates available for update.")
            return  # if candidates haven't been created yet, don't update

        for candidate in self.candidates.values():
            source_type = candidate.source_type
            if source_type != self.current_action_mapping.action.type: continue  # skip candidates that are unrelated to current action

            js_state = self.latest_js_states[candidate.topic]

            if source_type == ROVActionType.JS_AXIS: scores = js_state.axes
            elif source_type == ROVActionType.JS_BUTTON: scores = js_state.buttons
            else:  # TODO make this for when people create actions, not during mapping process
                self.get_logger().warning(f"Action type {self.current_action_mapping.action.type} isn't supported for mapping.")

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
                self.get_logger().warning(f"Found candidate with None score: {candidate} " + JoystickMapperLogMessages.TOPIC_STALE)
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
        
        # If there are multiple candidates with the same best score, return None to indicate that no clear winner was found, 
        if len(best_candidates) == 1: return best_candidates[0]
        elif len(best_candidates) == 0: return JoystickMapperLogMessages.MAPPING_NO_CANDIDATE
        return JoystickMapperLogMessages.MAPPING_CANDIDATE_TIE

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

        self.get_logger().info("Cleared activation states.")
        
    def subscribe_to_joytsicks(self) -> None:
        """
        Subscribe to joystick topics to receive Joy messages.
        This should be called during the activation process.
        """

        # This is a simple function, but I wanted to make the activation callback more readable
        for topic in self.js_topics:
            self.js_subscriptions.append(self.create_subscription(Joy, topic, lambda msg, bound_topic=topic: self.js_callback(bound_topic, msg), 10))
            self.get_logger().info(JoystickMapperLogMessages.SUBSCRIPTION_CREATED + topic)

    def unsubscribe_from_joysticks(self) -> None:
        """
        Unsubscribe from joystick topics.
        This should be called during the deactivation process.
        """

        # This is a simple function, but I wanted to make the activation callback more readable
        for subscription in self.js_subscriptions:
            self.destroy_subscription(subscription)
            self.get_logger().info(JoystickMapperLogMessages.SUBSCRIPTION_DESTROYED + subscription.topic_name)

    def start_candidate_updates(self) -> None:
        """
        Start a timer that updates candidate scores based on latest joystick states.
        This should be called during the mapping toggle process.
        """

        # This is a one-liner, but I wanted to make the activation callback more readable
        self.update_timer = self.create_timer(self.update_time, self.update_candidates)
        self.get_logger().info(JoystickMapperLogMessages.START_TIMER + str(self.update_candidates))

    def stop_candidate_updates(self) -> None:
        """
        Stop the candidate update timer.
        This should be called during the deactivation process.
        """

        # This is a one-liner, but I wanted to make the activation callback more readable
        self.update_timer.cancel()
        self.get_logger().info(JoystickMapperLogMessages.STOP_TIMER + str(self.update_candidates))


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
