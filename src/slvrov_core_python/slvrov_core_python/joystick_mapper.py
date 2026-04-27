from dataclasses import dataclass

import rclpy  # type: ignore
from rclpy.executors import ExternalShutdownException  # type: ignore
from rclpy.node import Node  # type: ignore

from sensor_msgs.msg import Joy  # type: ignore
from std_srvs.srv import Trigger  # type: ignore
from slvrov_interfaces.srv import String

from .control_objects import *
from .json_crud import *

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
        self.toggle_mapping_service = self.create_service(Trigger, "joystick_mapper/toggle_mapping", self.toggle_mapping_callback)
        self.set_action_service = self.create_service(String, "joystick_mapper/set_action", self.set_action_callback)

        self.active = False
        self.activation_service = self.create_service(String, "joystick_mapper/activation", self.activation_callback)

        self.get_logger().info("Joystick Mapper is up and ready for clients.")

    def js_callback(self, topic: str, msg: Joy) -> None:
        #self.get_logger().info(f"Received joystick message on topic {topic}: {msg}")
        if self.current_action_mapping is None: return  # if no action is currently being mapped but mapper is active, ignore joystick inputs
        self.latest_js_states[topic] = msg

        # If the subscriptions are active but candidates haven't been created yet, create them
        if self.candidates is None: self.create_candidates()
        
    def toggle_mapping_callback(self, req, resp):
        # if user hasn't activated joystick mapper
        if not self.active:
            self.get_logger().warning("Joystick mapper is not active but toggle_mapping was called.")

            resp.success = False
            resp.message = "Joystick mapper is not active. Please activate before toggling mapping."
            return resp
        
        # activate mapping
        if not self.mapping:
            # if user hasn't set action to map
            if self.current_action_mapping is None:
                self.get_logger().warning("No current action mapping set, but toggle_mapping was called.")

                resp.success = False
                resp.message = "No current action mapping set. Please set action before toggling mapping."
                return resp

            # will update mapping candidates on timer
            self.start_candidate_updates()
            self.mapping = True
            
            self.get_logger().info(f"Started mapping process for {self.current_action_mapping.action}. Please interact with the desired control. Toggle mapping to find the best match.")
            resp.success = True
            resp.message = f"Started mapping process for {self.current_action_mapping.action}. Please interact with the desired control. Toggle mapping to find the best match."
        
        # deactivate mapping
        else:
            self.mapping = False
            self.stop_candidate_updates()

            candidate = self.get_best_candidate()

            # if tie
            if candidate is None:
                self.get_logger().warning(f"No clear winner found for {self.current_action_mapping.action}. This action will not be mapped to any joystick controls.")

                resp.success = False
                resp.message = f"No clear winner found for {self.current_action_mapping.action}. This action will not be mapped to any joystick controls."

            else:
                self.current_action_mapping.topic = candidate.topic
                self.current_action_mapping.index = candidate.source_index
                self.js_mappings.append(self.current_action_mapping)

                self.get_logger().info(f"Mapped {self.current_action_mapping.action} to {candidate.source_type} {candidate.source_index} on {candidate.topic} with score {candidate.score}.")
                resp.success = True
                resp.message = f"Mapped {self.current_action_mapping.action} to {candidate.source_type} {candidate.source_index} on {candidate.topic} with score {candidate.score}."

            self.current_action_mapping = None
            self.clear_candidates()

        return resp
    
    def set_action_callback(self, req, resp):
        """
        Set the current action to be mapped based on the request data string.
        """
        if self.current_action_mapping is not None:
            self.get_logger().warning(f"Current action mapping is {self.current_action_mapping}, but set_action was called again before mapping was reset.")

            resp.success = False
            resp.message = f"Current action mapping is {self.current_action_mapping}, but set_action was called again before action was reset."
            return resp

        try:
            self.current_action_mapping = ROVActionMapping.from_string(req.data)

        except ValueError as exception:
            # While this validation should be handled client-side, best practice to call it here, too
            self.get_logger().error(f"ValueError occured, likely invalid action type:\n{exception}")

            resp.success = False
            resp.message = f"Invalid request. Check action type is ROVActionType Enum. Make sure all required fields are filled (name/type/topic/index). Check logs for details."
            return resp

        except Exception as exception:
            self.get_logger().error(f"Error occurred while setting current action: {exception}")

            resp.success = False
            resp.message = f"Error occurred while setting current action. Check logs for details."
            return resp

        self.get_logger().info(f"Set current action to {self.current_action_mapping}.")

        resp.success = True
        resp.message = f"Set current action to {self.current_action_mapping}"
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
                self.get_logger().error(f"Exception caught during activation:\n{exception}")

                resp.success = False
                resp.message = f"Something went wrong during activation. Check logs for details."
                return resp

            self.subscribe_to_joytsicks()
            self.active = True

            resp.success = True
            resp.message = "Successfully activated joystick mapper."
        
        else:
            self.unsubscribe_from_joysticks()

            if len(self.js_mappings) > 0:
                msg = f"Deactivation: There are unsaved mappings during deactivation. Saving joystick mappings to {self.js_mappings_path}."

                self.get_logger().warning(msg)
                resp.message += msg + "\n"

                self.save_mappings()  # TODO
                self.js_mappings = list()

            if self.mapping:
                msg = f"Deactivation: Deactivating while mapping still active for {self.current_action_mapping}. This action will not be mapped to any joystick controls."
                
                # stop candidate updates and clear candidate
                self.toggle_mapping_callback(None, None)

                self.get_logger().warning(msg)
                resp.message += msg + "\n"

            self.active = False
            self.clear_activation_states()

            self.get_logger().info("Successfully deactivated joystick mapper.")
            resp.success = True
            resp.message += "Successfully deactivated joystick mapper."

        return resp

    def save_mappings(self) -> None:
        mappings_dict = dict()

        for mapping in self.js_mappings:
            mappings_dict.update(mapping.__json__())

        save_to_json(mappings_dict, self.js_mappings_path)
        self.get_logger().info(f"Saved {len(self.js_mappings)} joystick mappings to {self.js_mappings_path}.")

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
                self.get_logger().warning(f"Found candidate {candidate} with None score. A joystick topic might've stopped publishing during the mapping process.")
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

        self.get_logger().info("Cleared activation states.")
        
    def subscribe_to_joytsicks(self) -> None:
        """
        Subscribe to joystick topics to receive Joy messages.
        This should be called during the activation process.
        """

        # This is a simple function, but I wanted to make the activation callback more readable
        for topic in self.js_topics:
            self.get_logger().info(f"Subscribing to {topic}...")
            self.js_subscriptions.append(self.create_subscription(Joy, topic, lambda msg, bound_topic=topic: self.js_callback(bound_topic, msg), 10))

        self.get_logger().info("Successfully subscribed to joystick topics.")

    def unsubscribe_from_joysticks(self) -> None:
        """
        Unsubscribe from joystick topics.
        This should be called during the deactivation process.
        """

        # This is a simple function, but I wanted to make the activation callback more readable
        for subscription in self.js_subscriptions:
            self.get_logger().info(f"Unsubscribing from {subscription.topic_name}...")
            self.destroy_subscription(subscription)

        self.js_subscriptions = None
        self.get_logger().info("Successfully unsubscribed from joystick topics.")

    def start_candidate_updates(self) -> None:
        """
        Start a timer that updates candidate scores based on latest joystick states.
        This should be called during the mapping toggle process.
        """

        # This is a one-liner, but I wanted to make the activation callback more readable
        self.update_timer = self.create_timer(self.update_time, self.update_candidates)
        self.get_logger().info("Started candidate update timer.")

    def stop_candidate_updates(self) -> None:
        """
        Stop the candidate update timer.
        This should be called during the deactivation process.
        """

        self.get_logger().info("Stopping candidate update timer.")
        # This is a one-liner, but I wanted to make the activation callback more readable
        self.update_timer.cancel()


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
