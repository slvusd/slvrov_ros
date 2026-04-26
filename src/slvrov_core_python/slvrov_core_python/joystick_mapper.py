from dataclasses import dataclass

import rclpy  # type: ignore
from rclpy.executors import ExternalShutdownException  # type: ignore
from rclpy.node import Node  # type: ignore

from sensor_msgs.msg import Joy  # type: ignore
# Joy:
# std_msgs/Header header
# int32[] axes
# int32[] buttons

from std_srvs.srv import Trigger, Empty  # type: ignore
# Trigger:
# ---
# bool success
# string message

# Empty:
# ---

from slvrov_interfaces.srv import String
# String:
# string data
# ---
# bool success
# string msg

from .control_objects import *

# Client Flow:
# 1. Activate joystick mapper with list of joystick topics to subscribe to
# 2. Set action to map using set_action service
# 3. Toggle mapping on to start mapping process
# 4. Toggle mapping off to end mapping process and save mapping results


class JoystickMapper(Node):
    def __init__(self) -> None:
        super().__init__("joystick_mapper")

        self.declare_parameter("joystick_topics", [])
        self.declare_parameter("joystick_mappings_path", "")
        self.declare_parameter("update_hz", 25.0)

        self.js_topics = [str(topic) for topic in self.get_parameter("joystick_topics").value]
        self.js_mappings_path = str(self.get_parameter("joystick_mappings_path").value)
        self.update_time = 1.0 / float(self.get_parameter("update_hz").value)

        self.js_subscriptions = None
        self.js_mappings: list[ROVActionMapping] | None = None

        self.latest_js_states: dict[str, Joy] = dict()
        self.candidates: dict[MappingCandidate, MappingCandidate] | None = None

        self.actions: list[ROVAction] = []  # TODO this should belong to the client

        self.current_action_mapping: ROVActionMapping | None = None
        self.toggle_mapping_service = self.create_service(Empty, "toggle_mapping", self.toggle_mapping_callback)
        self.set_action_service = self.create_service(String, "set_action", self.set_action_callback)

        self.active = False
        self.activation_service = self.create_service(String, "joystick_mapper_activation", self.activation_callback)

    def js_callback(self, topic: str, msg: Joy) -> None:
        if self.current_action_mapping is None: return  # if no action is currently being mapped but mapper is active, ignore joystick inputs
        self.latest_js_states[topic] = msg

        # If the subscriptions are active but candidates haven't been created yet, create them
        if self.candidates is None: self.create_candidates()

    def create_candidates(self) -> None:
        """
        When activated, Joystick Mapper will create MappingCandidates that will later be used to map actions.
        This should be run once per activation.
        """
        for topic, js_state in self.latest_js_states.items():
            for index, score in enumerate(js_state.buttons):
                candidate = MappingCandidate(topic, ROVActionType.JS_BUTTON, index, score)
                self.candidates[candidate] = candidate

            for index, score in enumerate(js_state.axes):
                candidate = MappingCandidate(topic, ROVActionType.JS_AXIS, index, score)
                self.candidates[candidate] = candidate

    def update_candidates(self) -> None:
        """
        Update candidate scores using latest joystick states, run on a timer based on the update_hz parameter.
        This should be run once per toggle.
        """
        for candidate in self.candidates.values():
            source_type = candidate.source_type
            if source_type != self.current_action_mapping.action.type: continue  # skip candidates that are unrelated to current action

            js_state = self.latest_js_states[candidate.topic]

            if source_type == ROVActionType.JS_AXIS: scores = js_state.axes
            elif source_type == ROVActionType.JS_BUTTON: scores = js_state.buttons
            else:  # TODO make this for when people create actions, not during mapping process
                self.get_logger().warning(f"Action type {self.current_action_mapping.action.type} isn't supported for mapping.")

            new_score = scores[candidate.source_index]
            if candidate.score is None: continue
            if candidate.score < new_score: candidate.score = new_score

    def toggle_mapping_callback(self, req, resp):
        if not self.active:
            self.get_logger().warning("Joystick mapper is not active but toggle_mapping was called.")

            resp.success = False
            resp.message = "Joystick mapper is not active. Please activate before toggling mapping."
            return resp
        
        if self.current_action_mapping is None:
            # will update mapping candidates on timer
            self.start_candidate_updates()

            resp.success = True
            resp.message = f"Started mapping process for {self.current_action_mapping.action}. Please interact with the desired control. Toggle mapping to find the best match."
        
        else:
            self.stop_candidate_updates()
            candidate = self.get_best_candidate()
            if candidate is None:
                self.get_logger().warning(f"No clear winner found for {self.current_action_mapping.action}. This action will not be mapped to any joystick controls.")

                resp.success = False
                resp.message = f"No clear winner found for {self.current_action_mapping.action}. This action will not be mapped to any joystick controls."

            else:
                self.js_mappings.append(ROVActionMapping(candidate.source_type, candidate.source_index, self.current_action_mapping.action))

                self.get_logger().info(f"Mapped {self.current_action_mapping.action} to {candidate.source_type} {candidate.source_index} on {candidate.topic} with score {candidate.score}.")
                resp.success = True
                resp.message = f"Mapped {self.current_action_mapping.action} to {candidate.source_type} {candidate.source_index} on {candidate.topic} with score {candidate.score}."

            self.current_action_mapping = None
            self.clear_candidates()

        return resp

    def get_best_candidate(self) -> MappingCandidate | None:
        """
        Get the candidate with the highest score from self.candidates that matches the current action type.
        If there are multiple candidates with the same best score, return None to indicate that no clear winner was found.
        This should be run once per toggle, after candidate updates have been stopped.
        """
        best_candidates = []
        best_score = None

        for candidate in self.candidates.values():
            if candidate.type != self.current_action_mapping.action.type: continue  # skip candidates that are unrelated to current action
            if candidate.score is None: continue  # skip candidates that haven't been updated yet

            if best_score is None: 
                best_candidates.append(candidate)
                best_score = candidate.score

            elif candidate.score > best_score:
                best_candidates = [candidate]
                best_score = candidate.score

            elif candidate.score == best_score:
                best_candidates.append(candidate)
            
        # If there are multiple candidates with the same best score, return None to indicate that no clear winner was found
        return best_candidates[0] if len(best_candidates) == 1 else None
    
    def set_action_callback(self, req, resp):
        """
        Set the current action to be mapped based on the request data string.
        The request data should be a string of the format "name/type", where type is an ROVActionType Enum (e.g. "strafe/axis").
        """
        try:
            self.current_action_mapping = ROVActionMapping.from_string(req.data)

        except ValueError as exception:
            # While this validation should be handled client-side, best practice to call it here, too
            self.get_logger().error(f"ValueError occured, likely invalid action type:\n{exception}")

            resp.success = False
            resp.message = f"Invalid action type. Please use ROVActionType Enums."
            return resp

        except Exception as exception:
            self.get_logger().error(f"Error occurred while setting current action: {exception}")

            resp.success = False
            resp.message = f"Error occurred while setting current action. Check logs for details."
            return resp

        self.get_logger().info(f"Set current action to {self.current_action_mapping}.")

        resp.success = True
        resp.message = f"Set current action to {self.current_action_mapping}."
        return resp

    def clear_candidates(self) -> None:
        """
        Clear candidate scores. 
        This should be run at the end of each action mapping process, so that current candidates don't interfere with the future mapping.
        """
        for candidate in self.candidates.values():
            candidate.score = None

    def activation_callback(self, req, resp):
        """
        Activate or deactivate the joystick mapper based on current state. If not currently active, activate with provided parameters; otherwise, deactivate.
        When activating, the request data should be a comma-separated string of joystick topics to subscribe to (e.g. "/joy1,/joy2").
        """

        # if not currently active, activate with provided parameters; otherwise, deactivate
        if not self.active:
            try:
                topics = req.data.split(",")
                self.topics = topics

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

            if self.js_mappings is not None:
                msg = f"Deactivation: Saving joystick mappings to {self.js_mappings_path}."

                resp.message += msg + "\n"
                self.get_logger().info(msg)

                self.save_mappings()  # TODO
                self.js_mappings = None

            if self.current_action_mapping is not None:
                msg = f"Deactivation: Deactivating while mapping still active for {self.current_action_mapping}. This action will not be mapped to any joystick controls."
                
                # stop candidate updates and clear candidate
                self.toggle_mapping_callback(None, None)

                self.get_logger().warning(msg)
                resp.message += msg + "\n"

            self.active = False
            self.clear_activation_states()

            resp.success = True
            resp.message += "Successfully deactivated joystick mapper."

        return resp
    
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
        
    def subscribe_to_joytsicks(self) -> None:
        """
        Subscribe to joystick topics to receive Joy messages.
        This should be called during the activation process.
        """

        # This is a simple function, but I wanted to make the activation callback more readable
        self.js_subscriptions = [
        self.create_subscription(
            Joy, 
            topic, 
            lambda msg, bound_topic=topic: self.js_callback(bound_topic, msg), 
            10
            ) for topic in self.js_topics]

        self.get_logger().info("Successfully subscribed to joystick topics.")

    def unsubscribe_from_joysticks(self) -> None:
        """
        Unsubscribe from joystick topics.
        This should be called during the deactivation process.
        """

        # This is a simple function, but I wanted to make the activation callback more readable
        for subscription in self.js_subscriptions:
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

    def stop_candidate_updates(self) -> None:
        """
        Stop the candidate update timer.
        This should be called during the deactivation process.
        """

        # This is a one-liner, but I wanted to make the activation callback more readable
        self.update_timer.cancel()


def main(args=None):
    node = None
    try:
        rclpy.init(args=args)
        node = JoystickMapper()

    except (KeyboardInterrupt, ExternalShutdownException):
        print("Shutdown signal received, exiting...")
    finally:
        if node is not None:
            node.shutdown_command_prompt = True
            if not node.finished:
                node.get_logger().info(
                    "Saving current progress before exit."
                )
                node._save_output()
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
