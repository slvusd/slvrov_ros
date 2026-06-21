"""ROS2 node for interactively mapping joystick inputs to ROV actions."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import rclpy  # type: ignore
from rclpy.executors import ExternalShutdownException  # type: ignore
from rclpy.node import Node  # type: ignore
from sensor_msgs.msg import Joy  # type: ignore
from slvrov_interfaces.srv import String
from slvrov_interfaces.srv.JoystickMapper import (
    DeleteMapping,
    FetchStatus,
    SendMapping,
    SetAction,
    SetMapperState,
)
from std_srvs.srv import Trigger  # type: ignore

from ..control_objects import MappingCandidate, ROVActionMapping, ROVActionType
from ..json_crud import load_from_json, save_to_json  # type: ignore # TODO: only have mapping done here and move crud somewhere else
from ..log_messages import BaseLogMessages, JoystickMapperLogMessages

ActionReq = SetAction.Request
ActionResp = SetAction.Response
DeleteReq = DeleteMapping.Request
DeleteResp = DeleteMapping.Response
FetchReq = FetchStatus.Request
FetchResp = FetchStatus.Response
MapperReq = SetMapperState.Request
MapperResp = SetMapperState.Response
SendReq = SendMapping.Request
SendResp = SendMapping.Response
StringReq = String.Request
StringResp = String.Response
TriggerReq = Trigger.Request
TriggerResp = Trigger.Response


class JoystickMapper(Node):
    """Manages joystick mapping state and mapping services.

    This node subscribes to joystick topics, records candidate input movement,
    and exposes services for mapping ROV actions to joystick inputs.

    Attributes:
        js_topics (list[str]): Configured joystick topics from ROS parameters.
        js_mappings_path (str): Default JSON path for saved joystick mappings.
        update_time (float): Timer interval used to update mapping candidates.
        js_subscriptions (list[Any]): Active joystick subscriptions.
        js_mappings (list[ROVActionMapping]): Unsaved action mappings.
        latest_js_states (dict[str, Joy]): Most recent joystick message by topic.
        candidates (dict[str, MappingCandidate] | None): Active mapping candidates.
        update_timer (Any | None): Timer used during an active mapping run.
        mapper_active (bool): True when joystick subscriptions are active.
        mapping_active (bool): True when a mapping run is active.
        mappings_saved (bool): True when no unsaved mappings remain.
        current_action_mapping (ROVActionMapping | None): Action being mapped.
    """

    def __init__(self) -> None:
        super().__init__("joystick_mapper")

        self.declare_parameter("joystick_topics", [])
        self.declare_parameter("joystick_mappings_path", "")
        self.declare_parameter("update_hz", 20.0)

        self.js_topics = [
            str(topic) for topic in self.get_parameter("joystick_topics").value
        ]
        self.js_mappings_path = str(
            self.get_parameter("joystick_mappings_path").value
        )
        self.update_time = 1.0 / float(self.get_parameter("update_hz").value)

        self.js_subscriptions: list[Any] = []
        self.js_mappings: list[ROVActionMapping] = []
        self.latest_js_states: dict[str, Joy] = {}
        self.candidates: dict[str, MappingCandidate] | None = None
        self.update_timer: Any | None = None

        self.mapper_active = False
        self.mapping_active = False
        self.mappings_saved = True
        self.current_action_mapping: ROVActionMapping | None = None

        self.set_mapper_state_service = self.create_service(
            SetMapperState,
            "joystick_mapper/set_mapper_state",
            self.set_mapper_state_callback,
        )
        self.get_logger().info(
            BaseLogMessages.SERVICE_CREATED
            + "joystick_mapper/set_mapper_state"
        )

        self.set_action_service = self.create_service(
            SetAction,
            "joystick_mapper/set_action",
            self.set_action_callback,
        )
        self.get_logger().info(
            BaseLogMessages.SERVICE_CREATED + "joystick_mapper/set_action"
        )

        self.set_mapping_state_service = self.create_service(
            Trigger,
            "joystick_mapper/set_mapping_state",
            self.set_mapping_state_callback,
        )
        self.get_logger().info(
            BaseLogMessages.SERVICE_CREATED
            + "joystick_mapper/set_mapping_state"
        )

        self.save_mapped_actions_service = self.create_service(
            String,
            "joystick_mapper/save_mapped_actions",
            self.save_mapped_actions_callback,
        )
        self.get_logger().info(
            BaseLogMessages.SERVICE_CREATED
            + "joystick_mapper/save_mapped_actions"
        )

        self.fetch_status_service = self.create_service(
            FetchStatus,
            "joystick_mapper/fetch_status",
            self.fetch_status_callback,
        )
        self.get_logger().info(
            BaseLogMessages.SERVICE_CREATED + "joystick_mapper/fetch_status"
        )

        self.delete_mapping_service = self.create_service(
            DeleteMapping,
            "joystick_mapper/delete_mapping",
            self.delete_mapping_callback,
        )
        self.get_logger().info(
            BaseLogMessages.SERVICE_CREATED + "joystick_mapper/delete_mapping"
        )

        self.edit_mapping_service = self.create_service(
            SendMapping,
            "joystick_mapper/edit_mapping",
            self.edit_mapping_callback,
        )
        self.get_logger().info(
            BaseLogMessages.SERVICE_CREATED + "joystick_mapper/edit_mapping"
        )

        self.add_mapping_service = self.create_service(
            SendMapping,
            "joystick_mapper/add_mapping",
            self.add_mapping_callback,
        )
        self.get_logger().info(
            BaseLogMessages.SERVICE_CREATED + "joystick_mapper/add_mapping"
        )

        self.view_mappings_service = self.create_service(
            Trigger,
            "joystick_mapper/view_mappings",
            self.view_mappings_callback,
        )
        self.get_logger().info(
            BaseLogMessages.SERVICE_CREATED + "joystick_mapper/view_mappings"
        )

        self.get_logger().info(BaseLogMessages.NODE_READY + "joystick_mapper")

    def js_callback(self, topic: str, msg: Joy) -> None:
        """Stores joystick state and initializes candidates when needed.

        Args:
            topic (str): Joystick topic that produced the message.
            msg (Joy): Latest joystick message.
        """

        if self.current_action_mapping is None:
            return

        self.latest_js_states[topic] = msg
        if self.candidates is None:
            self.create_candidates()

    def set_mapping_state_callback(self, req: TriggerReq, resp: TriggerResp) -> TriggerResp:
        """Toggles the current mapping run state.

        Args:
            req (TriggerReq): Trigger request.
            resp (TriggerResp): Trigger response to populate.

        Returns:
            TriggerResp: Populated service response.
        """

        if self.mapping_active:
            return self.set_mapping_inactive(req, resp)

        return self.set_mapping_active(req, resp)

    def set_action_callback(self, req: ActionReq, resp: ActionResp) -> ActionResp:
        """Sets the current action to be mapped.

        Args:
            req (ActionReq): Request containing action name and type.
            resp (ActionResp): Response to populate.

        Returns:
            ActionResp: Populated service response.
        """

        if self.current_action_mapping is not None:
            msg = (
                JoystickMapperLogMessages.ACTION_SET
                + BaseLogMessages.SERVICE_CALL_FAILED
                + JoystickMapperLogMessages.ACTION_ALREADY_SET
            )
            self.get_logger().warning(msg)
            resp.success = False
            resp.message = msg
            return resp

        try:
            action_type = ROVActionType(req.action_type)
        except ValueError:
            msg = (
                JoystickMapperLogMessages.ACTION_SET
                + BaseLogMessages.SERVICE_CALL_FAILED
                + JoystickMapperLogMessages.ACTION_INVALID
                + req.action_type
            )
            self.get_logger().warning(msg)
            resp.success = False
            resp.message = msg
            return resp

        self.current_action_mapping = ROVActionMapping(
            action_name=req.action_name,
            topic=None,
            action_type=action_type,
            index=None,
        )

        msg = (
            JoystickMapperLogMessages.ACTION_SET
            + BaseLogMessages.SERVICE_CALL_SUCCEEDED
            + JoystickMapperLogMessages.ACTION_IS_SET
            + str(self.current_action_mapping)
        )
        self.get_logger().info(msg)
        resp.success = True
        resp.message = msg
        return resp

    def set_mapper_state_callback(self, req: MapperReq, resp: MapperResp) -> MapperResp:
        """Toggles the joystick mapper activation state.

        Args:
            req (MapperReq): Request containing optional topic overrides.
            resp (MapperResp): Response to populate.

        Returns:
            MapperResp: Populated service response.
        """

        if self.mapper_active:
            return self.set_mapper_inactive(req, resp)

        return self.set_mapper_active(req, resp)

    def delete_mapping_callback(self, req: DeleteReq, resp: DeleteResp) -> DeleteResp:
        """Deletes a saved or unsaved mapping.

        Args:
            req (DeleteReq): Request describing where to delete from.
            resp (DeleteResp): Response to populate.

        Returns:
            DeleteResp: Populated service response.
        """

        deleted = False
        mappings_path = None

        if not req.check_unsaved_mappings and not req.check_mappings_file:
            msg = (
                JoystickMapperLogMessages.DELETE_MAPPING
                + BaseLogMessages.SERVICE_CALL_FAILED
                + "no mapping source selected"
            )
            self.get_logger().warning(msg)
            resp.success = False
            resp.message = msg
            return resp

        if req.check_mappings_file:
            mappings_path = self.resolve_mappings_path(req.mappings_path)
            if mappings_path is None:
                msg = (
                    JoystickMapperLogMessages.DELETE_MAPPING
                    + BaseLogMessages.SERVICE_CALL_FAILED
                    + BaseLogMessages.NO_PATH_PROVIDED
                )
                self.get_logger().warning(msg)
                resp.success = False
                resp.message = msg
                return resp

        if req.check_mappings_file and mappings_path is not None:  # TODO this seems redundant, so ask why?
            try:
                deleted = (
                    self.delete_saved_mapping(
                        req.action_name,
                        mappings_path,
                    )
                    or deleted
                )
            except (
                FileNotFoundError,
                KeyError,
                ValueError,
                OSError,
            ) as exception:
                msg = (
                    JoystickMapperLogMessages.DELETE_MAPPING
                    + BaseLogMessages.SERVICE_CALL_FAILED
                    + BaseLogMessages.SERVER_ERROR
                    + str(exception)
                )
                self.get_logger().error(msg)
                resp.success = False
                resp.message = msg
                return resp

        if req.check_unsaved_mappings:
            deleted = self.delete_unsaved_mapping(req.action_name) or deleted

        if not deleted:
            msg = (
                JoystickMapperLogMessages.DELETE_MAPPING
                + BaseLogMessages.SERVICE_CALL_FAILED
                + f"mapping not found: {req.action_name}"
            )
            self.get_logger().warning(msg)
            resp.success = False
            resp.message = msg
            return resp

        msg = (
            JoystickMapperLogMessages.DELETE_MAPPING
            + BaseLogMessages.SERVICE_CALL_SUCCEEDED
        )
        self.get_logger().info(msg)
        resp.success = True
        resp.message = msg
        return resp

    def save_mapped_actions_callback(self, req: StringReq, resp: StringResp) -> StringResp:
        """Saves unsaved mappings to JSON.

        Args:
            req (StringReq): Request with an optional path override.
            resp (StringResp): Response to populate.

        Returns:
            StringResp: Populated service response.
        """

        mappings_path = self.resolve_mappings_path(req.data)
        if mappings_path is None:
            msg = (
                JoystickMapperLogMessages.SAVE_MAPPED_ACTIONS
                + BaseLogMessages.SERVICE_CALL_FAILED
                + BaseLogMessages.NO_PATH_PROVIDED
            )
            self.get_logger().warning(msg)
            resp.success = False
            resp.message = msg
            return resp

        try:
            existing_json = (
                load_from_json(mappings_path)
                if mappings_path.exists()
                else {}
            )
            existing_actions = self.get_actions_json(existing_json)
        except (FileNotFoundError, ValueError, KeyError) as exception:
            msg = (
                JoystickMapperLogMessages.SAVE_MAPPED_ACTIONS
                + BaseLogMessages.SERVICE_CALL_FAILED
                + BaseLogMessages.SERVER_ERROR
                + str(exception)
            )
            self.get_logger().error(msg)
            resp.success = False
            resp.message = msg
            return resp

        mapped_actions = [mapping.to_json() for mapping in self.js_mappings]
        mapped_action_names = {
            str(action["action_name"]) for action in mapped_actions
        }
        actions = [
            action
            for action in existing_actions
            if str(action["action_name"]) not in mapped_action_names
        ]
        actions.extend(mapped_actions)
        try:
            save_to_json(
                {"actions": actions},
                mappings_path,
            )
        except (ValueError, OSError) as exception:
            msg = (
                JoystickMapperLogMessages.SAVE_MAPPED_ACTIONS
                + BaseLogMessages.SERVICE_CALL_FAILED
                + BaseLogMessages.SERVER_ERROR
                + str(exception)
            )
            self.get_logger().error(msg)
            resp.success = False
            resp.message = msg
            return resp

        self.js_mappings = []
        self.mappings_saved = True

        msg = (
            JoystickMapperLogMessages.SAVE_MAPPED_ACTIONS
            + BaseLogMessages.SERVICE_CALL_SUCCEEDED
            + BaseLogMessages.SAVE_SUCCESSFUL
        )
        self.get_logger().info(msg)
        resp.success = True
        resp.message = msg
        return resp

    def fetch_status_callback(self, req: FetchReq, resp: FetchResp) -> FetchResp:
        """Fetches mapper status for clients.

        Args:
            req (FetchReq): Status request.
            resp (FetchResp): Response to populate.

        Returns:
            FetchResp: Populated service response.
        """

        resp.success = True
        resp.message = str(BaseLogMessages.SERVICE_CALL_SUCCEEDED)
        resp.mapper_active = self.mapper_active
        resp.subscribed_joystick_topics = [
            subscription.topic_name for subscription in self.js_subscriptions
        ]
        resp.mapping_active = self.mapping_active
        resp.mappings_saved = self.mappings_saved
        resp.unsaved_mappings = [str(mapping) for mapping in self.js_mappings]

        if self.current_action_mapping is None:
            resp.current_action_name = ""
            resp.current_action_type = ""
            return resp

        resp.current_action_name = self.current_action_mapping.action_name
        resp.current_action_type = str(self.current_action_mapping.action_type)
        return resp

    def add_mapping_callback(self, req: SendReq, resp: SendResp) -> SendResp:
        """Adds a request-provided mapping to unsaved mappings.

        Args:
            req (SendReq): Request containing mapping data.
            resp (SendResp): Response to populate.

        Returns:
            SendResp: Populated service response.
        """

        try:
            mapping = self.mapping_from_request(req)
        except ValueError:
            msg = (
                JoystickMapperLogMessages.ADD_MAPPING
                + BaseLogMessages.SERVICE_CALL_FAILED
                + JoystickMapperLogMessages.ACTION_INVALID
                + req.action_type
            )
            self.get_logger().warning(msg)
            resp.success = False
            resp.message = msg
            return resp

        self.js_mappings.append(mapping)
        self.mappings_saved = False

        msg = (
            JoystickMapperLogMessages.ADD_MAPPING
            + BaseLogMessages.SERVICE_CALL_SUCCEEDED
            + str(mapping)
        )
        self.get_logger().info(msg)
        resp.success = True
        resp.message = msg
        return resp

    def edit_mapping_callback(self, req: SendReq, resp: SendResp) -> SendResp:
        """Replaces an unsaved mapping with the same action name.

        Args:
            req (SendReq): Request containing replacement mapping data.
            resp (SendResp): Response to populate.

        Returns:
            SendResp: Populated service response.
        """

        try:
            mapping = self.mapping_from_request(req)
        except ValueError:
            msg = (
                JoystickMapperLogMessages.EDIT_MAPPING
                + BaseLogMessages.SERVICE_CALL_FAILED
                + JoystickMapperLogMessages.ACTION_INVALID
                + req.action_type
            )
            self.get_logger().warning(msg)
            resp.success = False
            resp.message = msg
            return resp

        for index, existing_mapping in enumerate(self.js_mappings):
            if existing_mapping.action_name == mapping.action_name:
                self.js_mappings[index] = mapping
                self.mappings_saved = False
                msg = (
                    JoystickMapperLogMessages.EDIT_MAPPING
                    + BaseLogMessages.SERVICE_CALL_SUCCEEDED
                    + str(mapping)
                )
                self.get_logger().info(msg)
                resp.success = True
                resp.message = msg
                return resp

        msg = (
            JoystickMapperLogMessages.EDIT_MAPPING
            + BaseLogMessages.SERVICE_CALL_FAILED
            + f"mapping not found: {mapping.action_name}"
        )
        self.get_logger().warning(msg)
        resp.success = False
        resp.message = msg
        return resp

    def view_mappings_callback(self, req: TriggerReq, resp: TriggerResp) -> TriggerResp:
        """Returns unsaved mappings encoded as response-message JSON.

        Args:
            req (TriggerReq): Trigger request.
            resp (TriggerResp): Response to populate.

        Returns:
            TriggerResp: Populated service response.
        """

        mappings = [mapping.to_json() for mapping in self.js_mappings]
        msg = json.dumps(mappings)
        self.get_logger().info(
            JoystickMapperLogMessages.VIEW_MAPPING
            + BaseLogMessages.SERVICE_CALL_SUCCEEDED
        )
        resp.success = True
        resp.message = msg
        return resp

    def set_mapper_active(self, req: MapperReq, resp: MapperResp) -> MapperResp:
        """Activates the mapper and subscribes to joystick topics.

        Args:
            req (MapperReq): Request containing optional topic overrides.
            resp (MapperResp): Response to populate.

        Returns:
            MapperResp: Populated service response.
        """

        topics = self.resolve_joystick_topics(req.joystick_topics_override)
        if not topics:
            msg = (
                JoystickMapperLogMessages.MAPPER_SET_ACTIVE
                + BaseLogMessages.SERVICE_CALL_FAILED
                + BaseLogMessages.NO_TOPICS
            )
            self.get_logger().warning(msg)
            resp.success = False
            resp.message = msg
            return resp

        self.subscribe_to_joysticks(topics)
        self.mapper_active = True

        msg = (
            JoystickMapperLogMessages.MAPPER_SET_ACTIVE
            + BaseLogMessages.SERVICE_CALL_SUCCEEDED
            + JoystickMapperLogMessages.MAPPER_IS_ACTIVE
        )
        self.get_logger().info(msg)
        resp.success = True
        resp.message = msg
        return resp

    def set_mapper_inactive(self, req: MapperReq, resp: MapperResp) -> MapperResp:
        """Deactivates the mapper after checking mapping state.

        Args:
            req (MapperReq): Mapper state request.
            resp (MapperResp): Response to populate.

        Returns:
            MapperResp: Populated service response.
        """

        if self.mapping_active:
            msg = (
                JoystickMapperLogMessages.MAPPER_SET_INACTIVE
                + BaseLogMessages.SERVICE_CALL_FAILED
                + JoystickMapperLogMessages.MAPPING_IS_ACTIVE
            )
            self.get_logger().warning(msg)
            resp.success = False
            resp.message = msg
            return resp

        self.unsubscribe_from_joysticks()
        self.clear_activation_states()
        self.mapper_active = False

        msg = (
            JoystickMapperLogMessages.MAPPER_SET_INACTIVE
            + BaseLogMessages.SERVICE_CALL_SUCCEEDED
            + JoystickMapperLogMessages.MAPPER_IS_INACTIVE
        )
        self.get_logger().info(msg)
        resp.success = True
        resp.message = msg
        return resp

    def set_mapping_active(self, req: TriggerReq, resp: TriggerResp) -> TriggerResp:
        """Starts a mapping run for the current action.

        Args:
            req (TriggerReq): Trigger request.
            resp (TriggerResp): Response to populate.

        Returns:
            TriggerResp: Populated service response.
        """

        if not self.mapper_active:
            msg = (
                JoystickMapperLogMessages.MAPPING_SET_ACTIVE
                + BaseLogMessages.SERVICE_CALL_FAILED
                + JoystickMapperLogMessages.MAPPER_IS_INACTIVE
            )
            self.get_logger().warning(msg)
            resp.success = False
            resp.message = msg
            return resp

        if self.current_action_mapping is None:
            msg = (
                JoystickMapperLogMessages.MAPPING_SET_ACTIVE
                + BaseLogMessages.SERVICE_CALL_FAILED
                + JoystickMapperLogMessages.NO_ACTION_SET
            )
            self.get_logger().warning(msg)
            resp.success = False
            resp.message = msg
            return resp

        self.start_candidate_updates()
        self.mapping_active = True

        msg = (
            JoystickMapperLogMessages.MAPPING_SET_ACTIVE
            + BaseLogMessages.SERVICE_CALL_SUCCEEDED
            + JoystickMapperLogMessages.MAPPING_IS_ACTIVE
        )
        self.get_logger().info(msg)
        resp.success = True
        resp.message = msg
        return resp

    def set_mapping_inactive(self, req: TriggerReq, resp: TriggerResp) -> TriggerResp:
        """Stops a mapping run and saves the best candidate.

        Args:
            req (TriggerReq): Trigger request.
            resp (TriggerResp): Response to populate.

        Returns:
            TriggerResp: Populated service response.
        """

        self.mapping_active = False
        self.stop_candidate_updates()

        candidate = self.get_best_candidate()
        if candidate == JoystickMapperLogMessages.MAPPING_NO_CANDIDATE:
            msg = (
                JoystickMapperLogMessages.MAPPING_SET_INACTIVE
                + BaseLogMessages.SERVICE_CALL_FAILED
                + JoystickMapperLogMessages.MAPPING_NO_CANDIDATE
            )
            self.clear_current_mapping()
            self.get_logger().warning(msg)
            resp.success = False
            resp.message = msg
            return resp

        if candidate == JoystickMapperLogMessages.MAPPING_CANDIDATE_TIE:
            msg = (
                JoystickMapperLogMessages.MAPPING_SET_INACTIVE
                + BaseLogMessages.SERVICE_CALL_FAILED
                + JoystickMapperLogMessages.MAPPING_CANDIDATE_TIE
            )
            self.clear_current_mapping()
            self.get_logger().warning(msg)
            resp.success = False
            resp.message = msg
            return resp

        if self.current_action_mapping is None:
            msg = (
                JoystickMapperLogMessages.MAPPING_SET_INACTIVE
                + BaseLogMessages.SERVICE_CALL_FAILED
                + JoystickMapperLogMessages.NO_ACTION_SET
            )
            self.clear_current_mapping()
            self.get_logger().error(msg)
            resp.success = False
            resp.message = msg
            return resp

        self.current_action_mapping.topic = candidate.topic
        self.current_action_mapping.index = candidate.index
        self.js_mappings.append(self.current_action_mapping)
        self.mappings_saved = False

        msg = (
            JoystickMapperLogMessages.MAPPING_SET_INACTIVE
            + BaseLogMessages.SERVICE_CALL_SUCCEEDED
            + JoystickMapperLogMessages.MAPPING_SUCCEEDED
            + JoystickMapperLogMessages.MAPPING_FOUND_CANDIDATE
            + f"{self.current_action_mapping.action_name}->"
            + f"{candidate.topic}/"
            + f"{candidate.action_type}/"
            + f"{candidate.index}@{candidate.score_delta}"
        )
        self.get_logger().info(msg)
        resp.success = True
        resp.message = msg

        self.clear_current_mapping()
        return resp

    def clear_current_mapping(self) -> None:
        """Clears current action and candidate state."""

        self.current_action_mapping = None
        self.clear_candidates()

    def clear_candidates(self) -> None:
        """Clears all mapping candidates."""

        self.candidates = None
        self.get_logger().debug("Cleared mapping candidates.")

    def create_candidates(self) -> None:
        """Creates mapping candidates from the latest joystick states."""

        self.candidates = {}

        for topic, js_state in self.latest_js_states.items():
            for index, initial_score in enumerate(js_state.buttons):
                candidate = MappingCandidate(
                    topic=topic,
                    action_type=ROVActionType.BUTTON,
                    index=index,
                    initial_score=float(initial_score),
                )
                self.candidates[candidate.key()] = candidate

            for index, initial_score in enumerate(js_state.axes):
                candidate = MappingCandidate(
                    topic=topic,
                    action_type=ROVActionType.AXIS,
                    index=index,
                    initial_score=float(initial_score),
                )
                self.candidates[candidate.key()] = candidate

        self.get_logger().debug("Created mapping candidates.")

    def update_candidates(self) -> None:
        """Updates candidate scores from the latest joystick states."""

        if self.candidates is None:
            self.get_logger().warning("No candidates available for update.")
            return

        if self.current_action_mapping is None:
            self.get_logger().warning(
                str(JoystickMapperLogMessages.NO_ACTION_SET)
            )
            return

        for candidate in self.candidates.values():
            if (
                candidate.action_type
                != self.current_action_mapping.action_type
            ):
                continue

            js_state = self.latest_js_states.get(candidate.topic)
            if js_state is None:
                self.get_logger().warning(BaseLogMessages.TOPIC_STALE)
                continue

            if candidate.action_type == ROVActionType.AXIS:
                scores = js_state.axes
            else:
                scores = js_state.buttons

            if candidate.index >= len(scores):
                self.get_logger().warning(BaseLogMessages.TOPIC_STALE)
                continue

            candidate.update_score(float(scores[candidate.index]))

    def get_best_candidate(self) -> MappingCandidate | JoystickMapperLogMessages:
        """Returns the candidate with the largest score delta.

        Returns:
            MappingCandidate | JoystickMapperLogMessages: Best candidate or
            failure reason.
        """

        best_candidates: list[MappingCandidate] = []
        best_score: float | None = None

        if self.candidates is None or self.current_action_mapping is None:
            return JoystickMapperLogMessages.MAPPING_NO_CANDIDATE

        for candidate in self.candidates.values():
            if (
                candidate.action_type
                != self.current_action_mapping.action_type
            ):
                continue

            if candidate.score_delta is None:
                self.get_logger().warning(
                    f"Found candidate with no score delta: {candidate} "
                    + BaseLogMessages.TOPIC_STALE
                )
                continue

            if best_score is None or candidate.score_delta > best_score:
                best_candidates = [candidate]
                best_score = candidate.score_delta
                continue

            if candidate.score_delta == best_score:
                best_candidates.append(candidate)

        if len(best_candidates) == 1:
            return best_candidates[0]

        if not best_candidates:
            return JoystickMapperLogMessages.MAPPING_NO_CANDIDATE

        self.get_logger().info(f"Best candidates tied: {best_candidates}")
        return JoystickMapperLogMessages.MAPPING_CANDIDATE_TIE

    def clear_activation_states(self) -> None:
        """Resets mapper activation state after subscriptions are destroyed."""

        self.js_subscriptions = []
        self.latest_js_states = {}
        self.candidates = None
        self.update_timer = None
        self.get_logger().debug("Cleared activation states.")

    def subscribe_to_joysticks(self, topics: list[str]) -> None:
        """Subscribes to joystick topics.

        Args:
            topics (list[str]): Joystick topics to subscribe to.
        """

        for topic in topics:
            subscription = self.create_subscription(
                Joy,
                topic,
                lambda msg, bound_topic=topic: self.js_callback(
                    bound_topic,
                    msg,
                ),
                10,
            )
            self.js_subscriptions.append(subscription)
            self.get_logger().info(
                BaseLogMessages.SUBSCRIPTION_CREATED + topic
            )

    def unsubscribe_from_joysticks(self) -> None:
        """Destroys all active joystick subscriptions."""

        for subscription in self.js_subscriptions:
            self.destroy_subscription(subscription)
            self.get_logger().info(
                BaseLogMessages.SUBSCRIPTION_DESTROYED
                + subscription.topic_name
            )

    def start_candidate_updates(self) -> None:
        """Starts a timer that updates candidate scores."""

        if self.update_timer is not None:
            return

        self.update_timer = self.create_timer(
            self.update_time,
            self.update_candidates,
        )
        self.get_logger().info(
            BaseLogMessages.START_TIMER + "update_candidates"
        )

    def stop_candidate_updates(self) -> None:
        """Stops the candidate update timer if it exists."""

        if self.update_timer is None:
            return

        self.update_timer.cancel()
        self.update_timer = None
        self.get_logger().info(
            BaseLogMessages.STOP_TIMER + "update_candidates"
        )

    def resolve_joystick_topics(self, topics_override: list[str]) -> list[str]:
        """Prefers request topics over configured parameter topics.

        Args:
            topics_override (list[str]): Request-provided topics.

        Returns:
            list[str]: Topics to subscribe to.
        """

        topics = [topic for topic in topics_override if topic]
        if topics:
            return topics

        return [topic for topic in self.js_topics if topic]

    def resolve_mappings_path(self, path_override: str) -> Path | None:
        """Prefers request path over configured parameter path.

        Args:
            path_override (str): Request-provided mappings path.

        Returns:
            Path | None: Resolved mappings path, if one is available.
        """

        if path_override:
            return Path(path_override)

        if self.js_mappings_path:
            return Path(self.js_mappings_path)

        return None

    def mapping_from_request(self, req: SendReq) -> ROVActionMapping:
        """Converts a SendMapping request into an action mapping.

        Args:
            req (SendReq): Mapping request.

        Returns:
            ROVActionMapping: Parsed action mapping.

        Raises:
            ValueError: If `req.action_type` is not a valid ROVActionType.
        """

        return ROVActionMapping(
            action_name=req.action_name,
            topic=req.topic,
            action_type=ROVActionType(req.action_type),
            index=req.index,
        )

    def get_actions_json(self, mappings_json: dict[str, Any]) -> list[dict[str, Any]]:
        """Returns saved action mappings normalized to the JSON shape."""

        actions_json = mappings_json.get("actions", [])
        if not isinstance(actions_json, list):
            raise ValueError("actions must be a list")

        return [
            ROVActionMapping.from_json(action).to_json()  # TODO ask why is the AI doing this?
            for action in actions_json
            if isinstance(action, dict)
        ]

    def load_mappings(self, mappings_path: str | Path | None = None) -> dict[str, Any]:
        """Loads saved mappings from an explicit or configured path.

        Args:
            mappings_path (str | Path | None, optional): Path override.
                Defaults to None.

        Returns:
            dict[str, Any]: Saved mappings.

        Raises:
            ValueError: If no path is available.
            FileNotFoundError: If the mappings file does not exist.
        """

        resolved_path = (
            Path(mappings_path)
            if mappings_path is not None
            else self.resolve_mappings_path("")
        )
        if resolved_path is None:
            raise ValueError(str(BaseLogMessages.NO_PATH_PROVIDED))

        return load_from_json(resolved_path)

    def delete_unsaved_mapping(self, action_name: str) -> bool:
        """Deletes one unsaved mapping by action name.

        Args:
            action_name (str): Action name to delete.

        Returns:
            bool: True when a mapping was deleted.
        """

        for index, mapping in enumerate(self.js_mappings):
            if mapping.action_name == action_name:
                del self.js_mappings[index]
                self.mappings_saved = not self.js_mappings
                self.get_logger().info(
                    "Deleted mapping from unsaved mappings."
                )
                return True

        return False

    def delete_saved_mapping(self, action_name: str, mappings_path: Path) -> bool:
        """Deletes saved mapping entries matching an action name.

        Args:
            action_name (str): Action name to delete.
            mappings_path (Path): Saved mappings path.

        Returns:
            bool: True when at least one mapping was deleted.

        Raises:
            FileNotFoundError: If the mappings file does not exist.
            ValueError: If the mappings file contains invalid JSON.
            OSError: If the mappings file cannot be written.
        """

        mappings_json = self.load_mappings(mappings_path)
        actions = self.get_actions_json(mappings_json)
        remaining_actions = [
            action
            for action in actions
            if action.get("action_name") != action_name
        ]

        if len(remaining_actions) == len(actions):
            return False

        mappings_json["actions"] = remaining_actions
        save_to_json(mappings_json, mappings_path, overwrite=True)
        self.get_logger().info("Deleted mapping from saved mappings.")
        return True


def main(args: list[str] | None = None) -> None:
    """Runs the joystick mapper node.

    Args:
        args (list[str] | None, optional): ROS command-line arguments.
            Defaults to None.
    """

    node: JoystickMapper | None = None

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
