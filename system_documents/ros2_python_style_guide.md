# ROS 2 Python Style Guide for `slvrov_core_python`

This guide is for Python code in:

- `src/slvrov_core_python/slvrov_core_python`
- nearby ROS 2 Python packages that follow the same patterns

It is based on the current style in `control_objects.py`, `joystick_mapper.py`, and `json_crud.py`. Use it as the target style for new code and as a checklist when cleaning up existing code.

## Goals

- Keep ROS nodes predictable for students and future maintainers.
- Make service callbacks easy to read, test, and debug.
- Keep data objects simple and explicit.
- Prefer clear names, typed fields, and structured error handling over clever shortcuts.
- Keep comments useful without turning code into a diary of old experiments.

## Formatting

Follow PEP 8 and the package linters already present in `src/slvrov_core_python/test`:

- Use 4 spaces for indentation.
- Keep lines readable; prefer wrapping long expressions over very wide lines.
- Put one statement on each line.
- Leave two blank lines between top-level classes and functions.
- Leave one blank line between methods when it improves scanability.
- Avoid trailing whitespace.
- Keep the full function name and argument list on one line when the function has fewer than 7 arguments, counting `self` for methods. For example, write `def name(a, b) -> int:` on one line instead of splitting the arguments across multiple lines.

Prefer this:

```python
if action_type == str(ROVActionType.JS_AXIS):
    instance_type = ROVActionType.JS_AXIS
elif action_type == str(ROVActionType.JS_BUTTON):
    instance_type = ROVActionType.JS_BUTTON
else:
    raise ValueError(f"Invalid action type: {action_type}")
```

Over this:

```python
if action_type == str(ROVActionType.JS_AXIS): instance_type = ROVActionType.JS_AXIS
elif action_type == str(ROVActionType.JS_BUTTON): instance_type = ROVActionType.JS_BUTTON
else: raise ValueError(f"Invalid action type: {action_type}")
```

Short one-line returns are acceptable only when they stay very clear. In callbacks and state-changing code, prefer multi-line blocks.

## Imports

Group imports in this order:

1. Python standard library
2. Third-party packages, including ROS packages
3. Project-local imports

Avoid wildcard imports. They make it harder to see where message types, helper functions, and constants come from.

Prefer this:

```python
from .control_objects import MappingCandidate, ROVActionMapping, ROVActionType
from .json_crud import load_from_json, save_to_json
from .log_messages import BaseLogMessages, JoystickMapperLogMessages
```

Over this:

```python
from .control_objects import *
from .json_crud import *
from .log_messages import *
```

Use `# type: ignore` only when a ROS-generated import is known to confuse static tooling. Do not use it to hide real type problems.

## Naming

Use Python naming conventions consistently:

- Modules and files: `snake_case.py`
- Functions and methods: `snake_case`
- Variables and attributes: `snake_case`
- Classes and dataclasses: `PascalCase`
- Enums: `PascalCase`
- Enum members and constants: `UPPER_SNAKE_CASE`
- ROS topic and service names: lower-case slash-separated strings

Good examples from the current code:

- `ROVActionType`
- `ROVActionMapping`
- `MappingCandidate`
- `save_to_json`
- `load_from_json`
- `mapping_active`
- `current_action_mapping`

Avoid abbreviations when the full word is clearer. For joystick-specific code, `js` is acceptable because it is already established, but do not introduce new short names unless they are common in ROS or the project.

Prefer names that describe meaning, not type only:

- `mapping_path` instead of `filename` when it must be a mappings file
- `candidate_key` instead of `mapping` when iterating dictionary keys
- `response` instead of `resp` in non-callback helper functions

## Data Objects

Use dataclasses for plain data containers, as in `ROVAction`, `JoystickInput`, `ROVActionMapping`, and `MappingCandidate`.

Dataclass guidelines:

- Keep dataclasses free of ROS node side effects.
- Use explicit field types.
- Put required fields before optional/defaulted fields.
- Prefer `None` for "unknown" or "not selected yet"; document when a field can be `None`.
- Use methods such as `from_string`, `from_json`, `to_json`, or `key` for serialization and identity.
- Avoid naming fields after built-in functions when practical. For example, `action_type` is clearer than `type`.

Prefer serialization methods with normal names:

```python
def to_json(self) -> dict[str, object]:
    return {
        f"{self.topic}/{self.action_type}/{self.index}": {
            "action": self.action_name,
            "topic": self.topic,
            "type": str(self.action_type),
            "index": self.index,
        }
    }
```

`__str__` and `__hash__` are useful for display and dictionary keys, but do not hide important parsing or persistence behavior inside magic methods unless there is a strong reason.

## Enums

Use enums for fixed value sets such as joystick action types and log message categories.

Guidelines:

- Enum values should match the external string contract when they are sent over services, saved to JSON, or displayed in the UI.
- Use `ROVActionType(action_type)` when validating incoming strings against the enum.
- Raise `ValueError` with a clear message when a string cannot be converted.
- Keep `__str__` simple and predictable when external code relies on it.

## ROS Nodes

ROS node classes should follow this order:

1. `super().__init__("node_name")`
2. Declare parameters.
3. Read parameters into typed attributes.
4. Initialize state attributes.
5. Create publishers, subscriptions, services, clients, and timers.
6. Log node readiness.

Keep names consistent:

- Node names: `snake_case`
- Services: `"node_name/action_name"`
- Topic names: use parameters where hardware or launch files decide the name
- Callback methods: `thing_callback`
- State helpers: `set_mapper_active`, `set_mapper_inactive`, `clear_candidates`

For service setup, keep the service name and callback close together:

```python
self.set_action_service = self.create_service(
    SetAction,
    "joystick_mapper/set_action",
    self.set_action_callback,
)
self.get_logger().info(BaseLogMessages.SERVICE_CREATED + "joystick_mapper/set_action")
```

## Service Callbacks

Service callbacks should have a consistent shape:

1. Validate request fields.
2. Check node state.
3. Perform the state change or IO.
4. Log success or failure.
5. Set all response fields.
6. Return the response object.

Every callback path should return `resp`. Avoid bare `return` in service callbacks.

Prefer helper functions when a callback has multiple phases. For example, the mapper state callback is easier to read because activation and deactivation live in `set_mapper_active` and `set_mapper_inactive`.

Use clear response messages, but keep service logic independent from string formatting where possible. Build one message per failure or success path and reuse it for both the logger and response.

## State Management

Keep node state transitions explicit. A reader should be able to answer:

- Is the mapper active?
- Is a mapping run active?
- What action is being mapped?
- Are there unsaved mappings?
- Are subscriptions and timers currently alive?

When clearing state:

- Reset every related attribute in one helper.
- Destroy ROS resources before clearing lists that reference them.
- Use one representation for empty state. For example, prefer `[]` for an empty subscription list instead of sometimes using `None`.
- Do not leave stale timers, candidates, or current actions after a failed mapping run.

Use assertions or guard clauses only when they help protect a real invariant. In ROS callbacks, convert expected bad states into service failure responses rather than crashes.

## Error Handling

Catch specific exceptions when possible:

- `ValueError` for invalid enum values or bad request data
- `FileNotFoundError` for missing files
- `json.JSONDecodeError` for invalid JSON files
- `OSError` for filesystem failures

Avoid broad `except Exception` unless the callback must protect the ROS node from an unexpected failure. If broad catching is used, include the exception message in the log and response.

Prefer this:

```python
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
```

For file helpers, let low-level functions raise meaningful exceptions. Service callbacks should decide whether those exceptions become warnings, errors, or failed service responses.

## Logging

Use ROS logging through `self.get_logger()` inside nodes.

Use severity consistently:

- `debug`: high-volume internal values, timer ticks, candidate scoring details
- `info`: node ready, service created, successful state transitions
- `warning`: invalid user input, no topics, no action set, mapping tie, missing optional data
- `error`: unexpected exceptions, failed file writes, broken server state

Keep log messages short and searchable. Use the existing `BaseLogMessages` and `JoystickMapperLogMessages` enums when a message is shared by multiple callbacks.

Fix spelling in shared log strings when touching them, because these strings become part of debugging output. Examples in the current code include `RECIEVED` and `succcessfull`.

## JSON And File IO

Keep JSON helpers small and predictable:

- Accept `str | Path`.
- Convert to `Path` once at the start.
- Use `Path.open()` instead of mixing `open()` and `Path.open()`.
- Validate data before writing.
- Avoid creating empty files before you know the write will succeed.
- Be explicit about merge direction when combining existing and new data.

When saving mappings, decide whether new data should overwrite old data or old data should win. Document that behavior in the function docstring.

Recommended helper names:

- `load_from_json(path: str | Path) -> dict`
- `save_to_json(data: dict, path: str | Path, *, overwrite: bool = False) -> None`
- `update_json_key(path: str | Path, key: str, value: object) -> None`
- `delete_from_json(path: str | Path, keys: list[str]) -> None`

Do not rely on an object having a callable `__dict__`; most Python objects expose `__dict__` as an attribute. Prefer an explicit `to_json()` method or use `dataclasses.asdict()` for dataclasses.

## Type Hints

Use type hints on:

- Public helper functions
- Dataclass fields
- Callback return values when known
- Lists and dictionaries stored as node state

For ROS-generated request and response objects, use `object` only when the generated type is awkward to import. Otherwise, prefer generated request/response types if they are available.

Keep annotations truthful. If a function can return a log-message enum instead of a `MappingCandidate`, the return type should say so or the function should be refactored to return a clearer result object.

## Comments And Docstrings

Use docstrings for public classes, public helpers, and callbacks whose behavior is not obvious.

Docstrings should use Google style:

```python
def fetch_records(api_key: str, retry_limit: int = 3) -> list[dict]:
    """Fetches records from the remote API.

    This function connects to the cloud service and attempts to retrieve
    all active records. If the connection fails, it will retry.

    Args:
        api_key (str): The secret key for authenticating the request.
        retry_limit (int, optional): Max number of attempts. Defaults to 3.

    Returns:
        list: A list of record dictionaries if successful.

    Raises:
        ConnectionError: If the API is unreachable after all retries.
    """

    ...
```

Classes should also use Google-style docstrings with an `Attributes` section and an `Args` section when the class accepts constructor arguments:

```python
class DatabaseConnection:
    """Manages connections to the central PostgreSQL cluster.

    This class handles authentication, connection pooling, and automatic
    reconnection strategies for the application layer.

    Attributes:
        host (str): The network address of the database server.
        port (int): The port number the server listens on.
        is_connected (bool): True if the connection is currently live.

    Args:
        host (str): The network address of the database server.
        port (int, optional): The port number. Defaults to 5432.
    """

    ...
```

Always leave one empty line between a docstring and the first line of code in that scope. This applies to modules, classes, methods, and functions.

Good comments explain why something exists:

```python
# Bind topic at subscription time so each lambda keeps its own topic.
self.create_subscription(
    Joy,
    topic,
    lambda msg, bound_topic=topic: self.js_callback(bound_topic, msg),
    10,
)
```

Avoid comments that repeat the code:

```python
# Saves data
json.dump(json_dict, file, indent=indent)
```

Avoid long dated testing notes in source files. Put test notes, experiments, and manual checklists in `system_documents` or a dedicated issue instead.

Use TODOs only when they describe a concrete next action:

```python
# TODO: Implement loading saved mappings before enabling edit/delete services.
```

Avoid vague TODOs:

```python
# TODO: this will be used later
```

## Testing Expectations

At minimum, code in `slvrov_core_python` should pass:

```bash
colcon test --packages-select slvrov_core_python
colcon test-result --verbose
```

For pure helpers such as JSON utilities and dataclass serialization, prefer normal unit tests that do not require a running ROS graph.

For ROS nodes:

- Test state helpers separately when possible.
- Keep callback tests focused on request validation, state transitions, and response fields.
- Use small fake request/response objects for logic that does not need live ROS middleware.

## Implemented Style Improvements

The original cleanup opportunities below have been implemented in `control_objects.py`, `joystick_mapper.py`, and `json_crud.py`. Keep these as baseline expectations when editing these files or adding nearby code.

### `control_objects.py`

- Dataclass fields use `action_type` or `input_type` instead of shadowing the built-in `type`.
- One-line `if`/`elif`/`else` bodies have been replaced with normal multi-line blocks.
- `ROVActionMapping.from_json()` is implemented for saved mapping entries.
- Mapping serialization uses explicit `to_json()` methods instead of magic `__json__()` methods.
- Hash behavior no longer depends on mutable display strings; immutable value objects are frozen and mutable candidates use explicit keys.
- `InputSource` is a real dataclass used by `JoystickInput`.

### `joystick_mapper.py`

- Imports are explicit and grouped.
- Dated manual testing notes have been removed from the source file.
- Service callbacks and helpers use generated request/response type hints where practical.
- Every service callback path returns the response object.
- Empty state is consistent: subscription lists are lists, candidate state is either a dictionary or `None`, and timers are cleared explicitly.
- Candidate field names now match the dataclass (`index`, `action_type`) and candidate scoring lives in `MappingCandidate.update_score()`.
- Mapping delete logic uses request fields consistently and deletes saved mappings by resolved key.
- Status responses handle the no-current-action state without raising an exception.
- High-volume candidate logs use `debug`, while major state transitions use `info`.
- Add, edit, delete, view, load, candidate update, and timer helpers are implemented instead of placeholder methods.

### `json_crud.py`

- `Path.open()` is used consistently.
- Files are not touched or created before input data is validated.
- `save_to_json()` documents and implements merge behavior: existing data is loaded first, then new data replaces matching keys unless unique-key enforcement is enabled.
- Serialization accepts dictionaries, dataclass instances, and objects with explicit `to_json()` methods.
- `update_json_key()` and `delete_from_json()` are implemented.
- JSON errors are raised with clear exception types and messages so ROS service callbacks can convert them into failed responses.

## Quick Review Checklist

Before opening a PR or sharing code:

- Imports are explicit and grouped.
- Names follow Python and ROS conventions.
- Callbacks validate input before changing state.
- Every service callback returns a response.
- Function signatures stay on one line when they have fewer than 7 arguments.
- Docstrings use Google style and have one empty line before code begins.
- Class docstrings include `Attributes` and constructor `Args` where applicable.
- Expected user errors produce warnings and failed responses.
- Unexpected exceptions produce errors and failed responses.
- JSON helpers have clear overwrite/merge behavior.
- Comments explain why, not what.
- TODOs are specific.
- `ament_flake8` and `ament_pep257` pass for the package.
