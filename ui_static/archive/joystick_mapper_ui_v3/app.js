const endpoints = {
  mapperState: "/joystick_mapper/set_mapper_state",
  setAction: "/joystick_mapper/set_action",
  mappingState: "/joystick_mapper/set_mapping_state",
  status: "/joystick_mapper/status",
};

const state = {
  mapperActive: false,
  mappingActive: false,
  saveState: "saved",
  currentTopics: [],
  currentAction: null,
  queue: [],
  completedMappings: [],
};

const elements = {
  globalStatus: document.getElementById("global-status"),
  refreshBtn: document.getElementById("refresh-btn"),
  mapperToggleBtn: document.getElementById("mapper-toggle-btn"),
  mappingToggleBtn: document.getElementById("mapping-toggle-btn"),
  saveMappingsBtn: document.getElementById("save-mappings-btn"),
  topicInput: document.getElementById("topic-input"),
  savePathInput: document.getElementById("save-path-input"),
  actionPreset: document.getElementById("action-preset"),
  actionName: document.getElementById("action-name"),
  actionType: document.getElementById("action-type"),
  actionPreview: document.getElementById("action-preview"),
  queueList: document.getElementById("queue-list"),
  queueCount: document.getElementById("queue-count"),
  completedList: document.getElementById("completed-list"),
  completedCount: document.getElementById("completed-count"),
  consoleLog: document.getElementById("console-log"),
  statusMapper: document.getElementById("status-mapper"),
  statusMapping: document.getElementById("status-mapping"),
  statusTopics: document.getElementById("status-topics"),
  statusCurrentAction: document.getElementById("status-current-action"),
  statusSave: document.getElementById("status-save"),
};

/**
 * Collect joystick topics from the activation textarea.
 *
 * @returns {string[]} A list of non-empty topic names entered by the user.
 */
function getTopics() {
  return elements.topicInput.value
    .split(/\n|,/)
    .map((topic) => topic.trim())
    .filter(Boolean);
}

/**
 * Build the current action payload from the Action Wizard inputs.
 *
 * @returns {{name: string, type: string, actionString: string}} The normalized
 * action fields plus a display-friendly `name/type` string.
 */
function buildActionPayload() {
  const name = elements.actionName.value.trim();
  const type = elements.actionType.value;
  return {
    name,
    type,
    actionString: name ? `${name}/${type}` : `myAction/${type}`,
  };
}

/**
 * Update the top-right request status message.
 *
 * @param {string} message The status text to show.
 * @param {"info"|"success"|"warning"|"danger"} [level="info"] The color theme
 * used for the status text.
 * @returns {void}
 */
function setGlobalStatus(message, level = "info") {
  elements.globalStatus.textContent = message;
  const palette = {
    info: "#4aa8ff",
    success: "#78c7ff",
    warning: "#f5c84b",
    danger: "#ff5a67",
  };
  elements.globalStatus.style.color = palette[level] || palette.info;
}

/**
 * Add a request or response entry to the on-page console log.
 *
 * @param {string} direction A label such as `REQUEST` or `RESPONSE`.
 * @param {string} url The endpoint associated with the log entry.
 * @param {unknown} payload The request payload to display, if any.
 * @param {string} [resultText] Optional response summary text.
 * @returns {void}
 */
function logRequest(direction, url, payload, resultText) {
  const line = document.createElement("div");
  line.className = "console-line";
  line.innerHTML = `<strong>${direction}</strong> ${url}\n${payload ? JSON.stringify(payload, null, 2) : ""}${resultText ? `\n${resultText}` : ""}`;
  elements.consoleLog.prepend(line);
}

/**
 * Limit the topics string shown in the status strip so it stays compact.
 *
 * @param {string[]} topics The currently subscribed joystick topics.
 * @returns {string} A comma-separated topic string capped at 32 characters.
 */
function truncateTopics(topics) {
  const joined = topics.join(", ");
  return joined.length > 32 ? `${joined.slice(0, 29)}...` : (joined || "none");
}

/**
 * Re-render the mapper, mapping, topic, action, and save-state status boxes.
 *
 * @returns {void}
 */
function updateStatusStrip() {
  elements.statusMapper.textContent = state.mapperActive ? "active" : "inactive";
  elements.statusMapping.textContent = state.mappingActive ? "active" : "inactive";
  elements.statusTopics.textContent = truncateTopics(state.currentTopics);
  elements.statusCurrentAction.textContent = state.currentAction || "none";
  elements.statusSave.textContent = state.saveState;
}

/**
 * Update the activation and mapping toggle button labels and colors to match
 * the current frontend state.
 *
 * @returns {void}
 */
function updateButtons() {
  elements.mapperToggleBtn.textContent = state.mapperActive ? "Deactivate" : "Activate";
  elements.mapperToggleBtn.className = `btn btn-block ${state.mapperActive ? "btn-danger" : "btn-success"}`;

  elements.mappingToggleBtn.textContent = state.mappingActive ? "Stop Mapping" : "Start Mapping";
  elements.mappingToggleBtn.className = `btn btn-block ${state.mappingActive ? "btn-danger" : "btn-success"}`;
}

/**
 * Refresh the inline action preview string shown in the Action Wizard.
 *
 * @returns {void}
 */
function updateActionPreview() {
  const payload = buildActionPayload();
  elements.actionPreview.textContent = payload.actionString;
}

/**
 * Render the queued actions list from the frontend-only queue state.
 *
 * @returns {void}
 */
function renderQueue() {
  elements.queueCount.textContent = `${state.queue.length} queued`;

  if (!state.queue.length) {
    elements.queueList.innerHTML = '<div class="queue-empty">No queued actions yet.</div>';
    return;
  }

  elements.queueList.innerHTML = state.queue
    .map((action, index) => `
      <div class="queue-item">
        <div class="queue-item-title">${index + 1}. ${action.actionString}</div>
      </div>
    `)
    .join("");
}

/**
 * Render the completed mappings list shown in the Mapping Wizard.
 *
 * @returns {void}
 */
function renderCompletedMappings() {
  elements.completedCount.textContent = `${state.completedMappings.length} completed`;

  if (!state.completedMappings.length) {
    elements.completedList.innerHTML = '<div class="queue-empty">No completed mappings yet.</div>';
    return;
  }

  elements.completedList.innerHTML = state.completedMappings
    .map((mapping, index) => `
      <div class="queue-item">
        <div class="queue-item-title">${index + 1}. ${mapping.action}</div>
        <div class="queue-item-body">${mapping.message}</div>
      </div>
    `)
    .join("");
}

/**
 * Send a JSON HTTP request and log both the request and response.
 *
 * @param {string} url The endpoint to call.
 * @param {unknown} payload The JSON payload to send. Ignored for GET requests.
 * @param {string} [method="POST"] The HTTP method to use.
 * @returns {Promise<object>} The parsed JSON response body.
 * @throws {Error} When the response is not OK.
 */
async function sendJson(url, payload, method = "POST") {
  logRequest("REQUEST", url, payload);

  const response = await fetch(url, {
    method,
    headers: method === "GET" ? undefined : { "Content-Type": "application/json" },
    body: method === "GET" ? undefined : JSON.stringify(payload),
  });

  const data = await response.json().catch(() => ({}));
  const resultText = `HTTP ${response.status}\n${JSON.stringify(data, null, 2)}`;
  logRequest("RESPONSE", url, null, resultText);

  if (!response.ok) {
    const message = data.message || data.error || `Request failed with status ${response.status}`;
    throw new Error(message);
  }

  return data;
}

/**
 * Apply a status response from the server to local UI state.
 *
 * @param {{
 *   isMapperActive?: boolean,
 *   isMappingActive?: boolean,
 *   subscribedTopics?: string[],
 *   currentAction?: string | null,
 *   saveState?: string
 * }} data The JSON status payload returned by the server.
 * @returns {void}
 */
function applyStatusResponse(data) {
  state.mapperActive = Boolean(data.isMapperActive);
  state.mappingActive = Boolean(data.isMappingActive);
  state.currentTopics = data.subscribedTopics || [];
  state.currentAction = data.currentAction || null;
  state.saveState = data.saveState || state.saveState;
  updateStatusStrip();
  updateButtons();
}

/**
 * Fetch the latest mapper status from the server.
 *
 * @returns {Promise<void>}
 */
async function refreshStatus() {
  try {
    const data = await sendJson(endpoints.status, null, "GET");
    applyStatusResponse(data);
    setGlobalStatus("STATUS REFRESH OK", "success");
  } catch (error) {
    setGlobalStatus(error.message.slice(0, 50), "danger");
  }
}

/**
 * Request that the server activate or deactivate the mapper.
 *
 * @param {"active"|"inactive"} nextState The desired mapper state.
 * @returns {Promise<void>}
 * @throws {Error} When activation is requested without topics.
 */
async function setMapperState(nextState) {
  const topics = nextState === "active" ? getTopics() : [];

  if (nextState === "active" && !topics.length) {
    throw new Error("At least one topic is required.");
  }

  const payload = {
    setMapperState: nextState,
    topics,
  };

  const data = await sendJson(endpoints.mapperState, payload);
  if (data && typeof data === "object" && "isMapperActive" in data) {
    applyStatusResponse(data);
  } else {
    state.mapperActive = nextState === "active";
    state.currentTopics = topics;
    updateStatusStrip();
    updateButtons();
  }

  setGlobalStatus(`MAPPER ${nextState.toUpperCase()} REQUEST OK`, "success");
}

/**
 * Send the current action to the server so the mapper knows what to map next.
 *
 * @param {{name: string, type: string, actionString: string}} action The action
 * selected in the Action Wizard or popped from the queue.
 * @returns {Promise<void>}
 */
async function setAction(action) {
  const payload = {
    action_name: action.name,
    action_type: action.type,
  };

  await sendJson(endpoints.setAction, payload);
  state.currentAction = action.actionString;
  state.saveState = "unsaved";
  updateStatusStrip();
  setGlobalStatus("ACTION SET OK", "success");
}

/**
 * Request that the server start or stop the mapping process.
 *
 * @param {"active"|"inactive"} nextState The desired mapping state.
 * @returns {Promise<void>}
 */
async function setMappingState(nextState) {
  const payload = {
    setMappingState: nextState,
  };

  const data = await sendJson(endpoints.mappingState, payload);

  if (data && typeof data === "object" && "isMappingActive" in data) {
    applyStatusResponse(data);
  } else {
    state.mappingActive = nextState === "active";
    updateStatusStrip();
    updateButtons();
  }

  if (nextState === "inactive" && state.currentAction) {
    state.completedMappings.unshift({
      action: state.currentAction,
      message: data.message || "Mapping completed successfully.",
    });
    state.saveState = "unsaved";
    renderCompletedMappings();
    updateStatusStrip();
  }

  setGlobalStatus(`MAPPING ${nextState.toUpperCase()} REQUEST OK`, "success");
}

/**
 * Add the currently configured action to the frontend-only queue.
 *
 * @returns {void}
 * @throws {Error} When the action name is empty.
 */
function addToQueue() {
  const action = buildActionPayload();
  if (!action.name) {
    throw new Error("Action name is required.");
  }

  state.queue.push(action);
  renderQueue();
  setGlobalStatus("ACTION QUEUED", "success");
}

/**
 * Immediately send the currently configured action to the server.
 *
 * @returns {Promise<void>}
 * @throws {Error} When the action name is empty.
 */
async function mapNow() {
  const action = buildActionPayload();
  if (!action.name) {
    throw new Error("Action name is required.");
  }

  await setAction(action);
}

/**
 * Pop the next queued action and send it to the server.
 *
 * @returns {Promise<void>}
 * @throws {Error} When the queue is empty.
 */
async function mapNext() {
  if (!state.queue.length) {
    throw new Error("No queued actions available.");
  }

  const nextAction = state.queue.shift();
  renderQueue();
  await setAction(nextAction);
}

/**
 * Clear the completed mappings list and mark the save state as saved.
 *
 * @returns {void}
 */
function saveCompletedMappings() {
  state.completedMappings = [];
  state.saveState = "saved";
  renderCompletedMappings();
  updateStatusStrip();
  setGlobalStatus("COMPLETED LIST CLEARED", "warning");
}

/**
 * Attach all UI event handlers for the V3 page.
 *
 * @returns {void}
 */
function bindEvents() {
  elements.actionPreset.addEventListener("change", (event) => {
    if (!event.target.value) {
      updateActionPreview();
      return;
    }

    const [name, type] = event.target.value.split("|");
    elements.actionName.value = name;
    elements.actionType.value = type;
    updateActionPreview();
  });

  elements.actionName.addEventListener("input", updateActionPreview);
  elements.actionType.addEventListener("change", updateActionPreview);

  elements.refreshBtn.addEventListener("click", refreshStatus);

  elements.mapperToggleBtn.addEventListener("click", async () => {
    try {
      await setMapperState(state.mapperActive ? "inactive" : "active");
    } catch (error) {
      setGlobalStatus(error.message.slice(0, 50), "danger");
    }
  });

  elements.mappingToggleBtn.addEventListener("click", async () => {
    try {
      await setMappingState(state.mappingActive ? "inactive" : "active");
    } catch (error) {
      setGlobalStatus(error.message.slice(0, 50), "danger");
    }
  });

  document.getElementById("queue-add-btn").addEventListener("click", () => {
    try {
      addToQueue();
    } catch (error) {
      setGlobalStatus(error.message.slice(0, 50), "danger");
    }
  });

  document.getElementById("map-now-btn").addEventListener("click", async () => {
    try {
      await mapNow();
    } catch (error) {
      setGlobalStatus(error.message.slice(0, 50), "danger");
    }
  });

  document.getElementById("map-next-btn").addEventListener("click", async () => {
    try {
      await mapNext();
    } catch (error) {
      setGlobalStatus(error.message.slice(0, 50), "danger");
    }
  });

  elements.saveMappingsBtn.addEventListener("click", saveCompletedMappings);

  document.getElementById("clear-log-btn").addEventListener("click", () => {
    elements.consoleLog.innerHTML = "";
  });
}

updateActionPreview();
updateStatusStrip();
updateButtons();
renderQueue();
renderCompletedMappings();
bindEvents();
