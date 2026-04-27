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

function getTopics() {
  return elements.topicInput.value
    .split(/\n|,/)
    .map((topic) => topic.trim())
    .filter(Boolean);
}

function buildActionPayload() {
  const name = elements.actionName.value.trim();
  const type = elements.actionType.value;
  return {
    name,
    type,
    actionString: name ? `${name}/${type}` : `myAction/${type}`,
  };
}

function setGlobalStatus(message, level = "info") {
  elements.globalStatus.textContent = message;
  const palette = {
    info: "#00e5ff",
    success: "#37d67a",
    warning: "#ffb000",
    danger: "#ff3b5c",
  };
  elements.globalStatus.style.color = palette[level] || palette.info;
}

function logRequest(direction, url, payload, resultText) {
  const line = document.createElement("div");
  line.className = "console-line";
  line.innerHTML = `<strong>${direction}</strong> ${url}\n${payload ? JSON.stringify(payload, null, 2) : ""}${resultText ? `\n${resultText}` : ""}`;
  elements.consoleLog.prepend(line);
}

function truncateTopics(topics) {
  const joined = topics.join(", ");
  return joined.length > 32 ? `${joined.slice(0, 29)}...` : (joined || "none");
}

function updateStatusStrip() {
  elements.statusMapper.textContent = state.mapperActive ? "active" : "inactive";
  elements.statusMapping.textContent = state.mappingActive ? "active" : "inactive";
  elements.statusTopics.textContent = truncateTopics(state.currentTopics);
  elements.statusCurrentAction.textContent = state.currentAction || "none";
  elements.statusSave.textContent = state.saveState;
}

function updateButtons() {
  elements.mapperToggleBtn.textContent = state.mapperActive ? "Deactivate" : "Activate";
  elements.mapperToggleBtn.className = `btn btn-block ${state.mapperActive ? "btn-danger" : "btn-success"}`;

  elements.mappingToggleBtn.textContent = state.mappingActive ? "Stop Mapping" : "Start Mapping";
  elements.mappingToggleBtn.className = `btn btn-block ${state.mappingActive ? "btn-danger" : "btn-success"}`;
}

function updateActionPreview() {
  const payload = buildActionPayload();
  elements.actionPreview.textContent = payload.actionString;
}

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

function applyStatusResponse(data) {
  state.mapperActive = Boolean(data.isMapperActive);
  state.mappingActive = Boolean(data.isMappingActive);
  state.currentTopics = data.subscribedTopics || [];
  state.currentAction = data.currentAction || null;
  state.saveState = data.saveState || state.saveState;
  updateStatusStrip();
  updateButtons();
}

async function refreshStatus() {
  try {
    const data = await sendJson(endpoints.status, null, "GET");
    applyStatusResponse(data);
    setGlobalStatus("STATUS REFRESH OK", "success");
  } catch (error) {
    setGlobalStatus(error.message.slice(0, 50), "danger");
  }
}

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

function addToQueue() {
  const action = buildActionPayload();
  if (!action.name) {
    throw new Error("Action name is required.");
  }

  state.queue.push(action);
  renderQueue();
  setGlobalStatus("ACTION QUEUED", "success");
}

async function mapNow() {
  const action = buildActionPayload();
  if (!action.name) {
    throw new Error("Action name is required.");
  }

  await setAction(action);
}

async function mapNext() {
  if (!state.queue.length) {
    throw new Error("No queued actions available.");
  }

  const nextAction = state.queue.shift();
  renderQueue();
  await setAction(nextAction);
}

function saveCompletedMappings() {
  state.completedMappings = [];
  state.saveState = "saved";
  renderCompletedMappings();
  updateStatusStrip();
  setGlobalStatus("COMPLETED LIST CLEARED", "warning");
}

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
