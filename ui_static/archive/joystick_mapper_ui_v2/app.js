const endpoints = {
  status: "/joystick_mapper/status",
  activation: "/joystick_mapper/activation",
  setAction: "/joystick_mapper/set_action",
  queueMapNow: "/joystick_mapper/action_queue/map_now",
  queueMapNext: "/joystick_mapper/action_queue/map_next",
};

const state = {
  pendingMappings: [],
  queuedActions: [],
  lastResult: "none",
};

const elements = {
  topicInput: document.getElementById("topic-input"),
  actionPreset: document.getElementById("action-preset"),
  actionName: document.getElementById("action-name"),
  actionType: document.getElementById("action-type"),
  actionPreview: document.getElementById("action-preview"),
  globalStatus: document.getElementById("global-status"),
  consoleLog: document.getElementById("console-log"),
  mappingList: document.getElementById("mapping-list"),
  queueList: document.getElementById("queue-list"),
  queueCount: document.getElementById("queue-count"),
  statusBackend: document.getElementById("status-backend"),
  statusActive: document.getElementById("status-active"),
  statusMapping: document.getElementById("status-mapping"),
  statusSaved: document.getElementById("status-saved"),
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
  const effectiveName = name || "action_name";

  return {
    name,
    type,
    rosServiceData: `${effectiveName}/${type}/None/None`,
  };
}

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

function logRequest(direction, url, payload, resultText) {
  const line = document.createElement("div");
  line.className = "console-line";
  line.innerHTML = `<strong>${direction}</strong> ${url}\n${payload ? JSON.stringify(payload, null, 2) : ""}${resultText ? `\n${resultText}` : ""}`;
  elements.consoleLog.prepend(line);
}

function updateActionPreview() {
  const payload = buildActionPayload();
  elements.actionPreview.textContent = payload.rosServiceData;
}

function updateStatusPanel(status = {}) {
  elements.statusBackend.textContent = status.backend || "reachable";
  elements.statusActive.textContent = String(status.active ?? "unknown");
  elements.statusMapping.textContent = String(status.mapping ?? "unknown");
  elements.statusSaved.textContent = String(status.saved ?? "unknown");
}

function renderPendingMappings() {
  if (!state.pendingMappings.length) {
    elements.mappingList.innerHTML = '<div class="mapping-empty">No successful mappings recorded in this session.</div>';
    return;
  }

  elements.mappingList.innerHTML = state.pendingMappings
    .map((mapping) => `
      <div class="mapping-item">
        <div class="mapping-item-title">${mapping.action}</div>
        <div class="mapping-item-body">${mapping.message}</div>
      </div>
    `)
    .join("");
}

function renderQueue() {
  elements.queueCount.textContent = `${state.queuedActions.length} queued`;

  if (!state.queuedActions.length) {
    elements.queueList.innerHTML = '<div class="mapping-empty">No queued actions yet.</div>';
    return;
  }

  elements.queueList.innerHTML = state.queuedActions
    .map((item, index) => `
      <div class="queue-item">
        <div class="mapping-item-title">${index + 1}. ${item.name}</div>
        <div class="mapping-item-body">${item.rosServiceData}</div>
      </div>
    `)
    .join("");
}

async function sendJson(url, payload) {
  logRequest("REQUEST", url, payload);

  const response = await fetch(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload),
  });

  const data = await response.json().catch(() => ({}));
  const resultText = `HTTP ${response.status}\n${JSON.stringify(data, null, 2)}`;
  logRequest("RESPONSE", url, null, resultText);

  if (!response.ok) {
    const message = data.message || `Request failed with status ${response.status}`;
    throw new Error(message);
  }

  return data;
}

async function refreshStatus() {
  logRequest("REQUEST", endpoints.status, null);

  try {
    const response = await fetch(endpoints.status);
    const data = await response.json().catch(() => ({}));
    logRequest("RESPONSE", endpoints.status, null, `HTTP ${response.status}\n${JSON.stringify(data, null, 2)}`);

    if (!response.ok) {
      throw new Error(data.message || "Status request failed");
    }

    updateStatusPanel(data);
    setGlobalStatus("STATUS OK", "success");
  } catch (error) {
    updateStatusPanel({
      backend: "offline",
      active: "unknown",
      mapping: "unknown",
      saved: "unknown",
    });
    state.lastResult = error.message;
    setGlobalStatus("BACKEND OFFLINE", "danger");
  }
}

async function activateMapper() {
  const topics = getTopics();

  if (!topics.length) {
    throw new Error("At least one joystick topic is required.");
  }

  const payload = {
    desired_state: "activate",
    topics,
    rosServiceData: topics.join(","),
  };

  const data = await sendJson(endpoints.activation, payload);
  state.lastResult = data.message || "Activation request sent.";
  updateStatusPanel({
    backend: "reachable",
    active: true,
    mapping: false,
    saved: data.saved,
  });
  setGlobalStatus("MAPPER ACTIVE", "success");
}

async function deactivateMapper() {
  const payload = {
    desired_state: "deactivate",
  };

  const data = await sendJson(endpoints.activation, payload);
  state.lastResult = data.message || "Deactivation request sent.";
  updateStatusPanel({
    backend: "reachable",
    active: false,
    mapping: false,
    saved: data.saved,
  });
  setGlobalStatus("MAPPER INACTIVE", "warning");
}

async function addActionToQueue() {
  const payload = buildActionPayload();

  if (!payload.name) {
    throw new Error("Action name is required.");
  }

  state.queuedActions.push(payload);
  renderQueue();
  state.lastResult = `Queued ${payload.rosServiceData}.`;
  setGlobalStatus("ACTION QUEUED", "success");
}

async function mapActionNow() {
  const payload = buildActionPayload();

  if (!payload.name) {
    throw new Error("Action name is required.");
  }

  const request = {
    action: payload,
    desired_state: "start",
    orchestration: ["set_action", "toggle_mapping"],
  };

  const data = await sendJson(endpoints.queueMapNow, request);
  state.lastResult = data.message || "Immediate mapping started.";
  updateStatusPanel({
    backend: "reachable",
    mapping: true,
    saved: data.saved,
  });
  setGlobalStatus("MAP NOW SENT", "success");
}

async function mapNextAction() {
  if (!state.queuedActions.length) {
    throw new Error("No queued actions available.");
  }

  const nextAction = state.queuedActions[0];
  const request = {
    desired_state: "start",
    action: nextAction,
    queue_length: state.queuedActions.length,
    orchestration: ["set_action", "toggle_mapping"],
  };

  const data = await sendJson(endpoints.queueMapNext, request);
  state.lastResult = data.message || "Next queued action sent for mapping.";
  state.queuedActions.shift();
  renderQueue();
  updateStatusPanel({
    backend: "reachable",
    mapping: true,
    saved: data.saved,
  });
  setGlobalStatus("NEXT ACTION SENT", "success");
}

function pushCompletedMapping(actionString, message) {
  state.pendingMappings.unshift({
    action: actionString,
    message,
  });
  renderPendingMappings();
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

  document.getElementById("activate-btn").addEventListener("click", async () => {
    try {
      await activateMapper();
    } catch (error) {
      state.lastResult = error.message;
      elements.statusLastResult.textContent = state.lastResult;
      setGlobalStatus("ACTIVATION ERROR", "danger");
    }
  });

  document.getElementById("deactivate-btn").addEventListener("click", async () => {
    try {
      await deactivateMapper();
    } catch (error) {
      state.lastResult = error.message;
      elements.statusLastResult.textContent = state.lastResult;
      setGlobalStatus("DEACTIVATION ERROR", "danger");
    }
  });

  document.getElementById("refresh-btn").addEventListener("click", refreshStatus);

  document.getElementById("add-action-queue-btn").addEventListener("click", async () => {
    try {
      await addActionToQueue();
    } catch (error) {
      state.lastResult = error.message;
      setGlobalStatus("QUEUE ERROR", "danger");
    }
  });

  document.getElementById("map-action-now-btn").addEventListener("click", async () => {
    try {
      await mapActionNow();
    } catch (error) {
      state.lastResult = error.message;
      setGlobalStatus("MAP NOW ERROR", "danger");
    }
  });

  document.getElementById("map-next-action-btn").addEventListener("click", async () => {
    try {
      await mapNextAction();
    } catch (error) {
      state.lastResult = error.message;
      setGlobalStatus("MAP NEXT ERROR", "danger");
    }
  });

  document.getElementById("clear-log-btn").addEventListener("click", () => {
    elements.consoleLog.innerHTML = "";
  });
}

updateActionPreview();
renderPendingMappings();
renderQueue();
bindEvents();
refreshStatus();

window.joystickMapperV2Demo = {
  pushCompletedMapping,
};
