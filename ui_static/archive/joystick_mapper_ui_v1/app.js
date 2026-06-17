const endpoints = {
  status: "/joystick_mapper/status",
  activation: "/joystick_mapper/activation",
  setAction: "/joystick_mapper/set_action",
  toggleMapping: "/joystick_mapper/toggle_mapping",
};

const state = {
  pendingMappings: [],
  lastResult: "none",
};

const elements = {
  topicInput: document.getElementById("topic-input"),
  actionPreset: document.getElementById("action-preset"),
  actionName: document.getElementById("action-name"),
  actionType: document.getElementById("action-type"),
  actionPreview: document.getElementById("action-preview"),
  globalStatus: document.getElementById("global-status"),
  mappingIndicator: document.getElementById("mapping-indicator"),
  mappingPhase: document.getElementById("mapping-phase"),
  consoleLog: document.getElementById("console-log"),
  mappingList: document.getElementById("mapping-list"),
  statusBackend: document.getElementById("status-backend"),
  statusActive: document.getElementById("status-active"),
  statusMapping: document.getElementById("status-mapping"),
  statusCurrentAction: document.getElementById("status-current-action"),
  statusTopics: document.getElementById("status-topics"),
  statusLastResult: document.getElementById("status-last-result"),
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

function setMappingStage(active) {
  elements.mappingIndicator.classList.toggle("active", active);
  elements.mappingPhase.textContent = active ? "Mapping in progress" : "Idle";
}

function updateActionPreview() {
  const payload = buildActionPayload();
  elements.actionPreview.textContent = payload.rosServiceData;
}

function updateStatusPanel(status = {}) {
  elements.statusBackend.textContent = status.backend || "reachable";
  elements.statusActive.textContent = String(status.active ?? "unknown");
  elements.statusMapping.textContent = String(status.mapping ?? "unknown");
  elements.statusCurrentAction.textContent = status.current_action || "none";
  elements.statusTopics.textContent = (status.topics || []).join(", ") || "none";
  elements.statusLastResult.textContent = state.lastResult;

  setMappingStage(Boolean(status.mapping));
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
      current_action: "none",
      topics: [],
    });
    setGlobalStatus("BACKEND OFFLINE", "danger");
    state.lastResult = error.message;
    elements.statusLastResult.textContent = state.lastResult;
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
    current_action: data.current_action,
    topics,
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
    current_action: "none",
    topics: [],
  });
  setGlobalStatus("MAPPER INACTIVE", "warning");
}

async function setCurrentAction() {
  const payload = buildActionPayload();

  if (!payload.name) {
    throw new Error("Action name is required.");
  }

  const data = await sendJson(endpoints.setAction, payload);
  state.lastResult = data.message || "Action set.";
  updateStatusPanel({
    backend: "reachable",
    current_action: payload.rosServiceData,
  });
  setGlobalStatus("ACTION STAGED", "success");
}

async function startMapping() {
  const payload = {
    desired_state: "start",
  };

  const data = await sendJson(endpoints.toggleMapping, payload);
  state.lastResult = data.message || "Mapping started.";
  setMappingStage(true);
  updateStatusPanel({
    backend: "reachable",
    mapping: true,
  });
  setGlobalStatus("MAPPING ACTIVE", "success");
}

async function stopMapping() {
  const payload = {
    desired_state: "stop",
  };

  const data = await sendJson(endpoints.toggleMapping, payload);
  state.lastResult = data.message || "Mapping stopped.";
  setMappingStage(false);
  updateStatusPanel({
    backend: "reachable",
    mapping: false,
  });

  if (data.success && data.message) {
    state.pendingMappings.unshift({
      action: elements.actionPreview.textContent,
      message: data.message,
    });
    renderPendingMappings();
  }

  setGlobalStatus(data.success ? "MAPPING COMPLETE" : "MAPPING FAILED", data.success ? "success" : "warning");
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

  document.getElementById("set-action-btn").addEventListener("click", async () => {
    try {
      await setCurrentAction();
    } catch (error) {
      state.lastResult = error.message;
      elements.statusLastResult.textContent = state.lastResult;
      setGlobalStatus("ACTION ERROR", "danger");
    }
  });

  document.getElementById("start-mapping-btn").addEventListener("click", async () => {
    try {
      await startMapping();
    } catch (error) {
      state.lastResult = error.message;
      elements.statusLastResult.textContent = state.lastResult;
      setGlobalStatus("START ERROR", "danger");
    }
  });

  document.getElementById("stop-mapping-btn").addEventListener("click", async () => {
    try {
      await stopMapping();
    } catch (error) {
      state.lastResult = error.message;
      elements.statusLastResult.textContent = state.lastResult;
      setGlobalStatus("STOP ERROR", "danger");
    }
  });

  document.getElementById("clear-log-btn").addEventListener("click", () => {
    elements.consoleLog.innerHTML = "";
  });
}

updateActionPreview();
renderPendingMappings();
bindEvents();
refreshStatus();
