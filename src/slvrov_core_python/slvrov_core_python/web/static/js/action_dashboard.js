const defaultActions = JSON.parse(
  document.getElementById("default-actions-data").textContent,
);

const endpoints = {
  addActions: "/action_crud/add_actions",
  deleteAction: "/action_crud/delete_action",
  deleteFile: "/action_crud/delete_action_file",
  files: "/action_crud/get_action_files",
  load: "/action_crud/load_actions",
  merge: "/action_crud/merge_action_files",
  rename: "/action_crud/rename_action_file",
};

const state = {
  actions: [],
  currentFile: "",
  files: [],
  lastRoute: "None",
  lastSelectedFileIndex: -1,
  mergeFiles: new Set(),
};

const els = {
  actionCount: document.getElementById("action-count"),
  actionFile: document.getElementById("action-file"),
  actionPreset: document.getElementById("action-preset"),
  actionRows: document.getElementById("action-rows"),
  addAction: document.getElementById("add-action"),
  clearLog: document.getElementById("clear-log"),
  deleteFile: document.getElementById("delete-file"),
  editorTitle: document.getElementById("editor-title"),
  emptyState: document.getElementById("empty-state"),
  fileList: document.getElementById("file-list"),
  lastRoute: document.getElementById("last-route"),
  loadFile: document.getElementById("load-file"),
  mergeFiles: document.getElementById("merge-files"),
  mergedFile: document.getElementById("merged-file"),
  newFile: document.getElementById("new-file"),
  refreshFiles: document.getElementById("refresh-files"),
  renameCurrent: document.getElementById("rename-current"),
  renameFile: document.getElementById("rename-file"),
  responseLog: document.getElementById("response-log"),
  saveActions: document.getElementById("save-actions"),
  statusBanner: document.getElementById("status-banner"),
  themeToggle: document.getElementById("theme-toggle"),
  validationCount: document.getElementById("validation-count"),
};

function setTheme(theme) {
  document.body.dataset.theme = theme;
  els.themeToggle.textContent = theme === "dark" ? "Light Mode" : "Dark Mode";
  localStorage.setItem("sel-action-dashboard-theme", theme);
}

function setStatus(tone, label, message) {
  els.statusBanner.className = `notification ${tone}`;
  els.statusBanner.replaceChildren();
  const labelElement = document.createElement("strong");
  const messageElement = document.createElement("span");
  labelElement.textContent = label;
  messageElement.textContent = message;
  els.statusBanner.append(labelElement, messageElement);
}

function writeLog(payload) {
  els.responseLog.textContent = JSON.stringify(payload, null, 2);
}

function escapeHtml(value) {
  return String(value).replace(/[&<>"']/g, (character) => ({
    "&": "&amp;",
    "<": "&lt;",
    ">": "&gt;",
    "\"": "&quot;",
    "'": "&#39;",
  }[character]));
}

function normalizeFileName(value) {
  const trimmed = value.trim();
  if (!trimmed) {
    return "";
  }
  return trimmed.endsWith(".json") ? trimmed : `${trimmed}.json`;
}

function validateFileName(fileName) {
  if (!fileName) {
    return "Choose or enter an action file.";
  }
  if (fileName.includes("/") || fileName.includes("\\") || fileName.includes("..")) {
    return "Use a filename only, without folders.";
  }
  return "";
}

function defaultKey(action) {
  return `${action.action_name}:${action.action_type}`;
}

function isPredefinedAction(action) {
  return defaultActions.some((option) => defaultKey(option) === defaultKey(action));
}

function currentFileName() {
  return normalizeFileName(els.actionFile.value || state.currentFile);
}

async function apiRequest(route, options = {}) {
  state.lastRoute = route;
  els.lastRoute.textContent = route.replace("/action_crud/", "");
  const response = await fetch(route, {
    headers: {
      "Content-Type": "application/json",
      ...(options.headers || {}),
    },
    ...options,
  });
  const payload = await response.json().catch(() => ({
    success: false,
    message: `HTTP ${response.status}`,
  }));
  writeLog({ route, status: response.status, ...payload });
  if (!response.ok || payload.success === false) {
    throw new Error(payload.message || `Request failed with HTTP ${response.status}`);
  }
  return payload;
}

function renderFiles() {
  els.fileList.innerHTML = state.files.length
    ? state.files.map((fileName, index) => `
      <div class="file-row ${state.mergeFiles.has(fileName) ? "is-selected" : ""} ${fileName === state.currentFile ? "is-loaded" : ""}">
        <button type="button" data-file-name="${escapeHtml(fileName)}" data-file-index="${index}">
          <span>${escapeHtml(fileName)}</span>
        </button>
      </div>
    `).join("")
    : '<div class="empty-state">No saved files found.</div>';
}

function selectFile(fileName, index, event) {
  if (event.shiftKey && state.lastSelectedFileIndex >= 0) {
    const start = Math.min(state.lastSelectedFileIndex, index);
    const end = Math.max(state.lastSelectedFileIndex, index);
    state.mergeFiles.clear();
    state.files.slice(start, end + 1).forEach((selectedFile) => {
      state.mergeFiles.add(selectedFile);
    });
  } else if (event.metaKey || event.ctrlKey) {
    if (state.mergeFiles.has(fileName)) {
      state.mergeFiles.delete(fileName);
    } else {
      state.mergeFiles.add(fileName);
    }
    state.lastSelectedFileIndex = index;
  } else {
    state.mergeFiles.clear();
    state.mergeFiles.add(fileName);
    state.lastSelectedFileIndex = index;
  }

  els.actionFile.value = fileName;
  renderFiles();
}

function renderPresetOptions() {
  const used = new Set(state.actions.map((action) => action.action_name));
  const available = defaultActions.filter((action) => !used.has(action.action_name));
  els.actionPreset.innerHTML = available.length
    ? available.map((action) => (
      `<option value="${escapeHtml(defaultKey(action))}">${escapeHtml(action.action_name)} (${escapeHtml(action.action_type)})</option>`
    )).join("")
    : '<option value="">All predefined actions added</option>';
  els.addAction.disabled = available.length === 0;
}

function renderActions() {
  els.editorTitle.textContent = state.currentFile || "Unsaved Action File";
  els.actionCount.textContent = String(state.actions.length);

  const invalidCount = state.actions.filter((action) => !isPredefinedAction(action)).length;
  els.validationCount.textContent = invalidCount ? `${invalidCount} invalid` : "OK";
  els.validationCount.style.color = invalidCount ? "var(--error)" : "var(--status)";

  els.emptyState.classList.toggle("is-hidden", state.actions.length > 0);
  els.actionRows.innerHTML = state.actions.map((action, index) => {
    const valid = isPredefinedAction(action);
    return `
      <tr data-index="${index}">
        <td>
          <select data-field="action_name" aria-label="Action name">
            ${defaultActions.map((option) => `
              <option value="${escapeHtml(option.action_name)}" ${option.action_name === action.action_name ? "selected" : ""}>
                ${escapeHtml(option.action_name)}
              </option>
            `).join("")}
            ${valid ? "" : `<option value="${escapeHtml(action.action_name)}" selected>${escapeHtml(action.action_name)}</option>`}
          </select>
        </td>
        <td>
          <select data-field="action_type" aria-label="Action type">
            ${["axis", "button", "js_axis", "js_button"].map((type) => `
              <option value="${type}" ${type === action.action_type ? "selected" : ""}>${type}</option>
            `).join("")}
          </select>
        </td>
        <td>
          <span class="valid-pill ${valid ? "status" : "error"}">${valid ? "Yes" : "No"}</span>
        </td>
        <td class="row-actions">
          <button class="button error" type="button" data-delete-index="${index}">Delete</button>
        </td>
      </tr>
    `;
  }).join("");

  renderPresetOptions();
}

function setCurrentFile(fileName, actions = state.actions) {
  state.currentFile = fileName;
  els.actionFile.value = fileName;
  state.actions = actions.map((action) => ({
    action_name: String(action.action_name || ""),
    action_type: String(action.action_type || ""),
  }));
  renderFiles();
  renderActions();
}

async function refreshFiles() {
  try {
    setStatus("warning", "Warning", "Refreshing action file list.");
    const payload = await apiRequest(endpoints.files);
    state.files = Array.isArray(payload.files) ? payload.files : [];
    renderFiles();
    setStatus("status", "Status", `Found ${state.files.length} action files.`);
  } catch (error) {
    setStatus("error", "Error", error.message);
  }
}

async function loadCurrentFile() {
  const fileName = currentFileName();
  const error = validateFileName(fileName);
  if (error) {
    setStatus("error", "Error", error);
    return;
  }

  try {
    setStatus("warning", "Warning", `Loading ${fileName}.`);
    const query = new URLSearchParams({ action_file: fileName });
    const payload = await apiRequest(`${endpoints.load}?${query.toString()}`);
    const actions = Array.isArray(payload.object) ? payload.object : [];
    setCurrentFile(fileName, actions);
    setStatus("status", "Status", `${fileName} loaded.`);
  } catch (error) {
    setStatus("error", "Error", error.message);
  }
}

function getSaveValidationError() {
  const fileName = currentFileName();
  const fileError = validateFileName(fileName);
  if (fileError) {
    return fileError;
  }

  const invalid = state.actions.find((action) => !isPredefinedAction(action));
  if (invalid) {
    return `${invalid.action_name} is not one of the predefined route actions.`;
  }

  const names = state.actions.map((action) => action.action_name);
  if (new Set(names).size !== names.length) {
    return "Action names must be unique before saving.";
  }

  return "";
}

async function saveActions() {
  const validationError = getSaveValidationError();
  if (validationError) {
    setStatus("error", "Error", validationError);
    return;
  }

  const fileName = currentFileName();
  try {
    setStatus("warning", "Warning", `Saving ${fileName}.`);
    await apiRequest(endpoints.addActions, {
      method: "PUT",
      body: JSON.stringify({
        action_file: fileName,
        actions: state.actions,
      }),
    });
    setCurrentFile(fileName);
    await refreshFiles();
    setStatus("status", "Status", `${fileName} saved.`);
  } catch (error) {
    setStatus("error", "Error", error.message);
  }
}

async function deleteAction(index) {
  const action = state.actions[index];
  const fileName = currentFileName();
  if (!action) {
    return;
  }

  if (fileName !== state.currentFile || !state.files.includes(state.currentFile)) {
    state.actions.splice(index, 1);
    renderActions();
    setStatus("status", "Status", `${action.action_name} removed from draft.`);
    return;
  }

  try {
    setStatus("warning", "Warning", `Deleting ${action.action_name}.`);
    await apiRequest(endpoints.deleteAction, {
      method: "DELETE",
      body: JSON.stringify({
        action_file: fileName,
        action_name: action.action_name,
      }),
    });
    state.actions.splice(index, 1);
    renderActions();
    setStatus("status", "Status", `${action.action_name} deleted.`);
  } catch (error) {
    setStatus("error", "Error", error.message);
  }
}

async function deleteCurrentFile() {
  const fileName = currentFileName();
  const error = validateFileName(fileName);
  if (error) {
    setStatus("error", "Error", error);
    return;
  }

  try {
    setStatus("warning", "Warning", `Deleting ${fileName}.`);
    await apiRequest(endpoints.deleteFile, {
      method: "DELETE",
      body: JSON.stringify({ action_file: fileName }),
    });
    state.mergeFiles.delete(fileName);
    setCurrentFile("", []);
    await refreshFiles();
    setStatus("status", "Status", `${fileName} deleted.`);
  } catch (error) {
    setStatus("error", "Error", error.message);
  }
}

async function renameCurrentFile() {
  const oldFile = currentFileName();
  const newFile = normalizeFileName(els.renameFile.value);
  const error = validateFileName(oldFile) || validateFileName(newFile);
  if (error) {
    setStatus("error", "Error", error);
    return;
  }

  try {
    setStatus("warning", "Warning", `Renaming ${oldFile}.`);
    await apiRequest(endpoints.rename, {
      method: "PATCH",
      body: JSON.stringify({
        old_file: oldFile,
        new_file: newFile,
      }),
    });
    els.renameFile.value = "";
    setCurrentFile(newFile);
    await refreshFiles();
    setStatus("status", "Status", `${oldFile} renamed to ${newFile}.`);
  } catch (error) {
    setStatus("error", "Error", error.message);
  }
}

async function mergeSelectedFiles() {
  const mergedFile = normalizeFileName(els.mergedFile.value);
  const sourceFiles = [...state.mergeFiles];
  const error = validateFileName(mergedFile);
  if (sourceFiles.length < 2) {
    setStatus("error", "Error", "Select at least two source files.");
    return;
  }
  if (error) {
    setStatus("error", "Error", error);
    return;
  }

  try {
    setStatus("warning", "Warning", `Merging ${sourceFiles.length} files.`);
    await apiRequest(endpoints.merge, {
      method: "POST",
      body: JSON.stringify({
        action_files: sourceFiles,
        merged_file: mergedFile,
      }),
    });
    state.mergeFiles.clear();
    els.mergedFile.value = "";
    await refreshFiles();
    await loadNamedFile(mergedFile);
    setStatus("status", "Status", `${mergedFile} created.`);
  } catch (error) {
    setStatus("error", "Error", error.message);
  }
}

async function loadNamedFile(fileName) {
  els.actionFile.value = fileName;
  await loadCurrentFile();
}

function addSelectedAction() {
  const selected = defaultActions.find((action) => defaultKey(action) === els.actionPreset.value);
  if (!selected) {
    setStatus("warning", "Warning", "All predefined actions are already in this file.");
    return;
  }

  state.actions.push({ ...selected });
  renderActions();
  setStatus("status", "Status", `${selected.action_name} added locally.`);
}

function bindEvents() {
  els.themeToggle.addEventListener("click", () => {
    setTheme(document.body.dataset.theme === "dark" ? "light" : "dark");
  });

  els.fileList.addEventListener("click", (event) => {
    const button = event.target.closest("[data-file-name]");
    if (button) {
      selectFile(
        button.dataset.fileName,
        Number(button.dataset.fileIndex),
        event,
      );
    }
  });

  els.fileList.addEventListener("dblclick", (event) => {
    const button = event.target.closest("[data-file-name]");
    if (button) {
      loadNamedFile(button.dataset.fileName);
    }
  });

  els.actionRows.addEventListener("change", (event) => {
    const row = event.target.closest("tr");
    if (!row || !event.target.dataset.field) {
      return;
    }
    state.actions[Number(row.dataset.index)][event.target.dataset.field] = event.target.value;
    renderActions();
  });

  els.actionRows.addEventListener("click", (event) => {
    const button = event.target.closest("[data-delete-index]");
    if (button) {
      deleteAction(Number(button.dataset.deleteIndex));
    }
  });

  els.addAction.addEventListener("click", addSelectedAction);
  els.clearLog.addEventListener("click", (event) => {
    event.preventDefault();
    writeLog({});
  });
  els.deleteFile.addEventListener("click", deleteCurrentFile);
  els.loadFile.addEventListener("click", loadCurrentFile);
  els.mergeFiles.addEventListener("click", mergeSelectedFiles);
  els.newFile.addEventListener("click", () => setCurrentFile(normalizeFileName(els.actionFile.value), []));
  els.refreshFiles.addEventListener("click", refreshFiles);
  els.renameCurrent.addEventListener("click", renameCurrentFile);
  els.saveActions.addEventListener("click", saveActions);
}

function init() {
  setTheme(localStorage.getItem("sel-action-dashboard-theme") || "dark");
  bindEvents();
  renderActions();
  refreshFiles();
}

init();
