const styles = [
  {
    id: "apple",
    title: "Apple/macOS-inspired",
    summary: "Polished, calm, and familiar with subtle depth and native-control feel.",
    status: "Owner decision pending",
    tradeoff: "Best for confidence and readability. It is less distinctive than a control-station look, and it may need extra restraint so safety controls still feel urgent.",
  },
  {
    id: "station",
    title: "Existing project style",
    summary: "Dark control-station feel based on the existing joystick mapper and UI examples.",
    status: "Owner decision pending",
    tradeoff: "Best continuity with current prototypes and technical operators. It can feel dense quickly, so labels and alert hierarchy need careful limits.",
  },
  {
    id: "hybrid",
    title: "Hybrid style",
    summary: "Combines the existing dark station look with calmer spacing and clearer browser controls.",
    status: "Owner decision pending",
    tradeoff: "Best bridge from the current look toward a maintainable MVP. It keeps ROV character while reducing visual noise.",
  },
  {
    id: "recommended",
    title: "Agent-recommended style",
    summary: "Quiet operator console with strong safety hierarchy, camera-first Pilot, and dense Developer views only where useful.",
    status: "Recommended default only",
    tradeoff: "Recommended default: use this as the base, borrow the macOS control clarity, and keep a lighter version of the existing station identity for technical screens.",
    recommended: true,
  },
];

const screens = [
  { id: "main", title: "Main" },
  { id: "safety", title: "Safety" },
  { id: "pilot", title: "Pilot" },
  { id: "setup", title: "Setup" },
  { id: "developer", title: "Developer" },
];

const mock = {
  cameras: [
    { id: "CAM 01", label: "Forward", state: "Live", detail: "42 ms" },
    { id: "CAM 02", label: "Manipulator", state: "Live", detail: "48 ms" },
    { id: "CAM 03", label: "Rear", state: "Offline", detail: "No signal", offline: true },
    { id: "CAM 04", label: "Port", state: "Live", detail: "51 ms" },
    { id: "CAM 05", label: "Starboard", state: "Live", detail: "46 ms" },
    { id: "CAM 06", label: "Down", state: "Empty", detail: "Not assigned", offline: true },
  ],
  nodes: [
    ["web_ros_bridge", "Healthy", "42 ms"],
    ["safety_monitor", "Fault", "E-stop active"],
    ["preflight_test", "Warning", "Thruster test skipped"],
    ["camera_supervisor", "Offline", "Rear camera missing"],
  ],
};

let currentStyle = "apple";
let currentScreen = "main";

const styleList = document.getElementById("style-list");
const screenTabs = document.getElementById("screen-tabs");
const previewTitle = document.getElementById("preview-title");
const styleStatus = document.getElementById("style-status");
const styleKicker = document.getElementById("style-kicker");
const tradeoffNote = document.getElementById("tradeoff-note");
const mockShell = document.getElementById("mock-shell");

function currentStyleData() {
  return styles.find((style) => style.id === currentStyle);
}

function renderSelectors() {
  styleList.innerHTML = styles.map((style) => `
    <button class="style-button ${style.id === currentStyle ? "is-active" : ""} ${style.recommended ? "is-recommended" : ""}" type="button" data-style-id="${style.id}">
      <strong>${style.title}</strong>
      <span>${style.summary}</span>
    </button>
  `).join("");

  screenTabs.innerHTML = screens.map((screen) => `
    <button class="screen-button ${screen.id === currentScreen ? "is-active" : ""}" type="button" data-screen-id="${screen.id}">
      ${screen.title}
    </button>
  `).join("");
}

function appChrome(content) {
  return `
    <div class="mock-app">
      <header class="mock-topbar">
        <div class="mock-brand">
          <span class="brand-mark">ROV</span>
          <div>
            <h3>SLVROV Control</h3>
            <p class="mock-small">Static fixture data only</p>
          </div>
        </div>
        <div class="mock-actions">
          <button class="mock-button success" type="button">Controls Disabled</button>
          <button class="mock-button danger" type="button">Emergency Stop</button>
        </div>
      </header>
      <section class="mock-alert">
        <strong>Critical alert</strong>
        <span>Rear camera offline and emergency stop active in mock state.</span>
      </section>
      ${content}
    </div>
  `;
}

function renderMainScreen() {
  return appChrome(`
    <section class="mock-body">
      <div class="component-strip" aria-label="Button state examples">
        <button class="mock-button primary" type="button">Normal</button>
        <button class="mock-button primary is-pressed" type="button">Pressed</button>
        <button class="mock-button loading" type="button">Loading</button>
        <button class="mock-button warning" type="button">Warning</button>
        <button class="mock-button danger" type="button">Error</button>
        <button class="mock-button disabled" type="button" disabled>Disabled</button>
      </div>
      <div class="mock-grid main-grid">
        <article class="mode-card" data-state="warning">
          <div>
            <h3>Pilot Mode</h3>
            <p>Camera-first driving view with fixed layout presets and global safety controls.</p>
          </div>
          <button class="mock-button primary" type="button">Open Pilot</button>
        </article>
        <article class="mode-card" data-state="ready">
          <div>
            <h3>Setup Mode</h3>
            <p>Procedural configuration for controls, thrusters, cameras, and preflight checks.</p>
          </div>
          <button class="mock-button success" type="button">Open Setup</button>
        </article>
        <article class="mode-card" data-state="offline">
          <div>
            <h3>Developer Mode</h3>
            <p>Dense status view for ROS2 health, stale topics, services, and runtime warnings.</p>
          </div>
          <button class="mock-button warning" type="button">Open Developer</button>
        </article>
      </div>
    </section>
  `);
}

function renderSafetyScreen() {
  return appChrome(`
    <section class="mock-body">
      <div class="mock-grid state-grid">
        <article class="state-card error">
          <strong>Emergency stop</strong>
          <p>Active. Motion remains disabled until operator reset.</p>
        </article>
        <article class="state-card warn">
          <strong>Camera warning</strong>
          <p>Rear stream missing from mock fixture.</p>
        </article>
        <article class="state-card ok">
          <strong>Backend state</strong>
          <p>Offline-safe static demo. No server connection used.</p>
        </article>
        <article class="state-card">
          <strong>Control source</strong>
          <p>Physical joystick expected. Web joystick deferred.</p>
        </article>
      </div>
      <section class="mock-panel">
        <div class="mock-panel-header">
          <div>
            <h3>Safety action placement</h3>
            <p>Persistent top-right stop action plus critical alert banner.</p>
          </div>
          <button class="mock-button loading danger" type="button">Stopping</button>
        </div>
        <div class="component-strip">
          <button class="mock-button danger" type="button">Emergency Stop</button>
          <button class="mock-button warning" type="button">Disable Controls</button>
          <button class="mock-button disabled" type="button" disabled>Enable Controls</button>
          <button class="mock-button primary is-pressed" type="button">Alert Details</button>
        </div>
      </section>
    </section>
  `);
}

function cameraTiles() {
  return mock.cameras.map((camera, index) => `
    <article class="camera-tile ${index === 0 ? "large" : ""} ${camera.offline ? "offline" : ""}">
      <span class="camera-label">${camera.id} ${camera.label}</span>
      <div class="camera-status">
        <strong>${camera.state}</strong>
        <span>${camera.detail}</span>
      </div>
    </article>
  `).join("");
}

function renderPilotScreen() {
  return appChrome(`
    <section class="mock-body">
      <div class="mock-grid pilot-layout">
        <section class="mock-panel">
          <div class="mock-panel-header">
            <div>
              <h3>Pilot camera presets</h3>
              <p>Fixed layouts only. No drag or resize behavior in MVP.</p>
            </div>
            <div class="segmented" aria-label="Camera layout preset">
              <button type="button" class="active">Focus</button>
              <button type="button">Grid</button>
              <button type="button">Six</button>
              <button type="button" disabled>Custom</button>
            </div>
          </div>
          <div class="camera-grid">${cameraTiles()}</div>
        </section>
        <aside class="mock-panel">
          <h3>Pilot sidebar density</h3>
          <p>Low distraction: safety state, layout preset, and camera availability only.</p>
          <div class="state-card ok">
            <strong>Controls</strong>
            <p>Disabled while emergency stop is active.</p>
          </div>
          <div class="state-card warn">
            <strong>Offline state</strong>
            <p>Rear camera tile stays visible as a stable placeholder.</p>
          </div>
        </aside>
      </div>
    </section>
  `);
}

function renderSetupScreen() {
  return appChrome(`
    <section class="mock-body">
      <div class="mock-grid setup-layout">
        <nav class="setup-nav" aria-label="Setup sections">
          ${["Camera setup", "Control mapping", "Thruster tuning", "Preflight tests"].map((label, index) => `
            <div class="step-row ${index === 1 ? "active" : ""}">
              <span class="step-number">${index + 1}</span>
              <div>
                <strong>${label}</strong>
                <p>${index === 1 ? "Unsaved changes" : "Fixture status"}</p>
              </div>
            </div>
          `).join("")}
        </nav>
        <section class="mock-panel">
          <div class="mock-panel-header">
            <div>
              <h3>Control mapping status</h3>
              <p>Status bar remains visible while the operator moves through setup sections.</p>
            </div>
            <button class="mock-button warning loading" type="button">Saving</button>
          </div>
          <div class="form-grid">
            <div class="field">
              <label for="profile-name">Profile name</label>
              <input id="profile-name" value="pool_test_drive" readonly>
            </div>
            <div class="field">
              <label for="device-name">Joystick device</label>
              <select id="device-name" disabled>
                <option>/dev/input/js0 offline</option>
              </select>
              <span class="error-text">Device unavailable in static fixture.</span>
            </div>
          </div>
          <div class="component-strip">
            <button class="mock-button primary" type="button">Save Profile</button>
            <button class="mock-button" type="button">Revert</button>
            <button class="mock-button danger" type="button">Cancel Test</button>
            <button class="mock-button disabled" type="button" disabled>Run Thruster Pulse</button>
          </div>
        </section>
      </div>
    </section>
  `);
}

function nodeRows() {
  return mock.nodes.map(([name, status, detail]) => {
    const state = status === "Healthy" ? "ok" : status === "Warning" ? "warn" : "error";
    return `
      <tr>
        <td>${name}</td>
        <td><span class="status-pill ${state}">${status}</span></td>
        <td>${detail}</td>
      </tr>
    `;
  }).join("");
}

function renderDeveloperScreen() {
  return appChrome(`
    <section class="mock-body">
      <div class="mock-grid stat-grid">
        <article class="stat-card">
          <span class="subtext">Nodes healthy</span>
          <strong>14 / 16</strong>
        </article>
        <article class="stat-card">
          <span class="subtext">Fresh topics</span>
          <strong>28</strong>
        </article>
        <article class="stat-card">
          <span class="subtext">Warnings</span>
          <strong>3</strong>
        </article>
        <article class="stat-card">
          <span class="subtext">Critical</span>
          <strong>1</strong>
        </article>
      </div>
      <div class="mock-grid developer-layout">
        <section class="mock-panel">
          <div class="mock-panel-header">
            <div>
              <h3>ROS2 monitor density</h3>
              <p>Dense enough for developers, still grouped by actionable state.</p>
            </div>
            <button class="mock-button primary" type="button">Refresh View</button>
          </div>
          <table class="compact-table">
            <thead>
              <tr><th>Node</th><th>Status</th><th>Detail</th></tr>
            </thead>
            <tbody>${nodeRows()}</tbody>
          </table>
        </section>
        <aside class="mock-panel">
          <h3>Allowlisted diagnostics</h3>
          <div class="empty-state">No service-call results in this static fixture.</div>
          <div class="component-strip">
            <button class="mock-button primary" type="button">Copy Snapshot</button>
            <button class="mock-button disabled" type="button" disabled>Run Command</button>
          </div>
        </aside>
      </div>
    </section>
  `);
}

const screenRenderers = {
  main: renderMainScreen,
  safety: renderSafetyScreen,
  pilot: renderPilotScreen,
  setup: renderSetupScreen,
  developer: renderDeveloperScreen,
};

function renderPreview() {
  const style = currentStyleData();
  document.body.dataset.style = style.id;
  styleKicker.textContent = style.recommended ? "Agent recommendation" : "Style family";
  previewTitle.textContent = style.title;
  styleStatus.textContent = style.status;
  tradeoffNote.textContent = style.tradeoff;
  mockShell.innerHTML = screenRenderers[currentScreen]();
}

function renderAll() {
  renderSelectors();
  renderPreview();
}

styleList.addEventListener("click", (event) => {
  const button = event.target.closest("[data-style-id]");
  if (!button) {
    return;
  }
  currentStyle = button.dataset.styleId;
  renderAll();
});

screenTabs.addEventListener("click", (event) => {
  const button = event.target.closest("[data-screen-id]");
  if (!button) {
    return;
  }
  currentScreen = button.dataset.screenId;
  renderAll();
});

renderAll();
