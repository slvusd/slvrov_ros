const screens = [
  { id: "main", title: "Main" },
  { id: "safety", title: "Safety" },
  { id: "pilot", title: "Pilot" },
  { id: "setup", title: "Setup" },
  { id: "developer", title: "Developer" },
];

const cameras = [
  { id: "CAM 01", label: "Forward", state: "Live", detail: "42 ms", tone: "status" },
  { id: "CAM 02", label: "Manipulator", state: "Live", detail: "48 ms", tone: "status" },
  { id: "CAM 03", label: "Rear", state: "Offline", detail: "No signal", tone: "error" },
  { id: "CAM 04", label: "Port", state: "Live", detail: "51 ms", tone: "status" },
  { id: "CAM 05", label: "Starboard", state: "Live", detail: "46 ms", tone: "status" },
  { id: "CAM 06", label: "Down", state: "Empty", detail: "Not assigned", tone: "warning" },
];

const nodes = [
  ["web_ros_bridge", "Status", "42 ms"],
  ["safety_monitor", "Error", "E-stop active"],
  ["preflight_test", "Warning", "Thruster test skipped"],
  ["camera_supervisor", "Error", "Rear camera missing"],
];

let currentScreen = "main";

const screenTabs = document.getElementById("screen-tabs");
const previewTitle = document.getElementById("preview-title");
const mockShell = document.getElementById("mock-shell");
const themeToggle = document.getElementById("theme-toggle");

function renderTabs() {
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
          <span class="brand-mark">SEL</span>
          <div>
            <h3>SEL ROV Control</h3>
            <p>Sea Exploration League static fixture</p>
          </div>
        </div>
        <div class="mock-actions">
          <button class="button status" type="button">System Status</button>
          <button class="button error" type="button">Emergency Stop</button>
        </div>
      </header>
      <section class="notification error">
        <strong>Error</strong>
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
        <button class="button primary" type="button">Primary</button>
        <button class="button primary is-pressed" type="button">Pressed</button>
        <button class="button warning loading" type="button">Loading</button>
        <button class="button warning" type="button">Warning</button>
        <button class="button error" type="button">Error</button>
        <button class="button disabled" type="button" disabled>Disabled</button>
      </div>
      <div class="mode-grid">
        ${["Pilot Mode", "Setup Mode", "Developer Mode"].map((title) => `
          <article class="mode-card">
            <div>
              <h3>${title}</h3>
              <p>${modeDescription(title)}</p>
            </div>
            <button class="button primary" type="button">Open ${title.split(" ")[0]}</button>
          </article>
        `).join("")}
      </div>
    </section>
  `);
}

function modeDescription(title) {
  if (title === "Pilot Mode") {
    return "Camera-first driving view with fixed layout presets and global safety controls.";
  }
  if (title === "Setup Mode") {
    return "Procedural configuration for controls, thrusters, cameras, and preflight checks.";
  }
  return "Organized diagnostics for ROS2 health, stale topics, services, and runtime warnings.";
}

function renderSafetyScreen() {
  return appChrome(`
    <section class="mock-body">
      <div class="state-grid">
        <article class="state-card error"><strong>Emergency stop</strong><p>Active. Motion remains disabled until operator reset.</p></article>
        <article class="state-card warning"><strong>Camera warning</strong><p>Rear stream missing from mock fixture.</p></article>
        <article class="state-card status"><strong>System status</strong><p>Static demo is healthy without backend services.</p></article>
        <article class="state-card"><strong>Control source</strong><p>Physical joystick expected. Web joystick deferred.</p></article>
      </div>
      <section class="panel">
        <div class="panel-header">
          <div>
            <h3>Global safety placement</h3>
            <p>Persistent emergency action plus critical notification banner.</p>
          </div>
          <button class="button error loading" type="button">Stopping</button>
        </div>
        <div class="component-strip">
          <button class="button error" type="button">Emergency Stop</button>
          <button class="button warning" type="button">Disable Controls</button>
          <button class="button disabled" type="button" disabled>Enable Controls</button>
          <button class="button primary is-pressed" type="button">Alert Details</button>
        </div>
      </section>
    </section>
  `);
}

function cameraTiles() {
  return cameras.map((camera, index) => `
    <article class="camera-tile ${index === 0 ? "large" : ""} ${camera.tone}">
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
      <div class="pilot-layout">
        <section class="panel">
          <div class="panel-header">
            <div>
              <h3>Pilot camera presets</h3>
              <p>Fixed layouts only. No mode-specific color treatment.</p>
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
        <aside class="panel">
          <h3>Pilot status density</h3>
          <p>Low distraction: safety state, layout preset, and camera availability only.</p>
          <div class="state-card status"><strong>Controls</strong><p>Disabled while emergency stop is active.</p></div>
          <div class="state-card warning"><strong>Offline placeholder</strong><p>Unavailable cameras stay visible without changing mode color.</p></div>
        </aside>
      </div>
    </section>
  `);
}

function renderSetupScreen() {
  return appChrome(`
    <section class="mock-body">
      <div class="setup-layout">
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
        <section class="panel">
          <div class="panel-header">
            <div>
              <h3>Control mapping status</h3>
              <p>Status bar can remain visible without recoloring the whole mode.</p>
            </div>
            <button class="button warning loading" type="button">Saving</button>
          </div>
          <div class="form-grid">
            <div class="field"><label for="profile-name">Profile name</label><input id="profile-name" value="pool_test_drive" readonly></div>
            <div class="field"><label for="device-name">Joystick device</label><select id="device-name" disabled><option>/dev/input/js0 offline</option></select><span class="error-text">Device unavailable in static fixture.</span></div>
          </div>
          <div class="component-strip">
            <button class="button primary" type="button">Save Profile</button>
            <button class="button" type="button">Revert</button>
            <button class="button error" type="button">Cancel Test</button>
            <button class="button disabled" type="button" disabled>Run Thruster Pulse</button>
          </div>
        </section>
      </div>
    </section>
  `);
}

function nodeRows() {
  return nodes.map(([name, status, detail]) => {
    const tone = status.toLowerCase();
    return `<tr><td>${name}</td><td><span class="pill ${tone}">${status}</span></td><td>${detail}</td></tr>`;
  }).join("");
}

function renderDeveloperScreen() {
  return appChrome(`
    <section class="mock-body">
      <div class="stat-grid">
        <article class="stat-card"><span>Nodes status</span><strong>14 / 16</strong></article>
        <article class="stat-card"><span>Fresh topics</span><strong>28</strong></article>
        <article class="stat-card"><span>Warnings</span><strong>3</strong></article>
        <article class="stat-card"><span>Errors</span><strong>1</strong></article>
      </div>
      <div class="developer-layout">
        <section class="panel">
          <div class="panel-header">
            <div>
              <h3>ROS2 monitor density</h3>
              <p>Dense enough for developers, still grouped by notification state.</p>
            </div>
            <button class="button primary" type="button">Refresh View</button>
          </div>
          <table class="compact-table"><thead><tr><th>Node</th><th>State</th><th>Detail</th></tr></thead><tbody>${nodeRows()}</tbody></table>
        </section>
        <aside class="panel">
          <h3>Allowlisted diagnostics</h3>
          <div class="empty-state">No service-call results in this static fixture.</div>
          <div class="component-strip">
            <button class="button primary" type="button">Copy Snapshot</button>
            <button class="button disabled" type="button" disabled>Run Command</button>
          </div>
        </aside>
      </div>
    </section>
  `);
}

const renderers = {
  main: renderMainScreen,
  safety: renderSafetyScreen,
  pilot: renderPilotScreen,
  setup: renderSetupScreen,
  developer: renderDeveloperScreen,
};

function renderPreview() {
  const screen = screens.find((item) => item.id === currentScreen);
  previewTitle.textContent = screen.title === "Main" ? "Main Mode Selection" : `${screen.title} Screen`;
  mockShell.innerHTML = renderers[currentScreen]();
}

screenTabs.addEventListener("click", (event) => {
  const button = event.target.closest("[data-screen-id]");
  if (!button) {
    return;
  }
  currentScreen = button.dataset.screenId;
  renderTabs();
  renderPreview();
});

themeToggle.addEventListener("click", () => {
  const nextTheme = document.body.dataset.theme === "dark" ? "light" : "dark";
  document.body.dataset.theme = nextTheme;
  themeToggle.textContent = nextTheme === "dark" ? "Light Mode" : "Dark Mode";
});

renderTabs();
renderPreview();
