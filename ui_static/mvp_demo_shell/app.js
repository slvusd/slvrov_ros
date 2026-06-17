const demos = [
  {
    title: "Selected Apple/macOS Direction",
    description: "The owner-selected MVP visual direction with dark/light mode and consistent notification colors.",
    href: "./prototypes/apple_macos_direction/index.html",
    state: "ready",
  },
  {
    title: "Archived Legacy Demos",
    description: "Earlier joystick mapper concepts, old examples, and starter templates were moved to archive folders for reference.",
    href: "",
    state: "reserved",
  },
];

const demoList = document.getElementById("demo-list");
const refreshViewButton = document.getElementById("refresh-view-button");

function renderDemos() {
  demoList.innerHTML = demos.map((demo) => {
    const action = demo.href
      ? `<a href="${demo.href}">Open</a>`
      : '<span class="demo-badge">Reserved</span>';

    return `
      <article class="demo-card" data-state="${demo.state}">
        <div>
          <h3>${demo.title}</h3>
          <p>${demo.description}</p>
        </div>
        ${action}
      </article>
    `;
  }).join("");
}

refreshViewButton.addEventListener("click", renderDemos);

renderDemos();
