const demos = [
  {
    title: "Prototype Template",
    description: "A starter page with static mock state, safety markers, and tradeoff notes for future UI direction demos.",
    href: "./prototypes/demo_template/index.html",
    state: "ready",
  },
  {
    title: "Task 10A UI Direction Set",
    description: "Reserved for the broader visual direction comparison: macOS-inspired, existing project, hybrid, and recommended.",
    href: "./prototypes/ui_direction_10a/index.html",
    state: "ready",
  },
  {
    title: "Task 11 Main Page Options",
    description: "Reserved for static mode-selection demos after the owner chooses the broader visual direction.",
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
