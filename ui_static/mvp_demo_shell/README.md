# MVP Static UI Demo Shell

This directory is a disposable review area for static ROV web UI prototypes.
It is intentionally outside `src/slvrov_core_python/slvrov_core_python/web/static/`
so prototype work stays separate from production frontend files.

## View The Demo Index

Open this file directly in a browser:

```text
ui_static/mvp_demo_shell/index.html
```

Or run a static file server from the repository root:

```bash
python3 -m http.server 8000 --directory ui_static/mvp_demo_shell
```

Then open:

```text
http://localhost:8000/
```

## Add A Prototype

1. Copy `prototypes/demo_template/` to a new directory under `prototypes/`.
2. Update the title and tradeoff notes in the copied `index.html`.
3. Add the prototype to the `demos` array in `app.js`.

Prototype pages must remain static. Do not add Flask routes, ROS2 calls,
MediaMTX/WebRTC connections, API clients, or `fetch()` calls to backend
endpoints. Use local fixture objects or hard-coded mock state only.

## Review Checklist

- The page is labeled as non-production.
- Emergency stop and critical alert examples are visible when relevant.
- At least one offline, empty, warning, or error state is represented.
- Text and controls do not overlap at 4:3 and 16:10-ish viewports.
- The prototype is easy to remove or archive after the owner chooses a
  direction.
