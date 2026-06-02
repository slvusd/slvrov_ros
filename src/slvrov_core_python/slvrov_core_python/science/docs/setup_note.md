# `slvrov_core_python.science` MVP Setup Note

This package is reserved for science, camera, media-capture, and future
sensor data-collection code.

Task 1 only creates structure. Later tasks can add:

- `capture/`: photo/video capture helpers or interfaces.
- `media/`: media filename, metadata, and storage helpers.
- `data_logging/`: future CSV/sensor data logging helpers.

Collected photos, videos, CSV files, and metadata should be written to the
external data directory documented in `docs/data_directory.md`, not inside
this package.
