# MVP Data Directory

Collected ROV data should live outside the source repository so photos,
videos, CSV logs, and test logs do not get mixed into code history.

Recommended target-machine layout:

```text
slvrov_workspace/
├── slvrov_ros/
└── data/
    ├── photos/
    ├── videos/
    ├── csv/
    ├── preflight_logs/
    ├── test_logs/
    └── metadata/
```

Future MVP work should keep source-code defaults and examples in this
repository, but write real collected files to the sibling `data/`
directory or another configured runtime path.
