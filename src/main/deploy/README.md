# RoboRIO deploy files

GradleRIO copies this directory to `/home/lvuser/deploy` during deployment.

## Expected layout

```text
src/main/deploy/
|-- README.md
|-- swerve_offsets.json
`-- pathplanner/
    `-- paths/
        `-- <SelectedPath>.path
```

`FRCAutoOrchestrator` loads a selected path from `pathplanner/paths/<name>.path`. Competition paths must be placed here before deployment. The `SimPath.path` under `src/main/resources/deploy/` is a test/simulation classpath fixture and is not copied here automatically.

## Swerve offsets

`swerve_offsets.json` contains module azimuth offsets in rotations:

- `frontLeft`
- `frontRight`
- `backLeft`
- `backRight`

The Gradle `fetchOffsets` task retrieves `/home/lvuser/swerve_offsets_runtime.json` from the RoboRIO at `10.232.47.2` and overwrites this baseline file. Review all four values after fetching and re-test module orientation before deploying.

Do not put secrets or cloud credentials in this directory. The robot is offline-first: logs and telemetry are consumed over the local robot network, while cloud synchronization runs on the laptop.
