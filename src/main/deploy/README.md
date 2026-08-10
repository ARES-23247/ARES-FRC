# RoboRIO deploy files

GradleRIO copies this directory to `/home/lvuser/deploy` during deployment.

## Expected layout

```text
src/main/deploy/
|-- README.md
|-- swerve_offsets.json
|-- ares/
|   |-- auto-capabilities.json
|   `-- autos/
|       `-- <document-id>.aresauto
`-- paths/
    `-- field.json
```

`FRCAutoOrchestrator` loads the native document selected by
`SmartDashboard/SelectedAuto`. Analytics saves GUI-authored routines directly into
`ares/autos`; no robot connection or PathPlanner conversion is required. `do-nothing` is the
fail-safe default. `auto-capabilities.json` is the offline action catalog used by Analytics and is
kept in exact parity with `FrcAutoCapabilities` by tests.

`do-nothing` is reserved: the runner validates it but preserves the current localized pose instead
of applying the document's placeholder starting pose.

Autos are authored once in Blue-alliance, corner-origin field coordinates. Red execution reflects
X across the alliance-wall axis before trajectory generation. Keep every robot center at least
0.40 m from the field boundary for Marvin's current 0.80 m square bumper footprint.

## Swerve offsets

`swerve_offsets.json` contains module azimuth offsets in rotations:

- `frontLeft`
- `frontRight`
- `backLeft`
- `backRight`

The Gradle `fetchOffsets` task retrieves `/home/lvuser/swerve_offsets_runtime.json` from the RoboRIO at `10.232.47.2` and overwrites this baseline file. Review all four values after fetching and re-test module orientation before deploying.

Do not put secrets or cloud credentials in this directory. The robot is offline-first: logs and telemetry are consumed over the local robot network, while cloud synchronization runs on the laptop.
