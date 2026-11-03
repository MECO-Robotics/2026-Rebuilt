# Remy Elastic testing layout

Open `remy-sim-testing.json` in Elastic using **File → Open Layout**. If you
already have a layout you want to keep, export it first using **File → Export
Layout**. Start the updated robot simulation and connect Elastic to localhost
(`127.0.0.1`). On the driver station, connect to the robot instead.

The three tabs fit a 1280 × 768 dashboard canvas on a 64-pixel grid:

- **Driving:** estimated field pose, auto chooser, feeding, flywheel graphs,
  hood/rack positions, and feeding voltage requests.
- **Shooter Testing:** adjacent target/actual flywheel graphs, hood and rack,
  plus requested voltage, applied motor-voltage arrays, and measured speed for
  both indexers, conveyor, and intake rollers.
- **Auto Testing:** chooser, field pose, shot readiness, flywheel graphs,
  rack command owner and position, hood, and feeding voltage requests.

Use **File → Save** after resizing widgets for your screen. The chooser is an
input; the other widgets are telemetry. Do not edit telemetry text to attempt
motor control. Keep AdvantageScope open for the moving 3D mechanisms and fuel.

## Reading the widgets

- `Feeding requested` reports the command's decision, not confirmed movement.
- Auto/coordinated readiness and shot distance only update while a coordinated
  shot runs. They can retain their last values afterward. Manual RB's blocked
  reason belongs to the manual feed command; it is not an auto failure report.
- Target and actual speed graphs are separate, with independently scaled axes.
  Compare the numeric values and times, not just graph height.
- Applied motor volts are arrays in hardware configuration order. Requested
  voltage and applied voltage are different measurements; controller inversion
  and simulation implementation can affect their signs.
- Rack position is metres; hood position is mechanism rotations; flywheel and
  roller velocity are mechanism rotations per second.
- The rack command owner is not a complete auto-stage tracker. This layout does
  not claim to display elapsed auto time or identify every nested path command.
- Empty topics while disconnected are expected. Shot-only topics can remain
  empty until that command has run. The field shows estimated pose; the icon's
  dimensions are only a display footprint, not a collision or clearance check.

## Topic mapping and validation

The layout uses existing `/AdvantageKit/<subsystem>/...` inputs and
`/AdvantageKit/RealOutputs/...` outputs, `/SmartDashboard/Shooter/...`, and
`/SmartDashboard/Auto Choices`. The field reads `/Pose/Robot`, published alongside
the existing `/Pose/robotPose` with identical x/y/metres and heading/degrees.
No control bindings or mechanism requests were changed for this layout.

The JSON follows Elastic's official test-layout format. JSON parsing, widget
bounds/overlap checks, and Java compilation passed. Import/rendering and live
widget updates in the installed Elastic app still need a local check.

Official instructions: https://frc-elastic.gitbook.io/docs/getting-started/app-navigation

## Simulated scoring

The Driving tab has Auto, Teleop, and Total fuel counters. These read
`/AdvantageKit/RealOutputs/FieldSimulation/Scoring/Auto`, `/Teleop`, and `/Total`
(the last two share the same `Scoring` prefix). They count successful practice
shots, not official active-hub match points. Mode is captured at launch, so an
auto shot landing after the teleop transition still counts toward Auto. Total is
Auto + Teleop; Test-mode shots are excluded. Disable or DS disconnect resets all
three, while a direct enabled auto-to-teleop transition preserves them.

Restart simulation and reopen `remy-sim-testing.json` in Elastic to load the new
widget topics. This layout preserves the saved user arrangement, including the
three scoring widgets. No running Elastic preferences were overwritten. These
counters are simulation-only and are not an FMS score feed on the real robot.
