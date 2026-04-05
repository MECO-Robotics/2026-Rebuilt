# MECO 2026 Rebuilt Robot Code

This repository contains the 2026 rebuilt robot code for MECO Robotics. The project is built on WPILib command-based, AdvantageKit logging, CTRE Phoenix 6 swerve, and simulation support for desktop development.

## Robot Overview

The current robot configuration in [`RobotContainer`](src/main/java/frc/robot/RobotContainer.java) includes:

- `CommandSwerveDrivetrain`: CTRE Phoenix 6 swerve drivetrain
- `Flywheel` subsystems:
  - shooter flywheel
  - top indexer
  - bottom indexer
  - conveyor
  - intake roller
- `PositionJoint` subsystems:
  - intake rack
  - shooter hood
- `Vision`: Limelight-backed pose updates with sim support
- `RobotSimulation`: desktop simulation hooks for drivetrain, intake, hood, and shooter

## Key Files

- [`src/main/java/frc/robot/RobotContainer.java`](src/main/java/frc/robot/RobotContainer.java): subsystem construction, button bindings, autonomous chooser, named commands
- [`src/main/java/frc/robot/Robot.java`](src/main/java/frc/robot/Robot.java): AdvantageKit startup, mode selection, scheduler loop
- [`src/main/java/frc/robot/constants/Constants.java`](src/main/java/frc/robot/constants/Constants.java): runtime mode selection (`REAL`, `SIM`, `REPLAY`)
- [`src/main/java/frc/robot/constants/drive/DrivetrainConstants.java`](src/main/java/frc/robot/constants/drive/DrivetrainConstants.java): CANivore, swerve IDs, offsets, gains, sim model
- [`src/main/java/frc/robot/constants/subsystems/IntakeConstants.java`](src/main/java/frc/robot/constants/subsystems/IntakeConstants.java): intake roller, rack, and intake presets
- [`src/main/java/frc/robot/constants/subsystems/ShooterConstants.java`](src/main/java/frc/robot/constants/subsystems/ShooterConstants.java): shooter, hood, conveyor, indexers, and shot maps
- [`src/main/java/frc/robot/constants/vision/VisionConstants.java`](src/main/java/frc/robot/constants/vision/VisionConstants.java): camera naming and transforms

## Requirements

- Java 17
- WPILib 2026 toolchain
- Vendor dependencies from [`vendordeps`](vendordeps)
- Gradle wrapper included in the repo

## Common Commands

From the repository root:

```powershell
.\gradlew build
.\gradlew test
.\gradlew simulateJava
.\gradlew deploy
.\gradlew replayWatch
```

Notes:

- `simulateJava` runs the desktop sim. The WPILib sim GUI is disabled by default in [`build.gradle`](build.gradle) to support log replay workflows.
- `replayWatch` launches AdvantageKit ReplayWatch.
- Deploy behavior uses the team number from WPILib preferences.

## Runtime Modes and Logging

Runtime mode is selected in [`Constants.java`](src/main/java/frc/robot/constants/Constants.java):

- `REAL`: logs to WPILOG and NT4
- `SIM`: logs to NT4
- `REPLAY`: replays a WPILOG file and writes a `_sim` output log

AdvantageKit metadata is recorded at startup, including Git branch, SHA, build date, and dirty state. URCL is also enabled, and REV auto-logging is explicitly disabled in [`Robot.java`](src/main/java/frc/robot/Robot.java).

## Driver Controls

Current bindings defined in [`RobotContainer.java`](src/main/java/frc/robot/RobotContainer.java):

- Driver `Start`: reset heading
- Driver `Left Bumper`: run intake roller
- Driver `POV Up`: deploy intake
- Driver `POV Down`: stow intake
- Driver `Right Bumper`: feed shooter and agitate intake
- Driver `A`: auto-aim to hub while running calculated hood/flywheel shot
- Driver or Co-pilot `B`: shooter idle preset
- Co-pilot `X`: hub preset
- Co-pilot `Y`: ferry preset
- Co-pilot `A`: trench preset
- Co-pilot `POV Up/Down`: backup intake deploy/stow controls

## Autonomous

Autonomous support currently includes:

- PathPlanner named commands registration
- Choreo-based trajectory following through `AutoFactory`
- Dashboard chooser entries created in `configureAuto()`

Named commands currently registered:

- `DeployIntake`
- `StowIntake`
- `FeedRollers`
- `IdleRollers`
- `SpinIntake`
- `AutoSpinUp`
- `Fender`
- `AutoAim`

## Configuration Workflow

Before running on hardware, review these values:

1. CAN IDs, inversion, gearing, and encoder configuration in drivetrain, intake, shooter, and vision constants.
2. Limelight name and robot-to-camera transform in [`VisionConstants`](src/main/java/frc/robot/constants/vision/VisionConstants.java).
3. `Constants.simMode` if you want desktop execution to use `SIM` or `REPLAY`.
4. Logged tunable presets in `IntakeConstants` and `ShooterConstants`.

The intake and shooter presets are implemented with `LoggedTunableNumber`, so they can be adjusted during tuning and recorded in logs.

## Repository Layout

```text
src/main/java/frc/robot
|-- commands
|-- constants
|-- simulation
|-- subsystems
`-- util
```

Subsystem IO implementations are split by backend where needed, including Spark Max, TalonFX, sim, and replay-oriented code paths.
