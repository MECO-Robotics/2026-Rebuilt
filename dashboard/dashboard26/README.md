# Dashboard 26

This is a standalone Flutter dashboard modeled after Team 3015's 2024 driver dashboard, but wired for this robot
project's current auto-drive field poses and game-piece vision telemetry.

It publishes:

- `/AutoDrive/ClickedPose` as `[xMeters, yMeters, headingDegrees]`
- `/AutoDrive/RequestId` as an increasing integer
- `/AutoDrive/KillRequestId` as an increasing integer when Enter or Numpad Enter is pressed

The robot schedules clicked poses from those two topics when the robot is teleop-enabled. Requests sent while disabled
or autonomous are rejected, and the robot publishes the reason back to the dashboard.

Pressing Enter sends the dashboard software kill. The robot cancels running commands, stops drivetrain output, zeros
rollers/flywheels, and latches that stopped state until teleop is enabled again. This is not a replacement for the
Driver Station disable button, main breaker, or field E-stop.

It subscribes to:

- `/SmartDashboard/Match Time`
- `/SmartDashboard/Auto Choices/options`
- `/SmartDashboard/Auto Choices/active`
- `/SmartDashboard/AutoDrive/Is Red Alliance`
- `/SmartDashboard/AutoDrive/Enabled`
- `/SmartDashboard/AutoDrive/Mode`
- `/SmartDashboard/AutoDrive/Active`
- `/SmartDashboard/AutoDrive/LastAcceptedRequestId`
- `/SmartDashboard/AutoDrive/LastKillRequestId`
- `/SmartDashboard/AutoDrive/LastRejectedReason`
- `/SmartDashboard/DriverDashboard/*` for game-piece camera status, selected ball group, biggest/closest groups,
  distance, yaw, and camera stream URLs

It also writes `/SmartDashboard/Auto Choices/selected` when an autonomous mode is picked from the dashboard dropdown.
This is the same chooser value the robot reads at autonomous init. Raw AdvantageScope NetworkTables views show the
chooser data, but they do not render it as a clickable dropdown.

## Setup

Install Flutter with Windows desktop support, then generate the Windows runner once:

```powershell
cd dashboard\dashboard26
flutter create . --platforms=windows
flutter pub get
```

Run against simulation:

```powershell
flutter run -d windows --dart-define=ROBOT_ADDRESS=127.0.0.1
```

On macOS, run the web dashboard in Chrome:

```bash
flutter run -d chrome --dart-define=ROBOT_ADDRESS=127.0.0.1
```

Run against the robot:

```powershell
flutter run -d windows --dart-define=ROBOT_ADDRESS=10.83.24.2
```

On macOS against the robot:

```bash
flutter run -d chrome --dart-define=ROBOT_ADDRESS=10.83.24.2
```

Click directly on the field to request a pathfind to that pose. Click one of the named targets to request its preset
pose. The request only runs when the robot is teleop-enabled.

Use the `Auto` dropdown near the match timer to choose autonomous before enabling autonomous mode in the sim GUI or
Driver Station.

The right-side vision panel shows the front and rear game-piece camera streams from the URLs published by the robot.
If a stream cannot be decoded by Flutter on the current platform, the panel still shows the camera dashboard URL so the
same stream can be opened directly in a browser.
