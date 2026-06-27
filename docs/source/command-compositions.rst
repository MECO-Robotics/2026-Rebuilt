Command Compositions
====================

Use commands to combine subsystem behaviors into operator controls, autos, and repeatable actions.

In this repository, commands are where subsystem behaviors are combined into useful robot actions. Individual
subsystems expose focused control methods, while command factories in ``commands/`` build the multi-step actions used
for teleop bindings, autonomous routines, and simulation hooks.


What belongs in a command composition
-------------------------------------

Use command compositions when an action needs to coordinate more than one subsystem or more than one phase of behavior.

Examples from this repo:

- run the intake rack and intake roller together
- spin up the shooter while moving the hood
- aim the drivetrain while calculating a shot
- sequence an autonomous trajectory and then shoot

Keep single-mechanism control low in the subsystem or command factory layer. Put robot behavior that combines multiple
mechanisms in a composition.


Where compositions live
-----------------------

Most command compositions in this repo are built in command factory classes:

- ``IntakeCommands`` for intake rack, roller, and conveyor actions
- ``ShooterCommands`` for hood, shooter, indexers, and conveyor actions
- ``DriveCommands`` for heading control and driver-assist aiming
- ``ShooterCalculator`` for calculated shots based on robot state

Those factories are then wired into controller bindings and autonomous setup in :doc:`configure/robotcontainer`.


Common patterns in this repo
----------------------------

Parallel actions
~~~~~~~~~~~~~~~~

Use ``Commands.parallel(...)`` when several mechanisms should run at the same time.

This is used heavily in this codebase:

- ``IntakeCommands.deployIntake(...)`` moves the rack while keeping the roller idle
- ``IntakeCommands.stowIntake(...)`` moves the rack while driving the conveyor and roller
- ``ShooterCommands.hubPreset(...)`` moves the hood while spinning the shooter flywheel

This pattern keeps the subsystem commands small and makes the higher-level behavior easy to read.

Deadline groups
~~~~~~~~~~~~~~~

Use ``Commands.deadline(...)`` when one command should define how long the group runs and the rest should stop with it.

Examples:

- ``ShooterCommands.feedRollers(...)`` uses a deadline group so the feed action controls the lifetime of the roller set
- timed autonomous shot sequences use ``Commands.deadline(Commands.waitSeconds(...), ...)`` to run a preset only for a
  fixed window

Use this when you want "run these together until this one finishes."

Sequencing
~~~~~~~~~~

Use ``Commands.sequence(...)`` when order matters.

The autonomous code in ``RobotContainer`` uses sequencing to:

1. reset odometry
2. reset the path controller
3. follow a Choreo trajectory
4. stop the drivetrain
5. run a timed shot

This is the right pattern whenever a later step depends on an earlier one finishing first.

Driver-assist compositions
~~~~~~~~~~~~~~~~~~~~~~~~~~

Not every command composition is a simple preset. Some combine closed-loop drivetrain control with another mechanism.

The clearest example in this repo is the ``A`` button binding in ``RobotContainer``:

- ``DriveCommands.joystickAimToHub(...)`` keeps translation under driver control while locking heading toward the hub
- ``ShooterCalculator.calculateAndShoot(...)`` computes the shot and commands hood/flywheel targets

That composition gives assisted aiming without taking away all driver control.


Teleop examples from RobotContainer
-----------------------------------

This repo uses controller bindings to compose commands directly where operator intent is clearest.

Examples:

- left bumper runs ``IntakeCommands.spinIntake(...)`` while held and idles on release
- right bumper agitates the indexers while stowing the intake
- copilot preset buttons run ``hubPreset``, ``ferryPreset``, and ``trenchPreset``
- start resets drivetrain heading

These bindings are small because the actual behavior has already been packaged into reusable command factories.


Autonomous examples
-------------------

Autonomous behavior is built from the same command pieces rather than using a separate architecture.

This repo composes autonomous actions in two main ways:

- named commands registered with PathPlanner using ``NamedCommands.registerCommand(...)``
- explicit command sequences for Choreo routines in ``RobotContainer``

Examples of named commands in this repo include:

- ``DeployIntake``
- ``StowIntake``
- ``FeedRollers``
- ``AutoSpinUp``
- ``AutoAim``

This keeps autonomous behavior aligned with teleop behavior and avoids duplicate robot logic.

The ``Autonomous Ball Harvest`` chooser option uses the game-piece camera pipeline when running on the real robot. It
can run with the front camera, rear camera, or both, based on
``PieceDetectionConstants.ENABLE_FRONT_GAME_PIECE_CAMERA`` and
``PieceDetectionConstants.ENABLE_REAR_GAME_PIECE_CAMERA``. The enabled Beelink/Arducam services are expected to publish
JSON at the URIs configured in
``PieceDetectionConstants.FRONT_GAME_PIECE_CAMERA_DATA_URI`` and
``PieceDetectionConstants.REAR_GAME_PIECE_CAMERA_DATA_URI``. ``PieceDetectionIOMultiCamera`` polls both cameras and
uses the highest-scoring selected group. Each ``PieceDetectionIOHttp`` instance reads detected ball groups, estimates
the biggest group, closest group, and best group, carries the camera's circle-only ``shape`` label for the selected
group, then applies the ``60/70`` kept-ball ratio before the autonomous command pathfinds to the chosen group, deploys
the intake, and returns to a hub shot pose. Before feeding, the command
holds hub auto-aim while waiting briefly for ``Vision`` to report a fresh accepted AprilTag pose, so the final lineup is
based on AprilTag-corrected odometry instead of dead-reckoning alone. The chooser command is capped by
``GamePieceAutonomyCommands.FULL_MATCH_RUNTIME_SECONDS`` at 160 seconds, matching the 2:40 runtime from the start
button to the end of the game.

Known field obstacles should be represented in PathPlanner's navgrid/settings. Dynamic obstacle avoidance still needs a
separate source of obstacle positions. ``RobotObstacleTracker`` subscribes to the NetworkTables topic
``Autonomy/RobotObstacles`` and expects a flattened array of field-relative robot centers in meters:
``x0, y0, x1, y1, ...``. Each center is expanded into a padded obstacle box before being passed to PathPlanner's AD*
pathfinder, so the ball-harvest auto can route around other robots while chasing the selected group.

In desktop simulation, ``PieceDetectionIOSim`` supplies a small set of fixed game-piece poses so the harvest loop can be
exercised without a robot or camera services. The full Gradle simulation still requires the normal FRC Java 17 toolchain.

The driver station dashboard gets a dedicated ``8324 Driver`` Shuffleboard tab from ``DriverDashboard``. It embeds the
front and rear game-piece MJPEG streams configured by
``PieceDetectionConstants.FRONT_GAME_PIECE_CAMERA_VIDEO_URI`` and
``PieceDetectionConstants.REAR_GAME_PIECE_CAMERA_VIDEO_URI``, and publishes selected-group telemetry under
``DriverDashboard/*`` on SmartDashboard. The live values include camera connection, whether balls are visible, detected
group count, selected ball count, selected distance/yaw, biggest and closest group distance, kept-ball estimate, and the
reason the robot chose that group.

The standalone Flutter dashboard in ``dashboard/dashboard26`` subscribes to the same ``DriverDashboard/*`` and
``AutoDrive/*`` NetworkTables keys, so it can be used as the team's custom match dashboard alongside the normal driver
station and Shuffleboard views.

``DashboardAutoDriveController`` listens for Flutter field clicks on ``/AutoDrive/ClickedPose`` as
``[xMeters, yMeters, headingDegrees]`` and ``/AutoDrive/RequestId`` as an increasing integer. A new request is accepted
only while the robot is teleop-enabled and the pose is inside the field bounds, then it schedules a PathPlanner
``pathfindToPose`` command with the same moderate speed limits used by the ball-harvest behavior. Requests sent while
disabled, autonomous, malformed, or outside the field are rejected and reported through
``/SmartDashboard/AutoDrive/LastRejectedReason``.

The Flutter dashboard Enter/Numpad Enter key sends ``/AutoDrive/KillRequestId``. The robot treats that as a latched
software stop: it cancels running commands, stops drivetrain output, zeros flywheels and rollers, and keeps applying
those stopped outputs until teleop is enabled again. This does not replace the Driver Station disable button, main
breaker, or field E-stop.


Guidelines
----------

- Keep subsystem methods and primitive commands small.
- Put multi-subsystem behavior in command factory classes.
- Reuse the same commands in teleop and autonomous when possible.
- Prefer ``parallel``, ``deadline``, and ``sequence`` over custom state machines when standard command groups are
  enough.
- Put controller-specific wiring in ``RobotContainer``, not inside subsystem classes.
