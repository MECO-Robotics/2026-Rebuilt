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

- ``IntakeCommands.acquire(...)`` deploys the rack while running the complete acquisition path
- ``IntakeCommands.stowIntake(...)`` moves the rack while driving the conveyor and roller
- ``ShooterCommands.hubPreset(...)`` continuously holds the hood and shooter flywheel preset

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

The clearest example in this repo is the coordinated ``A`` button shot in ``RobotContainer``:

- when ``A`` is pressed inside the robot's own alliance zone, the alliance hub is selected even if no tag is visible
- outside the alliance zone, a visible same-alliance-side AprilTag selects the alliance hub; otherwise the command
  selects a safe ferry landing point inside the alliance zone; tag visibility is retained for 0.25 seconds to bridge
  normal gaps between camera frames
- red uses tags 1--16 for the hub-side visibility check and tags 6/7 only as trench-lane references; blue uses tags
  17--32 and tags 22/23 respectively
- each alliance has two ferry landing points, one on each side of the field; both are 0.90 m inside the alliance-zone
  boundary and shifted 0.40 m inward from the nearby guardrail-side tag reference, and neither is located at an
  AprilTag or at the hub
- the robot's current field half selects the matching ferry point; crossing the field centerline while holding ``A``
  switches sides, resets the readiness delay, and stops automatic feeding until the new aim is ready
- the hub-versus-ferry mode is latched until ``A`` is released, so seeing a trench tag after the robot starts turning
  cannot make the command switch modes
- driver translation remains available while heading locks to the selected target from the shooter exit position
- hub shots use continuously calculated hood/flywheel values; ferry shots use the ferry hood position and calculate
  the flywheel speed needed to land on the carpet at the selected target
- indexers and conveyor feed only after pose, range, speed, heading, hood, and flywheel readiness are stable
- until a field pose is ready, the command holds its current heading, stops the flywheel, and keeps the hood stowed

Before feeding, a ferry solution also checks the complete 47-inch trench depth using a conservative trajectory that
accounts for FUEL radius, launch-angle/speed variation, and extra vertical clearance. It rejects a robot that is too
close to clear the trench, a horizontal path through the hub structure, or a shot requiring more than the configured
ferry maximum RPS. Rejected ferry shots report ``UNSAFE_FERRY_PATH`` and do not feed automatically.

The reusable coordinated-shot command retains its readiness reporting for autonomous/test consumers. Driver
right bumper uses a separate agitation/feed command independent of ``A``; driver rumble uses two strong pulses per second while blocked and a weaker steady signal while feeding.
Release, Disable, and interruption clear feedback; ``Shooter/FeedBlockedReason`` explains the blocked state.

For live shot calibration, enable the AdvantageScope/NetworkTables calibration switch, then press ``X`` to latch
bounded live hood/flywheel values. Aim manually during calibration and use right bumper to fire the trial; A replaces calibration with table-based aiming and spin-up. ``B`` switches the
shooter to zero voltage. Autonomous commands use the saved shot table and ignore the calibration switch.


Teleop examples from RobotContainer
-----------------------------------

This repo uses controller bindings to compose commands directly where operator intent is clearest.

Examples:

- right trigger deploys the intake and runs acquisition: intake + conveyor inward, indexers reverse
- left trigger deploys the rack and ejects using independent intake/conveyor/indexer directions
- right bumper runs the intake immediately, then starts continuous feed and rack retraction together after initial flywheel spin-up
- releasing either trigger or right bumper stops its fuel-path rollers; right-bumper release returns the rack deployed
- ``A`` aims at the alliance hub while allowing translation and owns the hood and flywheel to continuously apply interpolation-table setpoints; it does not own the fuel path
- ``X``/``Y`` latch hub/ferry hood and flywheel presets with ``onTrue``; releasing the button does not cancel RPM hold
- copilot ``X``/``Y`` latch hub/ferry and copilot ``A`` latches trench
- ``B`` on either controller interrupts the preset and commands zero-voltage shooter output, disabling velocity PID
- driver Start resets heading; D-pad bindings retain their existing behavior
- driver left bumper is unassigned

Presets remain scheduled after the selection edge, so the normal zero-voltage default cannot overwrite their RPM
hold. Agitation/feed never claims the shooter or drivetrain: preset changes, stop, and auto-aim remain independent.
Normal command interruption and Disable stop outputs. Simultaneous fuel-path commands compete for the same
requirements; release and re-press a held action after it is interrupted by another action.

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
- ``CoordinatedHubShot``

``SpinIntake`` runs the intake roller, conveyor, and both indexers in the same collection
directions as teleop acquisition. All four outputs stop when the stage ends or is interrupted.
Existing paths use this behavior without waypoint or timing changes. Collection does not
wait for shooter readiness; shooting commands retain their readiness checks.

``FeedRollers`` and ``Agitate`` run a coordinated hub shot for the duration of their enclosing PathPlanner
race/deadline. They run the intake roller, maintain flywheel speed and hood position, and aim continuously.
The conveyor and both indexers wait for initial stable shot readiness, then feed continuously through speed,
hood and heading fluctuations until the stage ends. Loss of a valid pose or calibrated shot solution blocks
feeding and requires readiness again. Each new stage checks readiness again. Cancellation stops all rollers
and the shooter. These commands must have an enclosing duration; they do not finish on their own.
The standalone default/Choreo shot and systems-check practice shot retain their separate bounded timing.
Coordinated shooting uses the same conveyor direction as teleop RB (the negative of
``CONVEYOR_PRESET.FEED``); collection retains its separate conveyor output.

Adjacent feeding and intake-retraction phases now share one ``FeedRollers`` command for the entire
six-second stop. A parallel sequence waits three seconds before running ``StowIntake``; it does not restart
the shooter or readiness check. The former adjacent ``FeedRollers`` stages in Left Bump Old are also merged.
Standalone shooting stages retain their three-second windows. Initial readiness uses part of each window.
Moses 3 Cycle instead uses ``HubVolley`` at its first two stops: it waits up to three seconds for
initial readiness, then feeds continuously for three seconds and finishes immediately. A six-second overall
timeout bounds the command. The rack retracts during feeding using the saved shot retraction speed and hold
position; there is no separate retraction wait. The command stops its rollers when it completes or is interrupted.
This is timed feeding, not hopper-empty detection, and uses the same logic in simulation and on hardware.
Moses's second approach runs ``PrepareHubShot`` alongside the path to prepare hood and flywheel
without taking control of driving or feeding. It transitions directly into ``HubVolley`` without a fixed
``AutoAim`` wait. The routine ends immediately after ``Moses 2``: no final shooter preparation, aiming or
volley is scheduled, and the end-of-autonomous guard neutralizes outputs. Initial readiness still gates feeding. The first stop's one-second
preparation stage and all path geometry remain unchanged. Other autos retain their six-second combined stops.
The dashboard chooser defaults to the coordinated fender shot; the explicit ``none`` option remains available when no
autonomous action is desired. PathPlanner and Choreo routines explicitly seed odometry, while the standalone fender
option has no trajectory from which to obtain a starting pose and therefore requires an accepted absolute vision
alignment before it can aim or prepare the shooter.


Guidelines
----------

- Keep subsystem methods and primitive commands small.
- Put multi-subsystem behavior in command factory classes.
- Reuse the same commands in teleop and autonomous when possible.
- Prefer ``parallel``, ``deadline``, and ``sequence`` over custom state machines when standard command groups are
  enough.
- Put controller-specific wiring in ``RobotContainer``, not inside subsystem classes.

End of autonomous
-----------------

After the selected autonomous command finishes or is interrupted, Robot cancels remaining commands and
neutralizes drivetrain, shooter, intake roller, conveyor, both indexers, intake rack and hood. This shutdown
is reapplied after the scheduler while autonomous remains enabled, so defaults cannot resume aiming or
move the hood to stow. Exiting autonomous also performs the shutdown. Teleop bindings resume normally
after the mode switch; they are not subject to the autonomous stop guard.
