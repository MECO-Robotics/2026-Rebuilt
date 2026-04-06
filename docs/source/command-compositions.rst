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


Guidelines
----------

- Keep subsystem methods and primitive commands small.
- Put multi-subsystem behavior in command factory classes.
- Reuse the same commands in teleop and autonomous when possible.
- Prefer ``parallel``, ``deadline``, and ``sequence`` over custom state machines when standard command groups are
  enough.
- Put controller-specific wiring in ``RobotContainer``, not inside subsystem classes.
