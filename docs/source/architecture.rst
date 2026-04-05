Architecture
============

This page gives a high-level map of how ``2026-Rebuilt`` is organized.

Use it when you need to understand where new logic belongs before editing the codebase.

Top-level layout
----------------

The main Java source tree is organized into a few major areas:

- ``commands/`` for command factories and coordinated robot behaviors
- ``constants/`` for runtime mode, subsystem configs, gains, and field/vision constants
- ``subsystems/`` for the reusable mechanism and sensor abstractions
- ``simulation/`` for robot-level simulation helpers and field interactions
- ``util/`` for shared helpers, feedforwards, tuning utilities, and PathPlanner support


Runtime flow
------------

The main runtime path is:

1. ``Robot`` sets up AdvantageKit logging and chooses the current mode
2. ``RobotContainer`` constructs subsystems, controller bindings, named commands, and autonomous choices
3. ``CommandScheduler`` runs commands and subsystem periodic methods
4. simulation hooks run only when the current mode is not ``REAL``

This means most robot behavior belongs in command factories and subsystem implementations, not in ``Robot`` periodic
methods.


Modes
-----

Runtime mode is controlled in ``constants/Constants.java``.

This repo supports:

- ``REAL`` for hardware execution on the robot
- ``SIM`` for desktop physics and sensor simulation
- ``REPLAY`` for AdvantageKit log replay

Many subsystem IO factories switch implementation based on that mode. That is why the repo can use the same subsystem
API in real hardware, simulation, and replay.


Subsystem and IO pattern
------------------------

Most mechanisms follow the same design pattern:

- a subsystem class owns behavior and telemetry
- an IO interface defines hardware-facing methods and loggable inputs
- real and simulated IO implementations sit behind that interface

Examples in this repo:

- ``Flywheel`` and ``FlywheelIO``
- ``PositionJoint`` and ``PositionJointIO``
- ``Vision`` and ``VisionIO``
- ``DigitalSensor`` and ``DigitalSensorIO``

This keeps mechanism logic independent from vendor-specific hardware code.


Commands and compositions
-------------------------

Primitive mechanism actions are exposed as command factories in the subsystem classes and in ``commands/`` packages.

This repo uses:

- subsystem-level primitive controls such as setpoint or voltage commands
- command factory classes like ``IntakeCommands`` and ``ShooterCommands`` for coordinated actions
- ``RobotContainer`` for button bindings and autonomous assembly

See :doc:`command-compositions` for concrete examples.


Autonomous structure
--------------------

Autonomous behavior is built from reusable commands rather than a separate control path.

This repo currently uses:

- PathPlanner named commands via ``NamedCommands.registerCommand(...)``
- dashboard autonomous selection through ``LoggedDashboardChooser``
- Choreo trajectory support for explicitly composed routines

The key rule is to reuse the same mechanism commands in teleop and auto wherever possible.


Simulation structure
--------------------

Simulation is not treated as an afterthought in this repo.

There are two simulation layers:

- subsystem- and mechanism-level simulation through mode-specific IO
- robot- and field-level simulation through ``simulation/`` helpers and visualization hooks

``RobotContainer`` and ``Robot`` explicitly guard simulation-only behavior using ``Constants.currentMode`` checks.


Practical guidance
------------------

- Put scheduler wiring, controller bindings, and auto selection in ``RobotContainer``.
- Put hardware-specific behavior behind IO interfaces.
- Put multi-subsystem behavior in commands, not subsystem internals.
- Add constants before hardcoding values in commands or subsystem logic.
- Prefer extending an existing subsystem pattern before creating a new architectural layer.
