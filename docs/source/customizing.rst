Customizing
===========

Adapt this repository to your robot and workflow.

Good customization changes the robot-specific parts without breaking the reusable structure.


Safe first changes
------------------

Start with these before you change architecture:

- hardware IDs and buses
- gains, limits, and setpoints
- controller bindings
- autonomous selections
- mechanism naming and dashboard labels

These usually belong in ``constants/`` or ``RobotContainer``.


Common repo-specific customization points
-----------------------------------------

Hardware and constants
~~~~~~~~~~~~~~~~~~~~~~

Update:

- ``constants/subsystems/`` for subsystem-specific configs and gains
- ``constants/drive/`` for drivetrain setup
- ``constants/vision/`` for camera names, transforms, and filtering
- ``constants/sensors/`` for digital sensor configuration

Controller behavior
~~~~~~~~~~~~~~~~~~~

Most operator behavior is customized in ``RobotContainer``.

This includes:

- default commands
- button bindings
- co-pilot controls
- autonomous chooser options
- named commands used by auto tools

Mechanism behavior
~~~~~~~~~~~~~~~~~~

If an existing mechanism pattern fits your hardware, customize its constants before rewriting the subsystem.

Examples:

- use different presets in ``ShooterCommands``
- change rack or hood limits in subsystem constants
- swap SparkMax and TalonFX IO factories while keeping the same subsystem API

Simulation and visualization
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This repo contains real simulation support. Customize it only if you actually need simulation behavior to match the
robot more closely.

Examples:

- robot visualization assets
- drivetrain simulation parameters
- intake and launched game-piece simulation
- vision simulation transforms and noise models


When to restructure code
------------------------

Restructure only when constants and command wiring are no longer enough.

Good reasons:

- a mechanism no longer fits ``Flywheel`` or ``PositionJoint``
- one subsystem owns too many unrelated responsibilities
- a new hardware model requires a different IO implementation shape
- simulation needs a genuinely different integration path

Bad reasons:

- preferring a different style
- moving files before robot behavior is stable
- introducing new abstractions before the current ones are understood


Recommended process
-------------------

1. Change constants first.
2. Change ``RobotContainer`` bindings second.
3. Change command compositions third.
4. Change subsystem internals only when the first three are insufficient.
5. Add new abstraction layers only when the existing patterns clearly do not fit.


Warning
-------

Avoid unnecessary customization early. The fastest path is usually to keep the existing subsystem patterns and only
change configuration, commands, and bindings.
