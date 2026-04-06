Subsystems
==========

This repository is built around a small set of reusable subsystem patterns.

Choose the closest existing pattern before creating a new subsystem type.

Flywheel
--------

Use ``Flywheel`` for mechanisms controlled primarily by velocity or voltage.

Examples in this repo include:

- shooter flywheel
- indexers
- conveyor
- intake roller

This pattern works well when the mechanism behaves like a spinning wheel or roller and does not need independent
position control.


PositionJoint
-------------

Use ``PositionJoint`` for mechanisms that need position control, profile constraints, or bounded motion.

Examples in this repo include:

- hood
- intake rack

This pattern is intended for pivots, elevators, racks, and similar mechanisms where position is the main control
target.


Vision
------

Use ``Vision`` for pose-estimation and target-observation pipelines.

The vision subsystem in this repo is designed to:

- collect observations from one or more vision sources
- filter and validate those observations
- forward accepted measurements into drivetrain pose estimation

The hardware abstraction is handled through ``VisionIO`` implementations for Limelight, PhotonVision, QuestNav, and
simulation combinations.


Digital sensors and piece detection
-----------------------------------

Not every subsystem is a motorized mechanism.

This repo also includes:

- ``DigitalSensor`` for binary hardware inputs
- ``PieceDetection`` for target or game-piece style perception pipelines

These follow the same IO-driven architecture as the motor subsystems.


Drive subsystem
---------------

The drivetrain is more specialized than the reusable mechanism patterns above.

In this repo, the drivetrain is implemented separately as ``CommandSwerveDrivetrain`` because it owns:

- odometry
- vision measurement integration
- PathPlanner and Choreo interactions
- simulation pose synchronization
- drive-specific telemetry

Treat drivetrain changes as higher-risk than changes to isolated mechanism subsystems.


How to choose a pattern
-----------------------

Ask these questions first:

- Is the mechanism fundamentally velocity-controlled? Use ``Flywheel``.
- Is the mechanism fundamentally position-controlled? Use ``PositionJoint``.
- Is it just reporting state from hardware? Use a sensor-style subsystem.
- Does it affect global robot motion or pose estimation? It may need a specialized subsystem.


Guidelines
----------

- Reuse ``Flywheel`` and ``PositionJoint`` aggressively.
- Keep hardware details in the IO layer, not in the subsystem API.
- Add a new subsystem pattern only if the control model is genuinely different.
- Favor multiple small subsystems over one oversized subsystem with mixed responsibilities.
