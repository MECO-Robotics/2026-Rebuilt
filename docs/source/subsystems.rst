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


Field mapping
-------------

``FieldMap3d`` publishes the robot's live world model to AdvantageKit/AdvantageScope under ``FieldMap3d/*``. It logs:

- robot pose
- field boundary corners
- Hub pose
- AprilTag poses
- selected game-piece pose
- detected robot-obstacle centers and obstacle box corners

Use this view when debugging autonomous decisions, pathfinding, camera detections, and whether dynamic robot obstacles
line up with the field coordinate system.


Digital sensors and piece detection
-----------------------------------

Not every subsystem is a motorized mechanism.

This repo also includes:

- ``DigitalSensor`` for binary hardware inputs
- ``PieceDetection`` for target or game-piece style perception pipelines

These follow the same IO-driven architecture as the motor subsystems.

Game-piece shape mapping
~~~~~~~~~~~~~~~~~~~~~~~~

The game-piece camera service publishes a ``shape`` label for each tracked ball and a group's primary ``shape`` plus
``shape_counts`` in ``/data``. The current supported shape set is intentionally circle-only, so the robot treats missing
or old camera JSON as ``circle`` and logs the selected group's shape for debugging and future expansion.

The camera service also serves a browser dashboard on its root URL and an MJPEG stream at ``/video``. The web dashboard
shows the live processed image, visible balls, groups, yaw, and calibrated distance when distance data is available.

Game-piece range calibration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``PieceDetectionIOHttp`` can estimate game-piece range from the detected area using the calibration table in
``PieceDetectionConstants.GAME_PIECE_AREA_RANGE_SAMPLES``. To calibrate it:

1. Place a game piece at a tape-measured distance from the camera.
2. Run the game-piece camera service and read the detected group ``area`` value from ``/data`` or AdvantageKit logs.
3. Add a ``new AreaRangeSample(areaPixels, rangeMeters)`` entry for that measurement.
4. Repeat for several near, middle, and far distances.

When at least two samples exist, the robot interpolates range from area and clamps outside the measured area range.
Without two samples, it falls back to the camera-reported distance or the rough ``kDistance / sqrt(area)`` estimate.


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
