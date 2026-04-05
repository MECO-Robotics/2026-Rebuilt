Mechanism Types
===============

This project includes two main reusable mechanism patterns: ``Flywheel`` and ``PositionJoint``.

Pick the one that matches what your mechanism is actually supposed to control before you start copying constants or tuning gains.


Use ``Flywheel`` when
---------------------

Choose ``Flywheel`` for mechanisms where the main goal is to control speed, not hold a physical position.

Common examples:

- shooter wheels
- intake rollers
- feeder rollers
- conveyors

``Flywheel`` usually fits mechanisms that:

- spin continuously
- do not need an absolute encoder offset
- are primarily commanded in velocity or voltage
- do not need gravity compensation

In this codebase, ``Flywheel`` config is simpler:

- motor CAN IDs
- motor inversion
- gear ratio
- current limit
- CAN bus

Its gains are also velocity-focused: ``kP``, ``kI``, ``kD``, ``kS``, ``kV``, ``kA``, plus acceleration and tolerance values.


Use ``PositionJoint`` when
--------------------------

Choose ``PositionJoint`` for mechanisms where the main goal is to move to and hold a position.

Common examples:

- arms
- pivots
- wrists
- elevators

``PositionJoint`` usually fits mechanisms that:

- have a meaningful physical position range
- need motion limits
- need a known sensor zero or offset
- may need gravity compensation
- are commanded to setpoints such as angles or heights

In this codebase, ``PositionJoint`` config includes everything a ``Flywheel`` uses plus:

- gravity model
- encoder type
- encoder ID
- encoder offset

Its gains also include position-specific fields such as ``kG``, max velocity, max acceleration, min position, max position, tolerance, and a default setpoint.


Quick comparison
----------------

- ``Flywheel``: "spin at this speed"
- ``PositionJoint``: "move to this position"
- ``Flywheel``: simpler hardware config, usually no external position reference
- ``PositionJoint``: more setup because closed-loop motion depends on sensor alignment and safe travel limits
- ``Flywheel``: best for continuous rotation mechanisms
- ``PositionJoint``: best for bounded joints and linear mechanisms


How to choose
-------------

Ask this question:

Is failure mostly about wrong speed, or mostly about wrong position?

- If wrong speed is the main problem, use ``Flywheel``.
- If wrong position is the main problem, use ``PositionJoint``.

If a mechanism can crash into hard stops, over-rotate, or must hold a specific angle or extension, it should usually be modeled as ``PositionJoint``.


Where to look next
------------------

- To configure hardware IDs and ports, continue to :doc:`configure-robot`.
- To inspect the shared subsystem patterns directly, see :doc:`subsystems`.
