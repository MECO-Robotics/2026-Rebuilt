Simulation controls and physics
===============================

Simulation and REAL use the same ``RobotContainer.configureButtonBindings``.
Use the same pilot and copilot USB slots in the simulation Driver Station:
left stick drives, right stick turns, right trigger collects, left trigger ejects,
A aims and sets the shooter, and RB feeds after initial flywheel readiness.
X/Y and copilot presets retain their normal behavior. D-pad deployment alone
does not run the collection motors. No real gains or controller bindings are
changed by the simulation physics settings.

Practice session resets
-----------------------

Switch directly from enabled Auto to enabled Teleop to keep the robot pose,
remaining field and hopper fuel, airborne shots, and score. Disable or disconnect
the Driver Station to reset the practice field and return to the startup pose.
A disable between modes also resets the session. Starting an auto still uses
its selected path's starting pose. Reconnecting AdvantageScope does not reset it.

Fuel follows actuator behavior
-----------------------------

Pickup requires the rack near its deployed position and the intake, conveyor,
and both indexers commanded in collection directions with measured motion.
Shooting requires conveyor/indexer feeding outputs and measured motion, plus
flywheel speed. Disabled, stopped, or reversed transport cannot magically move
fuel just because a simulated intake/launch action was requested. Eject returns
held fuel to the field at an approximate 2 m/s, at most one ball per 0.08 seconds.
These rates are practice estimates, not measured hardware throughput.
``FieldSimulation/Transport/CanCollect``, ``CanFeed``, and ``CanEject`` report
whether the simulated transport conditions are currently satisfied.

Each projectile removes its estimated translational kinetic energy from the
simulated shooter wheel. The motor model then recovers speed. This replaces the
old optional constant projectile slowdown factor; the calibrated shot tables
and real motor gains are unchanged.

Wall and trench contact
-----------------------

MapleSim continues to simulate drivetrain mass, tire traction, field collisions,
ground fuel collisions, and airborne trajectories. The drivetrain plant uses
the same wheel radius as odometry. Rack and hood use motor/joint plants with
travel limits; rollers and shooter use motor plants.

``PracticeRebuiltArena.FIELD_CONTACT_FRICTION`` is 0.08 for static field panels,
independent of tire/carpet traction. This is an approximate smooth-panel contact
setting, not a measured friction coefficient. Walls still block penetration:
driving into them head-on should stop forward motion. Sliding along them should
lose less speed than with MapleSim's previous 0.6 panel setting. The local arena
also relocates MapleSim 0.4.0-beta's duplicated lower-left trench wall to the
missing upper-right location.

Limits of the model
-------------------

This is not a complete 3D mechanical simulation. Stored hopper fuel is counted
and visually packed, not individually simulated for compression or jams. The
planar drivetrain does not climb bumps, tip, or collide with overhead trench
structure; ramps remain passable. Tape grip, ball deformation, electrical faults,
and actual stopping distances still need hardware validation. Existing CAD
simulation and REAL shooter coordinate offsets remain separate; button mappings
are shared, but the display alone does not verify physical forward direction.

Verification
------------

``PracticeRebuiltArenaTest`` checks all four trench walls and compares free motion
with field-wall/trench sliding while asserting solid collisions.
``RobotContainerSimulationTest`` exercises the shared controls, pickup/eject,
autonomous indexers, readiness, and cancellation/disable shutdown.
``FuelTransportSimTest`` rejects stopped, reversed, and invalid transport inputs.
``FlywheelEnergyTest`` verifies that ball launches remove energy without reversing
or creating wheel motion. These tests supplement, rather than replace, robot testing.
