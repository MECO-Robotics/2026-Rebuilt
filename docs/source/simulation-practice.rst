Simulation controls and physics
===============================

Simulation and REAL use the same ``RobotContainer.configureButtonBindings``.
Use the same pilot and copilot USB slots in the simulation Driver Station:
left stick drives, right stick turns, right trigger collects, left trigger ejects,
A aims and sets the shooter, and RB feeds after initial flywheel readiness.
X/Y and copilot presets retain their normal behavior. D-pad deployment alone
does not run the collection motors. No real gains or controller bindings are
changed by the simulation physics settings.

Elastic testing dashboard
-------------------------

Import ``dashboards/remy-sim-testing.json`` with Elastic's **File → Open Layout**
for Driving, Shooter Testing, and Auto Testing tabs. Connect to localhost for
simulation. See ``dashboards/README.md`` for topic mappings, status limitations,
and import instructions. The updated drivetrain telemetry publishes the standard
``/Pose/Robot`` Field2d entry alongside its existing pose topic. Use AdvantageScope
for the 3D robot and fuel view.

Practice session resets
-----------------------

Switch directly from enabled Auto to enabled Teleop to keep the robot pose,
remaining field and hopper fuel, airborne shots, and score. Disable or disconnect
the Driver Station to reset the practice field and return to the startup pose.
A disable between modes also resets the session. Starting an auto still uses
its selected path's starting pose. Reconnecting AdvantageScope does not reset it.

Shooting status and direction parity
-----------------------------------

``Shooter/Feeding`` is an output, not a force-feed control. Auto and RB publish
it to SmartDashboard from their actual feeding decisions and clear it when
feeding stops. Changing the displayed boolean does not command a motor.

``IndexerFeedDirection`` supplies the existing signed voltage requests to both
commands and simulation: manual RB uses the presets, coordinated auto uses their
negatives. The launching command passes its direction explicitly, including in
Test mode. Simulation still checks moving motors and conveyor direction; it does
not treat an illuminated dashboard indicator as permission to launch.

Simulation aiming and projectiles use the original CAD-aligned +X yaw (0 degrees).
REAL and REPLAY retain their existing 180-degree shooter yaw. The attempted shared
180-degree simulation offset was reverted after the model faced the wrong way.
No path headings, real motor directions, or readiness thresholds were changed.

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
and actual stopping distances still need hardware validation. Simulation aiming and projectile yaw use the original 0-degree offset.
CAD import and intake-extension geometry remain separate from the control frame;
the display alone does not verify physical forward direction. Talon motor plants
and per-mechanism battery estimates do not reproduce every real current-limiting
or simultaneous-load condition.

Verification
------------

``PracticeRebuiltArenaTest`` checks all four trench walls and compares free motion
with field-wall/trench sliding while asserting solid collisions.
``RobotContainerSimulationTest`` exercises the shared controls, pickup/eject,
autonomous indexers, readiness, and cancellation/disable shutdown.
``FuelTransportSimTest`` rejects stopped, reversed, and invalid transport inputs.
``FlywheelEnergyTest`` verifies that ball launches remove energy without reversing
or creating wheel motion. These tests supplement, rather than replace, robot testing.

PathPlanner System Check auto
-----------------------------

The ``System Check`` chooser option follows its authored path, deploys the rack
at the ``DeployIntake`` marker, and runs the intake, conveyor, and both indexers
inside the ``SpinIntake`` marker interval. The zoned event ends at its authored
end position; path cancellation also ends its mechanism commands.

After the path ends, ``HubVolley`` aims and prepares for up to three seconds,
then feeds for three seconds if initial readiness is achieved, with a six-second
overall bound. A readiness failure ends without feeding. Aiming and shooting are
sequential auto stages rather than drivetrain-owning path events. The route,
rotation targets, and driving constraints are unchanged. This is a driving and
shooting autonomous routine, separate from the interlocked Test-mode system check.

When these files are changed outside PathPlanner, reopen the project before
editing or saving its path. Saving an older in-memory copy can restore marker
names without their attached commands. Both intake markers must retain a Named
Command action matching their name; a marker label alone does not run a command.
``SpinIntake`` must retain its end position to stop collection before shooting.
After saving, restart desktop simulation so it loads the updated deploy files.

Practice score counters
-----------------------

Elastic's Driving tab uses ``FieldSimulation/Scoring/Auto``, ``Teleop``, and
``Total`` under ``/AdvantageKit/RealOutputs/``. A successful shot counts once in
its enabled launch mode, even if it lands after the auto-to-teleop transition.
Total is Auto + Teleop. Test-mode shots are excluded. Disable or Driver Station
disconnect resets the session; direct enabled auto-to-teleop preserves it. The
legacy ``FieldSimulation/SuccessfulScoreCount`` topic mirrors Total. These are
practice hit counts, not MapleSim active-hub match points or real-robot scores.
