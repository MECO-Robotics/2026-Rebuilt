Experimental hybrid autonomous navigation
=========================================

Scope and selection
-------------------

``EXPERIMENTAL Hybrid Survey (neutral zone)`` is an opt-in chooser entry. Existing
competition autos and the default selection are unchanged. This is a navigation
survey with two arrival events, not a scoring auto. Its blue starting pose is
(7 m, 2 m, 0 degrees); the return objective is the same translation. Select the
correct alliance before enabling. Paths are flipped once at initialization;
recovery coordinates are already in the absolute blue-origin field frame.
No navigation transition resets odometry. Unknown alliance fails the start gate.

The survey requires field-aligned, connected localization and an accepted
absolute correction less than two seconds old. ``RobotNavigationFootprint`` uses
the team's measured 34.033 inch bumper length, 33.5 inch bumper width, and
42.141 inch total length with the intake extended only at the front. With the
bumper rectangle centered at the drivetrain origin (as configured in PathPlanner),
the front extent is 0.6381623 m. Its conservative circular radius is 0.7669803 m
plus 0.05 m margin, or 0.8169803 m. PathPlanner's bumper display and intake
extension length also reflect these measurements; the intake drawing retains
its previous width, which has not been remeasured. The deployed navgrid is
16.54 by 8.07 m at 0.3 m resolution. Verify its obstacles and field dimensions
against the event field: a planner cannot correct an inaccurate map. The team
reports that this grid has been verified on a full field, but whether its blocked
cells already include robot clearance is unknown. Static inflation remains
enabled until that padding convention is established; no blocked cells have
been removed to make an auto pass.

There is **no real opponent detector** in this integration. The production source
explicitly supplies an empty obstacle snapshot. QuestNav and AprilTags provide
localization, not opponent detection. Simulated rerouting does not establish
hardware opponent avoidance or competition readiness.

Editing in PathPlanner
----------------------

Open the ``2026-Rebuilt`` project and select
``EXPERIMENTAL Hybrid Survey (neutral zone)`` in the Autos list. The deployed
``.auto`` file sequences ``Hybrid Survey Out`` and ``Hybrid Survey Return``.
Edit their geometry in Paths and their order in the auto, then redeploy.
The robot reads that sequence from the file and runs it through the hybrid
supervisor; the chooser excludes the ordinary PathPlanner execution of this
same auto so there is only one survey option.

Keep Reset Odometry off. This survey accepts only sequential path commands,
including nested sequences, with marker-free paths ending at zero velocity.
Named commands, waits, parallel groups, Choreo, and empty missions are rejected
with a Driver Station error and a stop command. Mechanism actions require
explicit integration with the supervisor before adding them here.
``Hybrid/Survey/LastVisitedPath`` records each completed path's arrival.
This file does not enable hybrid recovery for other PathPlanner autos.

Competition auto migration
--------------------------

The ``Hybrid migration`` folder in PathPlanner contains eight ``HYBRID ...``
auto candidates and 47 separate path segments. The original autos remain
available. ``16th Note Auto`` is empty and has no route to migrate. The
stationary Fender option and the separately coded Choreo auto are unchanged.

The new command-tree adapter preserves sequential, parallel, deadline, race,
named-command, and wait structure. Every path leaf uses a hybrid supervisor.
Intake/spin-up commands in the surrounding group stay active through recovery;
they are not restarted with the follower. Named commands use fresh instances
for each occurrence. A drive failure never counts as a completed leaf: it
interrupts the complete auto, including active mechanisms, before advancing
to shooting. Localization is required before mechanisms start and throughout
the auto. There is a 30-second whole-auto deadline, with the existing bounded
recovery policy on each path. External cancellation stops the complete tree.
Concurrent navigation leaves, navigation inside a race, and navigation as a
non-deadline child are rejected because they could skip physical arrival.
WPILib also rejects simultaneous commands sharing subsystem requirements.

The candidate paths start and end at zero velocity; extra stops and recovery
will change cycle times and require simulation/hardware tuning. Original curve
geometry and authored rotation targets are preserved. The redundant
signal-only ``SpinIntake`` markers in copies of ``Left Double 1``,
``Left Trench Bump 0``, and ``Right Bump 0`` were removed: their parent deadline
already runs ``SpinIntake`` for the full drive.

``Left Bump 2``, both ``Bump Rush Depot`` routes, and ``Moses 3 cycle`` are
split at their existing waypoints into numbered segments. Each original cubic
Bezier span keeps its anchors and control points. Rotation targets are shifted
into each segment's coordinates; newly introduced stop headings interpolate
between the surrounding authored rotations along the shorter angular direction.
These stop headings and zero boundary speeds change trajectory timing.
The surrounding intake deadline stays active across the entire split route.

Moses uses explicit phases in its ``.auto`` file:

* Deploy intake for 0.5 seconds, matching the other autos' deployment interval.
* Drive segment 01, collect across 02--03, and spin up during approach 04.
* Run ``CoordinatedHubShot`` after measured arrival at the first shooting stop.
* Drive 05, collect across 06--07, spin up during 08, then run the second shot.
* Drive 09 and collect across 10--11.

This deliberately replaces the signal-only marker timeline with bounded
commands. The three intake ranges are rounded to existing waypoint boundaries.
The two AutoSpinUp/AutoAim/Agitate marker clusters become the existing
readiness-gated ``CoordinatedHubShot`` at waypoints 4 and 8. Its existing limits
are 2 seconds to become ready, 1 second of feed, and 3 seconds total. The source
contains two shooting clusters despite its name, so the migration retains two
shooting steps and a final collection leg. Intake activation positions and shot
timing therefore differ from the old markers and must be tuned on the robot.

All eight pass structural migration validation: every referenced path is
marker-free, stops at rest, and has unambiguous geometry. Tests also compose
each tree with the real named-command factories to check subsystem ownership.
Clearance is a separate, pending validation step. No clearance checks have been
disabled: the robot chooser still appends ``(BLOCKED)`` while those checks fail.
``Hybrid/Migration/<auto name>`` shows path migration status, and
``Hybrid/Readiness/<auto name>`` gives outstanding clearance reasons. Once the
clearance model is established, update it and redeploy to rerun those checks.

Clearance failures are relative to the current conservative model, not proof
that the original field-tested routes collide. Establish whether the navgrid
already represents allowed robot-center positions before changing its padding.
Then review exact path corridors with the deployed intake footprint. Do not
remove the self-crossing guard: split loops into ordered, unambiguous objectives
so recovery cannot skip a branch.

Run ``HybridMigrationTest`` to regenerate
``build/reports/hybrid/migration-readiness.tsv``. A retained snapshot is
available as :download:`the migration readiness report <_static/hybrid-migration-readiness.tsv>`.
The report separates structural migration PASS from clearance PENDING and lists
blue/red start, destination, and corridor failures. Structural validation does
not establish whole-auto cycle time or full-field hardware success.
The historical benchmark results in :doc:`hybrid-validation`
precede this migration and used the earlier 0.9 m footprint; they do not validate
these competition candidates.

Desktop whole-auto simulation
-----------------------------

The optional hybrid simulation profile runs all eight migrated autos without
waiting for the production clearance model. It is available only when both
``Constants.currentMode == SIM`` and WPILib reports simulation, and requires an
explicit opt-in. REAL and REPLAY cannot activate it, even if the opt-in is set.
Normal startup still uses the production clearance gate and normal field arena.

From the repository root on this workstation, launch the visual simulator::

  env JAVA_HOME=/Users/amadeusjackson/wpilib/2026/jdk MECO_HYBRID_SIM=true ./gradlew simulateJava

The opt-in also enables the Sim GUI by default. Set the desired alliance,
select ``SIM TEST | HYBRID <auto name>`` in ``Auto Choices``, and enable
Autonomous. Each enable places the physical simulated drivetrain and its
estimator at that auto's first path pose, flipped once for Red. Recovery never
resets the pose. Disable or switch to Teleop to interrupt the trial.

Connect AdvantageScope through NetworkTables 4 and import
``AdvantageScope Shooting and Intake.json`` from the repository root. Set the
custom-assets folder to this repository's ``sim`` directory. Inspect
``SmartDashboard/HybridSim`` for the selected auto, outcome, elapsed seconds,
completed/total paths, recovery attempts, failure reason, and production-budget
flag. Existing ``Hybrid/*`` telemetry provides detailed navigation state and
routes; ``FieldSimulation/*`` provides intake/fuel/score telemetry.

The profile is an **empty test arena**, with deterministic sparse fuel positions.
The field artwork may still show 2026 structures, but those structures have no
drive collision bodies in this profile. Navigation uses physical simulation
ground truth and a zero-margin, obstacle-free field-coordinate rectangle.
Real mechanism commands still run, including their own readiness checks.
An unobstructed direct-route planner replaces the production AD* map in this
fixture; separate AD* regression tests cover obstacle rerouting.
This profile cannot validate robot footprint clearance, collision avoidance,
vision alignment/drift, or production AD* behavior.

Speed and acceleration remain at the conservative production defaults. The
diagnostic state deadline is 30 seconds and whole-auto deadline is 180 seconds,
so slow segmented routes can be observed. A result exceeding the production
configuration's 30-second mission budget is flagged explicitly. ``COMPLETE``
means the command tree finished and all navigation leaves arrived; it does not
mean every shot scored or that competition timing is acceptable.

Every trial appends an outcome row to
``build/reports/hybrid/interactive-runs.csv``. To run the sixteen Blue/Red trials
headlessly, plus a disable-interruption check::

  env JAVA_HOME=/Users/amadeusjackson/wpilib/2026/jdk ./gradlew hybridSimTest

This dedicated task uses one full robot in a separate JVM, avoiding duplicate
native CAN device IDs in the ordinary test suite. It writes
``build/reports/hybrid/whole-auto-simulation.csv``. Its contract is a bounded,
classified outcome with completion/abort metrics; a passing harness test does
not mean the autos completed. The batch accelerates the simulation clock and
yields to native device workers; native asynchronous controllers can vary with
wall-clock scheduling. Use the interactive simulator to investigate those
outcomes before treating them as drivetrain tuning evidence.

The retained initial batch contains **16 safe aborts and zero complete autos**:
nine persistent recovery stalls, four state deadlines, two commanded stopping
corridor failures, and one measured stopping corridor failure. These are
unresolved simulation/control findings, not successful full-auto validation.
See :download:`the whole-auto simulation results <_static/hybrid-whole-auto-simulation.csv>`.
The regular regression suite continues to test controller recovery, path
structure, and mechanism sequencing separately.

Ownership and mission policy
----------------------------

``HybridAutoSupervisor`` exclusively owns the drivetrain and every arrival-action
requirement for the mission. Followers and actions are composed commands whose
lifecycle is run by the supervisor; they are never independently scheduled.
Disable, teleop cancellation, requirement conflicts, faults, and deadlines end
the active child and zero the output. Emergency stops intentionally bypass
ordinary output acceleration limiting.

``AutoMissionPlan`` contains named, stopping segments and one arrival action per
segment. Only marker-free, non-Choreo, non-self-crossing PathPlanner paths ending at zero velocity
are accepted. Arrival actions cannot require the drivetrain. They run once after
measured arrival; an interrupted action does not mark its objective complete and
is never automatically retried. Arrival pose and speed are rechecked while the
action runs; displacement aborts without retrying a potentially completed score.
A new autonomous enable starts a new mission.
Actions must finish within the state deadline, including their own readiness
checks; a command merely finishing is their declared success contract.

Arbitrary PathPlanner marker timelines cannot be paused and resumed safely.
Several named scoring commands also own the drivetrain. The competition
command-tree adapter runs those commands between independently validated
navigation leaves, after the follower has stopped and released its child
lifecycle. They are not supervisor arrival actions, and they cannot run in
parallel with a drive leaf. A command-tree scoring step uses its own existing
readiness checks; the supervisor's ACTION pose-hold check applies only to
``AutoMissionPlan`` arrival actions. Conventional autos retain their existing
event behavior.

Controller policy
-----------------

* ``NORMAL_PATH`` uses the actual PathPlanner ``FollowPathCommand`` and the
  project's ``AdvancedPPHolonomicDriveController``. Authored trajectories and
  rotation targets are retained; the common safety limiter can constrain output.
* Sustained cross-track error, along-track lag, or a stall enters
  ``GEOMETRIC_RECOVERY`` if the remaining corridor is safe. BLine starts at the
  measured pose, rejoins the next remaining polyline point, and completes the
  current stopping segment geometrically. The **next segment** starts a fresh
  authored PathPlanner trajectory. The interrupted timer is never restarted.
  This deliberately gives up the current segment's timed rotation/velocity
  profile during recovery; segment endpoints are the mandatory objectives. The
  tightest authored global/zone limits are applied throughout geometric recovery.
* A represented obstruction enters ``OBSTACLE_REROUTE``. The shared
  ``LocalADStarAK`` receives inflated static and dynamic bounds. It searches on
  its existing worker thread while output ramps toward zero. Detours are capped
  at 1 m/s to retain braking clearance through generated corners. Changed obstacle
  snapshots are refreshed at most every 0.5 s while waiting, without extending
  the planner deadline. Every candidate's
  continuous polyline, entry connection, and destination are checked before it
  is followed. Unsafe smoothed routes are rejected, even when AD* says they exist.
* Follower termination enters ``FINAL_ALIGNMENT`` only near the destination;
  premature timed completion instead requests geometric recovery. Measured
  translation, heading, linear speed, and angular speed must remain within
  tolerance for 0.2 s before the arrival action starts.
* ``COMPLETE`` means all arrival actions finished. ``SAFE_ABORT`` means no further
  objective is attempted. An unexplained persistent stall in recovery aborts;
  it does not repeatedly push the same field element. The initial stall recovery
  brakes for 0.75 s before constructing the retry from the then-current pose.

A common field-frame vector limiter preserves speed/acceleration bounds across
handoffs. Both commanded and measured stopping corridors are checked before
publishing the next command. Conservative clearance can abort a physically
recoverable route; stopping takes priority over repeated unsafe attempts.

Configuration
-------------

``AutoSafetyConfig`` validates immutable per-run limits. Survey thresholds are
read from ``Hybrid/Config/*`` tunables when the deferred command starts, not while
a mission is running. These are provisional hypotheses, not calibrated settings.
Competition candidates snapshot the configuration when the chooser is built.
The existing ``LoggedTunableNumber`` tuning-mode policy also applies. Normal
controller gains default to the drivetrain's 15 translation / 10 rotation P gains;
``Hybrid/Controller/*`` uses shared subscriptions across runs. BLine uses P gains
2 translation / 4 rotation / 3 cross-track. All require hardware calibration.

============================== =====================
Setting                        Initial value
============================== =====================
Translation speed/acceleration  2 m/s; 2 m/s²
Angular speed/acceleration      3 rad/s; 6 rad/s²
Cross-track trigger / release  0.25 m / 0.15 m
Severe cross-track error        0.5 m
Along-track lag / release       0.65 m / 0.39 m
Error debounce                 0.2 s
Stall trigger                  demand >0.8, measured <0.15 m/s for 0.4 s
Minimum normal dwell           0.5 s
Replan cooldown                0.5 s; new hazard inside it stops
Total recovery/replan attempts  4 per mission
State / planner / mission time  10 s / 2 s / 30 s
Arrival tolerances              0.10 m; 4°; 0.15 m/s; 0.15 rad/s
============================== =====================

``PathTrackingMonitor`` projects onto route geometry separately from the timed
reference. Position jumps larger than maximum travel plus 0.3 m per sample,
heading jumps larger than maximum rotation plus 0.3 rad, nonfinite state,
unhealthy localization, or a control interval outside (0, 0.1] s abort.
An internally consistent but wrong localization estimate cannot be detected
from odometry alone; the existing absolute-localization health gate is essential.

Obstacle contract and replay
----------------------------

``ObstacleSource`` returns a complete snapshot of field-frame AABBs with FPGA
monotonic timestamp, confidence, and field-frame velocity. Empty explicitly means
no detections; null means unavailable. No camera transform is guessed. A future
sensor adapter must validate its transform before producing this interface.

``NavigationSafety`` rejects stale (>0.5 s), future, nonfinite, out-of-field,
low-confidence (<0.7), oversized (>64 observations), or implausible (>15 m/s)
snapshots. Rejection fails closed. Bounds include robot footprint, 0.1 m sensor
uncertainty, and the velocity sweep over sensor latency plus a 0.5 s prediction
horizon. A disappeared detection is removed on the next valid snapshot; the
supervisor does not restart completed actions or cancel an otherwise safe detour.

``LoggedObstacleSource`` records complete observations through AdvantageKit
``processInputs`` and substitutes recorded snapshots during replay.
``LocalADStarAK`` retains its recorded route interface. End-to-end replay against
an actual match log has not been validated. Production obstacle logs currently
contain empty snapshots because there is no detector.

Telemetry and verification
--------------------------

See :doc:`hybrid-validation` for measured results, regressions, and retained raw data.

``Hybrid/*`` records state/reason, objective and pending actions, pose and target,
reference velocity, geometric error and timed lag, measured/commanded speeds,
recovery and AD* routes, attempts/replans/transitions, recovery/mission duration,
endpoint errors, and control-computation duration/overrun (excluding telemetry
publication). The benchmark separately times the full scheduler call. Existing drivetrain simulation
telemetry supplies physical ground truth. ``Hybrid/Obstacles`` stores timestamped
raw observations for age/confidence reconstruction.

Use the installed WPILib Java 17 JDK. This workstation's default Java 25 cannot
configure Gradle 8.11's test task. For example::

  env JAVA_HOME=/Users/amadeusjackson/wpilib/2026/jdk ./gradlew test
  env JAVA_HOME=/Users/amadeusjackson/wpilib/2026/jdk ./gradlew test --tests 'frc.robot.navigation.*'

The suite contains geometry and output-bound tests, real follower lifecycle
checks, a deterministic acceleration-limited plant benchmark, native MapleSim
motor/tire/inertia testing with a physical impulse and separate estimator, and
actual asynchronous AD* tests. The deterministic benchmark writes
``build/reports/hybrid/physical-benchmark.csv``. It compares the unchanged timed
follower against the supervisor using the same paths, robot config, plant,
initial conditions, and seeded disturbances. Success uses measured endpoint,
heading, and speed, not command termination. This simplified plant is not a
calibrated contact model. AD* tests explicitly label their ideal kinematic plant.

The moving-opponent suite uses continuously moving trajectories with seeded
positions, directions, speeds, timing, and bounded acceleration. Some cases are
expected to abort safely when an opponent occupies the footprint or stopping
corridor. Successful passage and safe abort must be reported separately.

Dependencies and source audit
-----------------------------

PathPlanner remains pinned at 2026.1.2; GradleRIO remains 2026.2.1. The new vendordep
pins BLine-Lib v0.9.2 (source commit
``c88adfd95f1006be9c19d4e3d89a326db8ee6012``). No mixed-language runtime is introduced.
BLine's default pose-reset callback is a no-op; this integration never supplies
``withPoseReset``. ``withTRatioBasedTranslationHandoffs(true)`` enables geometric
translation progress. Runtime paths supply explicit global defaults because
BLine otherwise attempts to read its own deployed configuration file.

Source behavior checked: PathPlanner's follower ends on time (including invalid
total time); BLine completion omits final speed; PathPlanner smoothing can cross
inflated safety margins. The supervisor adds independent checks for each.
References: `PathPlanner pathfinding <https://pathplanner.dev/pplib-pathfinding.html>`_
and `BLine v0.9.2 <https://github.com/edanliahovetsky/BLine-Lib/tree/v0.9.2>`_.

Hardware gates
--------------

Before competition adoption, verify the field/navgrid, footprint with mechanisms,
absolute-pose freshness, alliance transforms, motor response and braking distance,
controller gains, and false-positive thresholds. Test disabled/teleop interruption
on blocks, then closed-field low-speed displacement/stall trials before increasing
speed. Recovery is not a substitute for localization correction. Real opponent
avoidance requires a validated detector and latency/uncertainty characterization.
No physical robot testing has been performed for this change.
