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
absolute correction less than two seconds old. It uses a 0.9 m conservative
circular footprint: the 0.8509 m square bumper, 0.3 m rack extension, and margin.
Measure actual protrusions before hardware use. The deployed navgrid is
16.54 by 8.07 m at 0.3 m resolution. Verify its obstacles and field dimensions
against the event field: a planner cannot correct an inaccurate map.

There is **no real opponent detector** in this integration. The production source
explicitly supplies an empty obstacle snapshot. QuestNav and AprilTags provide
localization, not opponent detection. Simulated rerouting does not establish
hardware opponent avoidance or competition readiness.

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

Arbitrary PathPlanner autos cannot be paused and resumed safely. In this checkout,
several named scoring commands also own the drivetrain. Migrate such routines
explicitly into stopping objectives and mechanism-only arrival commands rather
than wrapping their existing marker timelines. Conventional autos retain their
existing event behavior.

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
