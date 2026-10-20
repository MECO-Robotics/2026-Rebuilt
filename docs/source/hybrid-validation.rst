Hybrid validation record
========================

Validated on 2026-10-20 in the local Java checkout, branch
``fix/auto-paths-get-hit``. Parent revision: ``2a8c76ea4b2567bf177034217b2715a5e3425295``.
See :doc:`hybrid-navigation` for the implemented architecture and operating limits.

Verified checks
---------------

* Baseline suite passed before implementation with the installed WPILib Java 17.
* Full final regression: **115 tests passed, zero failed/skipped**, including 42
  navigation tests and existing robot-container, shooter, vision, and mechanism tests.
* The final recovery-duration telemetry addition was followed by a passing focused
  supervisor suite and regeneration of the 552-trial benchmark.
* Actual Java PathPlanner 2026.1.2, BLine v0.9.2, LocalADStarAK, HAL scheduler,
  and native MapleSim APIs compiled and executed. No physical robot was used.
* Complete site HTML rendered, but strict Sphinx ``-W`` found 10 existing warnings
  in other pages (heading underlines, list indentation, and toctree membership).
  Both new pages then passed an isolated strict Sphinx build with zero warnings.
* Existing MapleSim configuration emits a warning for tire friction coefficient
  2.9. It was preserved; physical-model calibration remains necessary.

Deterministic disturbance comparison
------------------------------------

Each disturbance class uses 30 seeds across three route/heading shapes and both
alliance frames. Nominal has six combinations. Both followers use the same
robot config, default translation/rotation gains (15/10), route constraints, and
acceleration-limited plant. Impulses change physical velocity, not reported pose.
Both receive up to one second of passive braking after termination. No extra
position controller is added to the baseline.

The comparison evaluates the complete hybrid policy, including its output limiter
and final alignment, not only BLine versus PathPlanner in isolation. The simple
plant's 2 m/s² vector acceleration is deliberately restrictive; it is not the
calibrated drivetrain physics. Consult the separate MapleSim comparison below.

.. csv-table:: Completion and elapsed time
   :header: "Scenario", "Baseline success", "Hybrid success", "Baseline mean seconds", "Hybrid mean seconds", "Hybrid mean recovery seconds"
   :widths: 25, 15, 15, 15, 15, 15

   "NONE", "4/6", "6/6", "3.200", "3.883", "0.593"
   "LATERAL_PUSH", "0/30", "30/30", "3.786", "7.732", "5.763"
   "FORWARD_PUSH", "0/30", "30/30", "3.416", "6.491", "4.062"
   "BACKWARD_PUSH", "0/30", "30/30", "4.038", "5.289", "2.605"
   "NEAR_END_PUSH", "0/30", "30/30", "3.633", "7.431", "4.841"
   "MULTIPLE_PUSH", "0/30", "30/30", "4.100", "8.022", "6.052"
   "SLIP", "0/30", "30/30", "3.687", "6.698", "3.971"
   "STALL_HALF", "10/30", "30/30", "4.067", "4.953", "2.925"
   "STALL_ONE", "0/30", "30/30", "3.600", "4.899", "2.873"
   "PERMANENT_STALL", "0/30", "0/30", "3.113", "3.609", "0.000"

All 246 feasible hybrid trials completed; all 30 permanently blocked trials
aborted. No hybrid field-clearance violations were recorded in this empty-field
plant. Successful hybrid trials had worst final translation error 0.0393 m,
heading error 0.844 degrees, and speed 0.091 m/s. The largest timed hybrid scheduler
call in this desktop run was 2.801 ms. This excludes real robot I/O and active
match logging overhead, and is not a roboRIO timing guarantee.

RMS and 95th-percentile cross-track error, maximum deviation, endpoint/heading/speed,
recovery time, total elapsed time, attempts, transitions, clearance counts, and
per-trial peak scheduler time are retained in
:download:`the complete 552-row CSV <_static/hybrid-physical-benchmark.csv>`.
Recovery seconds means the last recovery episode to measured arrival; aborted
runs have no completed recovery duration. RMS is calculated over each controller's
own elapsed run and cannot by itself establish better tracking when durations differ.

There are regressions: the nominal hybrid run is slower; forward-displacement
RMS error is higher than baseline despite successful eventual arrival. Recovery
increases completion reliability in this test envelope, not speed or every error
metric. There is no claim that all competition autos improve.

MapleSim and obstacle checks
----------------------------

The native MapleSim sustained-push test applies bounded force at each physics
substep, requires an actual geometric-recovery transition, and verifies final
physical arrival. A separate impulse smoke test retains an independent estimator
corrected by a synthetic absolute sensor.

.. csv-table:: Curved 4 m/s MapleSim trial (one trial per controller)
   :file: _static/hybrid-maple-high-speed.csv
   :header-rows: 1

Both reach the precision envelope, but the hybrid takes about twice as long and
enters recovery once on this high-speed curve. This is a demonstrated nominal
performance regression requiring further tuning before expanding beyond the
conservative survey. The synthetic absolute sensor and empty arena are not an
end-to-end camera/QuestNav validation.

Actual asynchronous AD* tests complete viable static detours, including a 0.65 m
footprint, and reject an impassable wall within the two-second planner deadline.
Tests cover obstacles appearing during BLine, disappearing during replanning,
and multiple obstacles. Among 30 seeded acceleration-bounded moving-opponent
trials, 14 complete and 16 abort safely. Every trial reaches a bounded terminal
state with stopped output and correct action count. These use an explicitly ideal
kinematic plant: they verify that commands do not drive through unsafe geometry,
not that a moving opponent cannot collide with a stopped robot.

Failure handling and limitations
--------------------------------

Regression tests cover invalid/null/throwing localization, discontinuous correction,
requirement conflicts, disable/cancellation, interruption during an arrival action,
drift during a long action, invalid obstacle snapshots, and missing/corrupt replay
snapshots. Two-segment actions execute exactly once in order. Self-crossing and
embedded-marker paths are rejected rather than silently skipping objectives.

Bugs found and fixed included missing BLine global configuration, output publication
before clearance checking, unsafe exception boundaries, leaked command composition
on constructor rejection, action completion after displacement, premature abort on
one-second obstructions, repeated tunable subscriptions, and overly fast obstacle
corner approaches. The bounded detour speed is now 1 m/s.

Unverified: full recorded-match replay, real perception/transform calibration,
real braking/traction and timing, scoring-autonomous migration, and competition
readiness. Recovery uses the current segment's remaining polyline and endpoint;
it does not preserve that interrupted segment's intermediate timed rotation
profile. Precise intermediate objectives must be separate stopping segments.
No opponent detector, odometry reset, automatic scoring retry, or global competition
auto replacement was introduced.

Artifacts and dependency changes
--------------------------------

Only BLine-Lib v0.9.2 was added; existing dependency versions were retained.
Exact source/artifact changes (generated BuildConstants are excluded)::

  README.md
  docs/source/index.rst
  docs/source/hybrid-navigation.rst
  docs/source/hybrid-validation.rst
  docs/source/_static/hybrid-physical-benchmark.csv
  docs/source/_static/hybrid-maple-high-speed.csv
  vendordeps/BLine-Lib.json
  src/main/java/frc/robot/RobotContainer.java
  src/main/java/frc/robot/subsystems/drive/CommandSwerveDrivetrain.java
  src/main/deploy/pathplanner/paths/Hybrid Survey Out.path
  src/main/deploy/pathplanner/paths/Hybrid Survey Return.path
  src/main/java/frc/robot/navigation/AutoMissionPlan.java
  src/main/java/frc/robot/navigation/AutoSafetyConfig.java
  src/main/java/frc/robot/navigation/HybridAutoSupervisor.java
  src/main/java/frc/robot/navigation/HybridAutos.java
  src/main/java/frc/robot/navigation/LoggedObstacleSource.java
  src/main/java/frc/robot/navigation/NavigationOutput.java
  src/main/java/frc/robot/navigation/NavigationSafety.java
  src/main/java/frc/robot/navigation/ObstacleSource.java
  src/main/java/frc/robot/navigation/PathTrackingMonitor.java
  src/test/java/frc/robot/navigation/AutoMissionPlanTest.java
  src/test/java/frc/robot/navigation/HybridAutoSupervisorTest.java
  src/test/java/frc/robot/navigation/HybridMapleSimulationTest.java
  src/test/java/frc/robot/navigation/HybridMissionLifecycleTest.java
  src/test/java/frc/robot/navigation/HybridPathfindingTest.java
  src/test/java/frc/robot/navigation/NavigationOutputTest.java
  src/test/java/frc/robot/navigation/NavigationSafetyTest.java
  src/test/java/frc/robot/navigation/ObstacleReplayTest.java
  src/test/java/frc/robot/navigation/PathTrackingMonitorTest.java
