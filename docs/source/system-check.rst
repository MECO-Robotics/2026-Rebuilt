Safe Automated System Check
===========================

The automated system check is an unloaded, on-blocks Test-mode sequence for pit diagnosis. It checks every currently
instantiated drivetrain, intake, feed, hood, shooter, and vision device while recording the result and measurements in
the normal AdvantageKit log. It takes about 85 seconds after Start, including the three-second motion countdown.

The normal pit sequence never launches fuel. The separately armed controlled-area shot is a practice-field procedure,
not a pit procedure.


Before enabling
---------------

#. Put the robot securely on stable blocks with all four wheels clear of the floor.
#. Remove **all fuel** and clear the intake, conveyor, indexers, hood, and shooter.
#. Keep all people, tools, cables, and loose clothing outside the mechanism and wheel envelopes.
#. Assign one operator to Driver Station Disable. Do not attach to FMS.
#. Install a charged battery. Start is blocked below 11.5 V.
#. Connect AdvantageScope over NT4 and import ``AdvantageScope Pit System Check.json`` from the repository root.
#. In Driver Station, select **Test**, then enable. Normal driver/copilot bindings and joystick drive are gated off in
   Test mode.

In ``Pit Check Controls``, set ``SetupConfirmed`` and ``Armed`` true only after every item above is true. Set ``Start``
true once. The software consumes that rising edge, resets it to false, and gives a three-second countdown before motion.
Press Driver Station Disable or set ``Abort`` true at any time to neutralize the robot.


Sequence and expected motion
----------------------------

The run samples preflight health for three seconds; points the swerve modules forward, sideways, and in X-lock; drives
the suspended wheels forward and reverse at 0.5 m/s; moves the intake rack through 0.00, 0.13, 0.30, and 0.00 m; tests
the intake roller, conveyor, bottom indexer, and top indexer separately at 2 V; moves the hood to 0.020 and back to
0.000 rotations; holds the shooter at 10 RPS; exercises the unloaded acquisition and feed paths; observes vision for
five seconds; and verifies neutral output for three seconds.

Mechanism and swerve stages run for three to five seconds. A mechanism failure is recorded and later independent
checks continue. A failed individual mechanism causes a related coordinated-path check to be ``SKIPPED``. After the
last stage, the command continues owning every subsystem with the rollers and drivetrain stopped and the rack and hood
at stow until the robot is disabled or leaves Test mode.


Status meanings
---------------

``PASS``
   The measured value met its threshold.

``WARNING``
   The check was usable but needs attention, such as vision connected without an accepted tag observation, a motor
   above 70 C, or CAN utilization above 80 percent.

``FAIL``
   A connection, position, velocity, follower agreement, vendor-fault, or timeout requirement failed. Independent
   stages continue.

``SKIPPED``
   A combined motion was withheld because an individual prerequisite failed.

``ABORTED``
   A safety condition or operator action stopped and neutralized the entire check.

``AWAITING_CONFIRMATION``
   The controlled-area shot fired and needs the operator to select Made or Missed.

An abort occurs immediately if enabled Test mode or Driver Station connection is lost, FMS attaches, the robot is
e-stopped or browned out, voltage falls below 9.5 V, a new CAN bus-off occurs, a measurement is non-finite, a motor
reaches 85 C, or a mechanism stays near its current limit without moving for 250 ms. Treat an abort as a real safety
event: disable, inspect the robot and log, and correct the cause before restarting Test mode.


AdvantageScope and logs
-----------------------

The prepared layout has separate ``Pit Check Controls``, ``Passive Health``, ``Reports & CAN``, and
``Controlled-Area Shot`` tabs. The controls, live state, progress, blocking reason, result explanations, current,
normalized velocity, temperature, connectivity, configured device/CAN IDs, vendor faults, and status of all three CAN
networks are published below ``/SystemCheck``. Complete run results, run ID, thresholds, measurements, interruptions,
vision alignment age, failure timing, and passive health are also recorded below ``SystemCheck/...`` in the
AdvantageKit WPILOG. The preflight snapshot records both controller connections as warnings; the AdvantageScope
controls remain usable if a pit controller is intentionally disconnected.

When a normal run completes or aborts, the robot also creates a standalone HTML report and matching JSON and CSV
files. In the ``Reports & CAN`` tab, copy ``Report/IndexUrl`` into a web browser while the robot program is running.
The addresses are:

* Real robot: ``http://roborio-8324-frc.local:5805/`` (or ``http://10.83.24.2:5805/`` if mDNS is unavailable)
* Desktop simulation: ``http://localhost:5805/``

The HTML page shows every stage in sequence, including PASS, WARNING, FAIL, SKIPPED, ABORTED, and NOT_RUN results;
the final explanation and measurements; when the failing condition began; when the failure was declared relative to
the stage and complete run; failed prerequisites; the final hardware snapshot; and CAN bus error counters. Use the
links at the top to download the complete JSON report or a stage-oriented CSV. ``LatestHtmlUrl``, ``LatestJsonUrl``,
and ``LatestCsvUrl`` point directly to the newest run. A write or web-server problem appears in ``Report/Error``.

Reports remain in ``/home/lvuser/system-check-reports`` on the roboRIO and ``build/system-check-reports`` in desktop
simulation. The standalone report is a diagnostic summary, not a replacement for the high-rate WPILOG. Download the
WPILOG through AdvantageScope when the complete time history is needed.

CAN neighbor localization is only as accurate as the recorded physical yellow/green wire order. The initial inventory
in ``SystemCheckCanTopology.java`` is deliberately marked ``PHYSICAL_ORDER_VERIFIED = false``. Before relying on the
suggested upstream/downstream connector, trace each of the rio, ``MECO 2``, and ``MECO CANIvore`` loops on the real
robot, reorder the devices in that file to match the wire path, and set the flag true. Until then, reports label every
neighbor suggestion as unverified. Even with a verified map, a bus-off without disconnected devices cannot identify a
single controller; inspect the report's CAN error counters and WPILOG in that case.

Passive motor, encoder, gyro, vision, temperature, and vendor-fault information is refreshed at 2 Hz in every robot
mode. This low-rate diagnostic polling avoids adding those signals to the high-rate CAN path.

In desktop simulation, the ``Simulation Faults`` tab can inject an intake motor disconnect, frozen joint encoder,
stalled high-current intake, 90 C shooter, missing vision, or 9.0 V battery. These switches have no effect on a real
robot. Clear every injection before using simulation for ordinary mechanism testing.

Simulation uses the QuestNav/AprilTag simulator for vision and MapleSim's physical module states for the swerve
checks. The REV intake-rack plant can oscillate slightly around a target; an error from 0.02 through 0.03 m is reported
as ``WARNING`` in simulation so the coordinated-path test can continue. The real robot still uses the original 0.02 m
passing requirement.

Tune limits in ``SystemCheckConstants.java`` only after examining several known-good real-robot logs. Do not raise an
abort limit merely to make a failing check pass.


Controlled-area one-fuel shot
-----------------------------

This procedure is forbidden in the pit. Move to a controlled practice field, leave Driver Station in enabled Test
mode, select the correct alliance, place the robot at a calibrated hub distance, and establish a valid field pose.
The unloaded system check must already have passing drivetrain, hood, shooter, conveyor, and indexer results.

#. Clear the entire firing area and set the controlled-area ``Armed`` and ``AreaClearConfirmed`` controls true.
#. Manually load **exactly one fuel**, then set ``ExactlyOneFuelConfirmed`` true.
#. Set the controlled-area ``Start`` true.
#. The robot uses the normal pose-based alliance-hub aim and readiness gates for at most five seconds.
#. Once ready, it holds aim, hood, and flywheel while feeding for three seconds, then stops and stows.
#. Select ``Made`` or ``Missed`` to finalize the result.

There is no possession sensor. Software cannot verify that only one fuel is loaded, and the three-second feed can
launch every fuel accidentally left in the robot. It also cannot determine whether the shot scored; Made/Missed is a
manual observation.

The controlled-area ``Armed`` control is separate from the unloaded pit-check arm and automatically resets when an
accepted shot starts. A second shot therefore requires an intentional re-arm.


Validation and maintenance
--------------------------

Run Java tests with the WPILib Java 17 toolchain::

   export JAVA_HOME=/Users/amadeusjackson/wpilib/2026/jdk
   export PATH="$JAVA_HOME/bin:$PATH"
   ./gradlew test

Before relying on the sequence at an event, complete three consecutive unloaded runs on blocks. Confirm the physical
direction and CAN ID of every mechanism against :doc:`shooting-intake-checklist`, disconnect one device at a time to
verify identification, and press Disable once during each mechanism group to confirm immediate neutral output. Only
perform the projectile check in the controlled practice area with exactly one fuel.


Diagnosing an intake rack that does not move
-------------------------------------------

The expected deployment device is a NEO/SPARK MAX at **CAN 21** on the roboRIO bus, using its internal encoder.
Verify that identity in REV Hardware Client and the wiring before changing code. Intake rollers are separate devices;
identify them from the current configuration rather than changing their ordering to fix rack motion.

The rack is linear: stow is 0.00 m, safe is 0.13 m, deploy is 0.30 m, and the configured upper limit is 0.35 m.
The conversion currently represents 9 motor rotations per pi*0.0254 metres of travel (about 112.79 motor rotations
per metre). Its position conversion factor is 1/ratio and velocity factor is 1/(60*ratio), giving metres and metres/sec.
REVLib 2026 MAXMotion velocity/acceleration honor the velocity conversion factor; do not multiply these settings by
60 again. The configured 0.5 m/s and 100 m/s^2 remain existing settings, not newly verified hardware measurements.
See `REV closed-loop units <https://docs.revrobotics.com/revlib/spark/closed-loop/units>`_.

In AdvantageScope, inspect ``AdvantageKit/IntakeRack`` inputs and ``AdvantageKit/RealOutputs/IntakeRack`` outputs:

* ``RequestedPosition``, ``GoalPosition``, ``CommandOwner``, and ``ControlMode`` distinguish a missing request from
  a clamped target, compliance hold, or another command owning the mechanism.
* ``ConfigurationStatus``, ``ConfigurationHealthy``, ``ControlStatus``, and ``ControlRequestHealthy`` contain vendor
  results. Nonzero output is blocked after a known configuration failure; neutral output remains available.
* ``EncoderResetStatus`` records explicit reset calls. An internal encoder is not an external-encoder fault, and
  the code does not automatically zero the rack just because a check starts.
* ``ConfiguredMinPosition``, ``ConfiguredMaxPosition``, profile constraints, conversion factors, and the
  ``AtReverseLimit``/``AtForwardLimit`` position comparisons help diagnose conversion or zero errors. These comparisons
  describe configured bounds; they are not a claim that a physical limit switch is installed or active.
* ``MotorVoltages`` uses the controller's applied duty and bus voltage; compare it with ``MotorCurrents`` and encoder
  motion. ``CommandedVoltage`` is only the open-loop request and normally reads zero during closed-loop position control.
* ``ProfileStatus`` and ``BrakeStatus`` explicitly distinguish synchronous results from asynchronous enqueue results.
  A queued request or cached successful read does not prove a fresh CAN frame or acknowledged configuration.

The report includes requested/clamped targets, stage start position/travel, actual voltage/current, and controller
acceptance/limit data. Safe and deploy stages must demonstrate encoder motion as well as reach the target. A failed
stow blocks the safe stage; failed safe blocks deploy. The return stage remains available under the original global
interlocks even if deployment fails. Starting at stow is allowed to pass without moving.

Use this order when reviewing a failed check:

#. Confirm the report's run/stage state and that Test mode, setup confirmation, arm, and Start were accepted.
#. Confirm CAN 21 is the expected controller and inspect configuration/request error strings.
#. Compare requested target, current position, encoder zero, soft limits, and physical travel. Never zero at an
   unknown position or bypass limits to force movement.
#. If there is a valid position request but no reported voltage, inspect controller faults/configuration/limits.
#. If voltage is reported but the encoder does not move, inspect wiring, power and the mechanical drive. High current
   is different evidence from zero current; neither proves the cause by itself.
#. If the encoder moves but the mechanism does not, inspect coupling/gearing and conversion before tuning gains.

Save the HTML/JSON report and robot log for the failed rack stages. Real deployment is accepted only after observing
safe/deploy/retract motion and stopping on Disable/cancellation. Passing desktop simulation cannot certify this.
