Shooter and Intake Verification
===============================

Use this checklist after deploying changes to the coordinated shooter or intake. The software prevents known command
conflicts, but motor direction, roller wrap, mechanism compression, and the shot table must be verified on the real
robot.


Control-change contract
-----------------------

Scope: the approved driver mapping changes command ownership and lifecycle only. Existing rack limits, saved
retraction settings, roller voltages, current limits, and shooter presets remain owned by their existing constants
and subsystem code. No new physical ratings or hardware performance are inferred from desktop tests.

Acceptance requirements (owner: ``RobotContainer`` bindings and ``ShooterCommands``):

- Start retains heading reset and D-pad behavior is preserved by binding review.
- Acquisition runs intake/conveyor inward and both indexers opposite their feed direction; release stops rollers.
- Ejection uses the intake eject direction and independently corrected conveyor/indexer directions; release stops all rollers.
- Agitation runs the intake immediately and retracts the rack with continuous feed after initial spin-up, independently of aiming.
- Feed waits for an active positive RPM target and initial speed; release or shooter stop neutralizes feed rollers.
- Preset selection persists after button release; replacing/stopping/disabling interrupts it.
- Shooter off uses open-loop zero voltage, not a zero-RPM velocity PID request.

``RobotContainerSimulationTest`` checks bindings, targets, release, preset replacement, feed gating, and stop while
agitation remains held. ``CommandLifetimeTest`` checks interruption/zero-voltage behavior. Physical direction and
ball movement require observation on the robot; no automated test commands real hardware.


Driver controls
---------------

- Hold driver ``A`` to aim at the alliance hub while retaining left-stick translation and continuously update hood angle and flywheel speed from the interpolation tables. A alone does not feed. Release A to stop automatic spin-up; RB remains the feeding control.
- Press ``X`` once to latch the hub hood/RPM preset, or ``Y`` once for the ferry preset. Releasing the button keeps
  the preset active. Copilot ``X``/``Y`` select the same presets; copilot ``A`` latches the trench preset.
- Press ``B`` on either controller to interrupt the preset, command shooter voltage to zero (leave velocity PID),
  and stow the hood. Disable/interruption also stops the preset; re-enabling does not restart it.
- Hold driver right bumper to agitate and shoot: run the intake roller immediately; once feeding starts, retract
  the intake rack to the saved feeding position (default 0.13 m) at the saved maximum speed (default 0.10 m/s),
  while the intake roller continues inward.
  The conveyor and indexers wait until a positive active flywheel RPM target is reached within 3 RPS (180 RPM) for 0.15 s.
  All rollers stay running during retraction. Once feeding begins, speed dips do not interrupt the volley.
  Pressing ``B`` stops shooter and feed even if the bumper remains held; intake agitation continues until release.
- Release right bumper to stop the intake, conveyor, and indexers and return the rack to deployed (0.30 m).
  The selected flywheel preset keeps running. Aim and feed are independent: ``A`` is optional while feeding.
- Hold driver right trigger to deploy the rack and acquire balls using intake + conveyor with both indexers running
  backwards relative to shooting. Release to stop the rollers and leave the rack deployed.
- Hold driver left trigger for full fuel-path ejection: rack deployed, intake at its eject voltage, conveyor at
  half the intake voltage, and both indexers at their negated feed voltages. Release stops all four rollers.
- While right bumper is held, two strong rumble pulses each second mean feeding is blocked (no shooter preset or
  waiting for initial shooter speed). Steady lighter rumble means feeding is active. Release, Disable, or command
  interruption clears rumble. ``Shooter/FeedBlockedReason`` shows ``SELECT_SHOOTER_PRESET`` or
  ``WAITING_FOR_SHOOTER_SPEED`` and clears when feeding starts or the command ends.
- Driver left bumper is unassigned. Driver Start resets heading.
- Driver and copilot D-pad up/down retain deploy/safe-retract behavior; down runs intake + conveyor inward and
  stops them on release. Copilot D-pad left/right retain rack deploy/full-stow setpoints.
- When ``/TunableNumbers/ShooterCalibration/Enabled`` is true, pressing ``X`` selects bounded live calibration
  hood/RPM values instead of the hub preset. Right bumper still handles agitation/feed; ``B`` stops the preset.

For coordinated autonomous shots, ``Shooter/Readiness`` identifies the current blocker and
``Shooter/TargetMode`` reports ``HUB`` or ``FERRY``. Autonomous automatic feeding requires a field-aligned pose, translation at or below 0.25 m/s, rotation at or below 10
degrees/s, heading error at or below 2 degrees, hood error at or below 0.002 rotations, and flywheel error at or below
0.75 RPS. Hub shots additionally require a distance from 46.003 through 236 inches. Ferry shots use the ferry hood
position and calculate enough flywheel speed to land at the safe field point. If that would exceed the configured
ferry maximum, pass through the hub structure, or fail to clear the complete trench depth with the configured safety
margin, readiness reports ``UNSAFE_FERRY_PATH`` and automatic feeding remains blocked. All conditions must remain
valid for 0.15 seconds. Without a valid pose, the command holds the current heading, stops the flywheel, and stows the
hood instead of preparing from an uninitialized field position. This autonomous gate does not apply to the independent
manual right-bumper feed action.


Deployment identity and neutral-output check
--------------------------------------------

Before deploying, run ``git branch --show-current`` and ``git log -1 --oneline``. Confirm the branch and commit contain
the intended mechanism changes; do not assume code tested on another branch is present. After deployment, confirm the
same identity in the AdvantageKit ``RealMetadata/GitBranch`` and ``RealMetadata/GitSHA`` fields before enabling.

Raise the robot securely and remove all fuel for the first enabled test. Hold driver ``X`` and confirm the flywheel
receives its velocity request. Release ``X`` and confirm ``ShooterFlywheel/DesiredVelocity`` and
``ShooterFlywheel/CommandedVoltage`` are zero and ``ShooterFlywheel/VelocityControlActive`` is false. On real hardware,
also confirm every value in ``ShooterFlywheel/MotorVoltages`` settles to zero. The wheel may coast mechanically, but
the controller must enter open-loop 0 V mode immediately. Repeat the release check for the intake roller, conveyor,
and both indexers. Disable the robot immediately if a released mechanism reports a nonzero commanded voltage, remains
in velocity control, or makes an unexpected powered sound.


Desktop simulation and AdvantageScope
-------------------------------------

Run ``WPILib: Simulate Robot Code`` with the Sim GUI enabled, map the driver controller to joystick port 0, and
connect AdvantageScope to the simulator using NetworkTables 4 (AdvantageKit). Import
``AdvantageScope Shooting and Intake.json`` for the prepared ``Remy Simulation`` 3D field and
``Shot & Intake Status`` table. The layout shows the articulated Remy model, field and launched fuel, readiness,
feeding state, pose validity, hopper count, and successful simulated scores.

Starting fuel is populated as soon as the physics simulation initializes, including when entering teleop directly;
starting autonomous first is not required. Restart the robot simulation after rebuilding so this field state is reset.

AdvantageScope's custom-assets folder must be the repository's ``sim`` directory, which contains
``Robot_Remy/config.json`` and the base/component ``.glb`` files. The component-pose source must be configured as a
child ``Component`` of the Remy robot, not as a separate ``Ghost`` robot.

Use desktop simulation to validate command scheduling, readiness transitions, intake collection, aiming direction,
and autonomous timing. Use each flywheel subsystem's ``CommandedVoltage`` and ``VelocityControlActive`` fields to
verify command output in simulation; vendor-reported ``MotorVoltages`` can be stale on simulated custom CAN buses. It
does not replace the real-robot motor-direction, roller-contact, current-limit, or shot-table checks below.

Run the complete Java 17 simulation test suite once for each alliance before deploying direction or aiming changes::

  FRC_TEST_ALLIANCE=blue ./gradlew clean test
  FRC_TEST_ALLIANCE=red ./gradlew clean test

The full robot integration test rejects an unknown value and confirms that Driver Station simulation actually applied
the requested alliance before exercising the driver bindings, coordinated shooting, autonomous shooting, and system
check flow. In the same robot process it also disables, switches to the opposite alliance, re-enables, and returns to
the starting alliance. Each transition must update path mirroring, driver-forward field direction, the selected hub,
and the physical intake/shooter-side aim without rebuilding or restarting robot code.


Low-voltage mechanism verification
----------------------------------

Raise the robot securely, remove fuel, keep one person at the emergency stop, and test one mechanism at a time at low
voltage. Do not change inversion and roller wrap during the same test; that makes the actual cause impossible to
identify.

Record the observed surface direction for each mechanism:

.. list-table:: Mechanism map
   :header-rows: 1

   * - Mechanism
     - CAN ID(s)
     - Required fuel motion
     - Verified
   * - Intake rack
     - 21
     - Positive position deploys
     - |square|
   * - Intake rollers
     - 22, 53
     - Pulls contacted fuel under the frame
     - |square|
   * - Conveyor
     - 23
     - Intake command moves fuel toward storage; feed moves fuel toward shooter
     - |square|
   * - Bottom storage indexer
     - 31
     - Acquisition moves away from shooter; feed moves toward shooter
     - |square|
   * - Top storage indexer
     - 32
     - Acquisition moves away from shooter; feed moves toward shooter
     - |square|
   * - Shooter flywheel
     - 34, 35
     - Accelerates fuel through the shooter exit
     - |square|

The current hardware configuration contains no separate agitator motor. The top and bottom indexers are therefore the
agitation/feed assembly. If a separate motor exists on the robot, stop here and add its CAN configuration and command
requirement before operating the coordinated shot.

The current **real-robot** shooter/camera configuration retains a 180-degree yaw offset and a shooter throat
at ``-0.19 m`` X. Its physical relationship to robot-relative forward still needs verification. Neither direction
should depend on alliance. Do not change this hardware yaw merely to repair the simulation display.

The Remy CAD asset places the intake roller on **+X** after its existing import rotations. Desktop simulation now
collects, extends the rack/hopper, and launches toward +X. Simulated automatic aiming uses that same yaw; real/replay
aiming retains the existing hardware yaw. The throat location remains the existing estimated -0.19 m offset;
being behind robot center does not imply shooting backward. No drivetrain or whole-asset rotation was changed.
Intake rack extension has its own direction constant and no longer borrows shooter yaw.

Before unifying the physical and CAD frames, identify which end leads during a robot-relative forward command and
verify the shooter throat/camera mounting. This is a required hardware measurement, not an inferred motor inversion.


Touch-and-own intake calibration
--------------------------------

1. Inspect roller wrap, belt/chain tension, contact height, and whether the two intake motors fight each other.
2. Start with the dashboard deploy setpoint at 0.30 m. Approach loose fuel on carpet without using a wall.
3. Run 20 contact attempts, including center and both outer portions of the intake. Record captures, stalls, current
   limits, carpet contact, and frame contact.
4. If fewer than 18 attempts are captured, increase the deploy setpoint by 0.01 m and repeat, never exceeding the
   configured 0.35 m soft limit.
5. Select the smallest extension that captures at least 18/20 without unsafe contact or current limiting.
6. Perform a deliberate low-speed wall contact and verify that the rack becomes compliant near target, moves safely,
   and resumes position control after being displaced by more than 0.02 m.

Rewrap a roller only when the motor shafts rotate as intended but the roller surfaces still push fuel away from the
robot. Save the selected deploy value back into ``IntakeConstants.RACK_PRESETS.DEPLOY`` after testing.


Stationary shot calibration
---------------------------

Verify stationary shots before attempting motion compensation. Test the 46.003-inch close point, several intermediate
distances, and the 236-inch far point from several field angles on both alliances.

The prepared AdvantageScope layout includes these editable NetworkTables values:

- ``/TunableNumbers/ShooterCalibration/Enabled``
- ``/TunableNumbers/ShooterCalibration/HoodRotations``
- ``/TunableNumbers/ShooterCalibration/FlywheelRPS``

Set ``Enabled`` true only during a controlled calibration session, then press ``X`` to latch the live hood/RPM preset.
Aim manually and hold right bumper for each firing trial. Do not hold A during live calibration: A replaces the calibration preset with the interpolation-table solution.
Release right bumper to stop feed and press ``B`` to stop the shooter. Hood input is clamped to 0--0.049 rotations and
flywheel input is clamped to 0--60 RPS. Autonomous shooting always ignores calibration mode.

Driver ``A`` always targets the alliance hub using the interpolation tables. X/Y/copilot presets replace the A command if pressed while A is held; release and press A again to resume automatic aiming and spin-up.

Watch ``Shooter/CalibrationDistanceInches`` and the applied/measured hood and flywheel rows in the
``Shot & Intake Status`` tab. Dashboard values are temporary calibration inputs, not permanent shot-table changes.
Copy every accepted distance, hood, and flywheel value into ``ShooterConstants.hoodMap`` and
``ShooterConstants.shooterVelocityMap``, deploy the saved values, and turn ``Enabled`` false when calibration is
complete.

The current measured real-robot references with the worn shooter tape are 46.003 inches at 0.000 hood rotations and
30 RPS, 144.60 inches at 0.020 rotations and 40 RPS, 153.64 inches at 0.020 rotations and 41 RPS, 191.86 inches at
0.020 rotations and 46 RPS, 202.78 inches at 0.020 rotations and 47 RPS, and 210.47 inches at 0.020 rotations and 47
RPS. Other RPS entries are provisional values scaled by ``30 / 28.5`` and must be validated on the real robot; their
hood entries were not shifted because the real and simulated close-shot hood positions are both zero. The projectile
simulation applies the matching ``28.5 / 30`` surface-efficiency scale so those adjusted RPS values retain the
previously calibrated simulated launch speed instead of overshooting the hub.

The hood encoder measures increasing deflection down from vertical, while MapleSim measures projectile pitch up from
horizontal. The simulation converts between these references: ``0.000`` hood rotations produces a nominal 74-degree
launch pitch and ``0.049`` rotations produces 56.36 degrees. Confirm the exact pitch used for a trial with
``FieldSimulation/LaunchDebug/LastLaunchPitchDegrees`` in the prepared status tab.

At each location:

1. Confirm the displayed distance agrees with a physical measurement from the shooter exit.
2. Adjust hood position to center the trajectory vertically before changing flywheel speed.
3. Adjust flywheel RPS to center the landing point in the hub.
4. Shoot ten trials and require at least eight successful shots before accepting the point.
5. Store accepted values in the hood and flywheel interpolation maps, then repeat adjacent points to check smooth
   interpolation.

Automatic feed must remain blocked outside 46.003--236 inches. Validate every PathPlanner shooting auto three consecutive
times and confirm it either completes one second of ready feed or logs the readiness reason and exits within three
seconds. PathPlanner and Choreo routines seed their field pose from the selected trajectory. The chooser's standalone
fender shot does not reset odometry, so it requires an accepted absolute vision alignment and exits without aiming or
shooting if that pose never becomes ready.

.. |square| unicode:: U+2610

Saved feeding-rack tuning
------------------------

With NetworkTables connected, edit ``/Preferences/Intake/ShotRetractSpeedMps``
(default 0.10, allowed 0.01–0.50 m/s) and
``/Preferences/Intake/ShotHoldPositionMeters`` (default 0.13, allowed 0.13–0.30 m).
These are WPILib Preferences, saved on the roboRIO across program restarts;
simulation uses its own NetworkTables persistence file and does not transfer values to the robot.
Use a NetworkTables editor or Shuffleboard Preferences widget to edit these entries.

Lower speed means slower inward motion. Larger position means less inward travel:
0.30 m is deployed and 0.13 m is the existing safe retracted position. Values outside
these ranges are clamped; nonfinite values use the defaults. The applied values are
logged as ``Intake/Shot/AppliedRetractSpeedMps`` and ``AppliedHoldPositionMeters``.
The target is a commanded position, not a physical hard stop; verify actual stopping
and clearance on the robot at low speed before increasing travel or speed.

Settings are sampled when feeding begins and stay fixed for that volley. Release RB
and start another volley to apply edits. RB release restores the usual deployment
speed and sends the rack to the deployed preset. Other rack commands retain their
normal motion settings. Initial flywheel readiness and continuous feeding behavior
are unchanged.
