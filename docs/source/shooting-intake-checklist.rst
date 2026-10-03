Shooter and Intake Verification
===============================

Use this checklist after deploying changes to the coordinated shooter or intake. The software prevents known command
conflicts, but motor direction, roller wrap, mechanism compression, and the shot table must be verified on the real
robot.


Driver controls
---------------

- Hold driver ``A`` to run the coordinated hub-or-ferry shot. Inside the robot's own alliance zone, ``A`` always
  selects the alliance hub even if no tag is visible. Outside that zone, any visible red-side tag (1--16) selects the
  red hub for red, and any visible blue-side tag (17--32) selects the blue hub for blue. If no tag from the robot's
  alliance side is visible, ferry mode uses one of two safe landing points inside the robot's alliance zone, one on
  each side of the field. The robot's current field half selects the matching landing point, and crossing the field
  centerline while holding ``A`` switches the point and resets shot readiness. Tags 6/7 and 22/23 are lane references
  only; neither pair is a landing target. The hub-versus-ferry mode remains fixed until ``A`` is released. A tag remains
  recently visible for 0.25 seconds so a normal gap between camera frames cannot select ferry mode accidentally.
- Hold driver ``A`` and right bumper to force the feed path for recovery only. This bypasses range, pose, motion, and
  mechanism-readiness protection.
- When ``/TunableNumbers/ShooterCalibration/Enabled`` is true, holding ``A`` uses the live calibration hood and
  flywheel values and automatic feeding is disabled. Hold ``A`` and right bumper to fire a calibration trial.
- Hold driver left bumper to deploy the rack and run the intake roller, conveyor, and both storage indexers. Release it
  to stop the path while leaving the rack deployed.
- Hold driver left trigger to reverse the complete acquisition path.
- Copilot POV left/right commands rack deploy/stow setpoints. POV up/down remain open-loop velocity backups.

The dashboard key ``Shooter/Readiness`` identifies the current blocker, and ``Shooter/TargetMode`` reports ``HUB`` or
``FERRY``. Automatic feeding requires a field-aligned pose, translation at or below 0.25 m/s, rotation at or below 10
degrees/s, heading error at or below 2 degrees, hood error at or below 0.002 rotations, and flywheel error at or below
0.75 RPS. Hub shots additionally require a distance from 46.003 through 236 inches. Ferry shots use the ferry hood
position and calculate enough flywheel speed to land at the safe field point. If that would exceed the configured
ferry maximum, pass through the hub structure, or fail to clear the complete trench depth with the configured safety
margin, readiness reports ``UNSAFE_FERRY_PATH`` and automatic feeding remains blocked. All conditions must remain
valid for 0.15 seconds. Without a valid pose, the command holds the current heading, stops the flywheel, and stows the
hood instead of preparing from an uninitialized field position; ``A`` plus right bumper remains the explicit override.


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

The intake, shooter, and Limelight face the Remy model's ``+X`` direction. The shooter throat is behind robot center at
``-0.19 m`` X, but its projectile travels toward model-forward. The drivetrain heading convention is 180 degrees
opposite that model-forward direction, so ``AUTO_AIM_HEADING_OFFSET`` is fixed at 180 degrees while
``SHOOTER_YAW_OFFSET`` remains zero. Neither offset changes with alliance; only the hub coordinates do.


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

Set ``Enabled`` true only during a controlled calibration session. Hold driver ``A`` to keep AprilTag/QuestNav-based
aiming active while the live hood and flywheel values are applied. Calibration mode never feeds automatically; hold
driver ``A`` and right bumper together for each firing trial. Hood input is clamped to 0--0.049 rotations and
flywheel input is clamped to 0--60 RPS. Autonomous shooting always ignores calibration mode.

Calibration mode and autonomous shooting always select the alliance hub; the no-visible-tag ferry fallback applies
only to the normal driver-held ``A`` command.

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
