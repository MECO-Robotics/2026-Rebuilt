Shooter and Intake Verification
===============================

Use this checklist after deploying changes to the coordinated shooter or intake. The software prevents known command
conflicts, but motor direction, roller wrap, mechanism compression, and the shot table must be verified on the real
robot.


Driver controls
---------------

- Hold driver ``A`` to aim at the alliance hub, update the hood/flywheel solution, and feed automatically when ready.
- Hold driver ``A`` and right bumper to force the feed path for recovery only. This bypasses range, pose, motion, and
  mechanism-readiness protection.
- Hold driver left bumper to deploy the rack and run the intake roller, conveyor, and both storage indexers. Release it
  to stop the path while leaving the rack deployed.
- Hold driver left trigger to reverse the complete acquisition path.
- Copilot POV left/right commands rack deploy/stow setpoints. POV up/down remain open-loop velocity backups.

The dashboard key ``Shooter/Readiness`` identifies the current blocker. Automatic feeding requires a field-aligned
pose, a distance from 58 through 236 inches, translation at or below 0.25 m/s, rotation at or below 10 degrees/s,
heading error at or below 2 degrees, hood error at or below 0.002 rotations, and flywheel error at or below 0.75 RPS.
All conditions must remain valid for 0.15 seconds.


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

The shooter is confirmed to face robot-forward in the same direction as the intake. ``SHOOTER_YAW_OFFSET`` is
therefore zero degrees for both alliances. The simulation imports this same constant so simulated and real aiming use
the same physical direction.


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

Verify stationary shots before attempting motion compensation. Test 58, 80, 100, 120, 140, 160, 180, 200, 220, and
236 inches from several field angles on both alliances.

At each location:

1. Confirm the displayed distance agrees with a physical measurement from the shooter exit.
2. Adjust hood position to center the trajectory vertically before changing flywheel speed.
3. Adjust flywheel RPS to center the landing point in the hub.
4. Shoot ten trials and require at least eight successful shots before accepting the point.
5. Store accepted values in the hood and flywheel interpolation maps, then repeat adjacent points to check smooth
   interpolation.

Automatic feed must remain blocked outside 58--236 inches. Validate every PathPlanner shooting auto three consecutive
times and confirm it either completes one second of ready feed or logs the readiness reason and exits within three
seconds.

.. |square| unicode:: U+2610
