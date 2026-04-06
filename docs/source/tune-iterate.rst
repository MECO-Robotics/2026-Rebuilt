Tune & Iterate
==============

Only begin tuning after the robot is already functioning correctly.

Tuning is not the step where you fix wiring, wrong IDs, reversed motors, bad unit conversions, or broken command logic. Those problems must be resolved first.


Before you tune
---------------

Confirm these first:

- Hardware config matches the robot. See :doc:`configure/hardware-ids`.
- Mechanism gains and limits match the real robot. See :doc:`configure/mechanism-constants`.
- Motors move in the correct direction.
- Encoders report the correct direction and reasonable values.
- Commands schedule correctly and target the intended subsystem.
- The robot can build, deploy, and run without startup errors.

If any of those are still wrong, stop and fix them before adjusting gains.


Recommended order
-----------------

Tune in this order:

1. Fix direction, units, and control mapping.
2. Tune drivetrain feedforward and basic drive behavior.
3. Tune steering and module response if using swerve.
4. Tune simple mechanisms such as flywheels.
5. Tune position-controlled mechanisms such as arms, wrists, and elevators.
6. Re-test full robot behavior after each tuning pass.

This order matters because later tuning depends on earlier layers being correct.


Tools
-----

- **SysId**
  Use for system identification and feedforward measurement when appropriate.
- **AdvantageScope**
  Use to inspect logs, sensor traces, error, velocity, and setpoint tracking over time.
- **SmartDashboard / NetworkTables**
  Useful for observing live values and interacting with tunable parameters.
- **Vendor tools**
  Use REV Hardware Client or Phoenix Tuner when you need to verify hardware-side behavior during tuning.


What can be tuned in this project
---------------------------------

This codebase supports tuning for several subsystem types.

- **Drive motors**
  Use drive motor gains such as ``kP``, ``kI``, ``kD``, ``kS``, ``kV``, and ``kA``.
- **Azimuth / steering motors**
  Tune steering response and steering feedforward if applicable.
- **Flywheels**
  Tune velocity-loop gains and acceleration constraints.
- **Position joints**
  Tune PID, feedforward, motion constraints, safe range limits, and tolerance.

Some mechanisms expose tunable values through logged tunable numbers, which allows iteration without rebuilding every small change.


Drivetrain tuning
-----------------

Start by making sure the drivetrain is mechanically and electrically correct.

Check:

- The robot drives in the direction commanded.
- Wheel directions are correct.
- Steering modules point where the software expects.
- Odometry is not obviously unstable.

After that, tune drivetrain behavior in this order:

1. Measure or estimate feedforward.
2. Confirm velocity response is smooth and predictable.
3. Adjust closed-loop gains only after feedforward is reasonable.
4. Test teleop drive feel and autonomous path tracking separately.


Drive characterization
----------------------

This project includes drivetrain characterization support in ``frc.robot.commands.drive.DriveCharacterization``.

That characterization code can:

- measure drive feedforward values
- estimate wheel radius through a dedicated characterization routine

Use the resulting values to improve the drivetrain constants before trying to make PID compensate for a poor model.

When reviewing characterization results:

- ``kS`` helps overcome static friction
- ``kV`` helps match steady-state velocity demand
- wheel radius affects odometry and kinematic accuracy

If feedforward is badly wrong, autonomous tracking and velocity control usually suffer no matter how much PID you add.


Mechanism tuning
----------------

Mechanisms should be tuned only after they move safely and consistently.

In general:

1. Verify the mechanism does not run away when enabled.
2. Set safe position limits before aggressive testing.
3. Start with conservative gains.
4. Tune feedforward before pushing PID too high.
5. Increase constraints only after tracking is stable.


How to tune common values
-------------------------

- **kS**
  Static gain. Increase until the mechanism can overcome friction reliably without needing excessive PID.
- **kG**
  Gravity gain. Important for arms and elevators. If this is wrong, the mechanism may sag, drift, or require too much feedback correction.
- **kV**
  Velocity gain. Helps the mechanism match moving setpoints more naturally.
- **kA**
  Acceleration gain. Helps during starts, stops, and rapid profile changes.
- **kP**
  Corrects position or velocity error. Raise it gradually until the system responds well without obvious oscillation.
- **kI**
  Use sparingly. It can help with persistent steady-state error but often causes trouble if used too early.
- **kD**
  Can reduce overshoot and ringing, but too much makes the system noisy and sluggish.
- **kMaxVelo / kMaxAccel**
  Motion constraints. These shape how aggressively a position-controlled mechanism moves.
- **kTolerance**
  Determines when a mechanism is considered close enough to its target to finish a command.


Position-controlled mechanisms
------------------------------

Position joints in this project expose tunable values such as:

- PID gains
- feedforward gains
- max velocity
- max acceleration
- minimum position
- maximum position
- tolerance
- default setpoint

That means tuning is not only about response quality. It is also about defining safe and realistic motion behavior.

For arms, elevators, and similar mechanisms:

- verify zeroing first
- confirm unit conversions are correct
- clamp motion to a safe range
- tune gravity compensation before expecting accurate tracking
- test small moves before long or fast moves


Flywheel-style mechanisms
-------------------------

For flywheels and other velocity-controlled mechanisms:

- tune feedforward so commanded speed and measured speed are close
- add proportional gain to tighten response
- use integral only if necessary
- check spin-up time and recovery after load changes
- keep acceleration limits realistic for the motor and mechanism


How to iterate well
-------------------

Use a consistent loop:

1. Change one thing.
2. Run the same test again.
3. Observe the response in logs.
4. Decide whether the change improved tracking, stability, or repeatability.
5. Keep or revert the change based on evidence.

Good iteration means using repeatable tests, not just driving around until the robot feels different.


What to watch in logs
---------------------

When using AdvantageScope or other logging tools, compare:

- commanded setpoint
- measured position or velocity
- tracking error
- overshoot
- settling time
- oscillation
- current draw or voltage demand if available

A clean tuning change should improve one of those signals without causing a worse problem somewhere else.


Common tuning mistakes
----------------------

- **Tuning before configuration is correct**
  Bad IDs, inversion, or units will make tuning results meaningless.
- **Changing multiple gains at once**
  This makes it hard to know what actually caused the behavior change.
- **Using PID to compensate for bad feedforward**
  This often creates unstable or inconsistent behavior.
- **Ignoring mechanism limits**
  A mechanism can be well tuned and still unsafe if limits are wrong.
- **Testing only one case**
  A subsystem that works for one short move may still fail at larger moves, higher speeds, or under load.
- **Overusing integral gain**
  Integral is often the first way teams make a stable system unstable.


Expected result
---------------

After tuning:

- the robot responds predictably
- mechanisms track setpoints with reasonable error
- motion is stable without excessive oscillation
- commands finish consistently
- autonomous and teleop behavior are easier to trust


Warning
-------

Do not tune a broken system. Fix functionality first, then tune performance.
