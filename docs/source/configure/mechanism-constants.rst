Mechanism Gains & Limits
========================

Adjust parameters for your specific mechanisms.

.. toctree::
   :hidden:

   pid-ff-tuning-recalc

Example templates live in ``src/main/java/frc/robot/constants/subsystems/example/``.
Copy those files when creating a new flywheel or position-joint mechanism, then rename them
for your subsystem.

This page is focused on gains objects and the robot-specific values that shape mechanism behavior. Once the hardware config is correct, these constants define how the mechanism should respond and be controlled.


Where these constants live
--------------------------

Most mechanism configuration lives in:

- ``src/main/java/frc/robot/constants/subsystems/``
  Subsystem-specific constants files.
- ``src/main/java/frc/robot/constants/types/PositionJointConstants.java``
  Shared position-joint gain records.
- ``src/main/java/frc/robot/constants/types/FlywheelConstants.java``
  Shared flywheel gain records.

In this project, the usual pattern is:

1. Copy one of the example constants files.
2. Rename it for your subsystem.
3. Fill in the gains and motion limits.

Example flywheel gains:

.. code-block:: java

   public static final FlywheelGains GAINS =
       new FlywheelGains(
           0.2,   // kP
           0.0,   // kI
           0.0,   // kD
           0.0,   // kS
           0.065, // kV
           0.0,   // kA
           1.0,   // kMaxAccel
           1.0);  // kTolerance

Example position-joint gains:

.. code-block:: java

   public static final PositionJointGains GAINS =
       new PositionJointGains(
           1.5,     // kP
           0.0,     // kI
           0.0,     // kD
           0.5,     // kS
           1.0,     // kG
           2.0,     // kV
           0.0,     // kA
           10.0,    // kMaxVelo
           20.0,    // kMaxAccel
           0.0,     // kMinPosition
           Math.PI, // kMaxPosition
           0.2,     // kTolerance
           0.0);    // kDefaultSetpoint


What belongs here
-----------------

Gains objects usually include:

- motion limits
- control gains
- default setpoints and tolerances

These values are specific to how your robot is assembled and how you want it to behave. Even if two robots use similar motors, their gains often differ because mechanism mass, gearing, friction, and travel range differ.


Do this
-------

- Copy the example constants file that matches your mechanism
- Choose initial gains using either rough values or ReCalc
- Set limits
- Tune gains for the real mechanism


Choosing initial gains
----------------------

For a first pass, you have two reasonable options:

- **Option A: Use rough "vibe values"**
  Pick conservative values that are close enough to get safe first motion, then iterate on the real robot.
- **Option B: Use ReCalc**
  Use :doc:`pid-ff-tuning-recalc` to generate more informed starting PID and feedforward values from mechanism data.

Option A is faster if you already have a feel for similar mechanisms.

Option B is better if you want a more structured starting point or you are tuning a mechanism type you have less intuition for.

Limits and safety
-----------------

Set safe motion limits before doing serious tuning on real hardware. Most of these constants can be determined using CAD.

Mechanism gains and limits should define:

- **minimum position**: the lowest position the controller will allow, which can prevent crashing into the floor or other hard stops
- **maximum position**: the highest position the controller will allow, which can prevent damage to the mechanism or surrounding components
- **max velocity**: the maximum velocity the controller will command, which can reduce kinetic energy
- **max acceleration**: the maximum acceleration the controller will apply to the mechanism, which can prevent skipping and reduce stress on the mechanism
- **current limits**: the maximum current the controller will apply to the motor, which protects hardware and prevents runaway behavior
- **tolerance**: how close to the target is close enough for the controller to consider it "at the setpoint"

These values are part of the robot definition. They protect hardware and make command behavior more predictable.


How to verify
-------------

After updating mechanism gains and limits:

- the mechanism moves in the correct direction
- sensor readings change in the correct direction
- position and velocity values look reasonable
- soft limits match the real safe range
- closed-loop control behaves consistently
- the mechanism reaches targets without obvious runaway or instability


Expected result
---------------

- Mechanisms move in the correct direction
- Sensor feedback uses the correct units and sign
- Safe limits match the real mechanism travel
- Closed-loop control starts from reasonable physical assumptions
- Mechanisms move correctly and safely


Common mistakes
---------------

- Wrong units
- Limits that do not match the real mechanism
- Using random starting gains without checking behavior carefully
- Expecting first-pass values to be final
- Copying values without verification
