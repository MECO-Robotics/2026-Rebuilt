PID and FF Tuning with ReCalc
=============================

Use this page when you want a more structured way to choose initial PID and feedforward values.

These are still starting values, not final tuned values. The goal is to get close enough for safe first motion and faster iteration on the real robot.


Why use ReCalc
--------------

`ReCalc <https://www.reca.lc/>`_ can generate first-pass estimates from real mechanism information such as:

- motor type
- number of motors
- gear ratio
- mechanism mass or load
- wheel, drum, arm, or radius dimensions

This is usually better than picking numbers at random, especially for teams that want a more defensible starting point.


When to use this page
---------------------

Use ReCalc after:

- hardware config is correct
- basic mechanism gains and limits are in place
- units are correct
- motion limits are set

If the mechanism is still wired or configured incorrectly, fix that first. ReCalc will not compensate for bad setup.


What ReCalc can help estimate
-----------------------------

Depending on the mechanism, ReCalc can help you choose starting values for:

- ``kP``
- ``kS``
- ``kG``
- ``kV``
- ``kA``

For this codebase:

- flywheel-style mechanisms usually care most about ``kP``, ``kS``, ``kV``, and ``kA``
- position-joint mechanisms may also need ``kG`` plus motion limits such as max velocity and max acceleration


Where the values go
-------------------

Put the values into the gains objects in your subsystem constants file.

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


How to use ReCalc well
----------------------

1. Identify the mechanism type.
2. Enter real mechanism data into ReCalc.
3. Copy the estimated values into the gains object.
4. Build and deploy.
5. Run a small, controlled first test.
6. Refine the values on the real robot.


How to think about the output
-----------------------------

Treat ReCalc values as:

- a strong first guess
- better than random guessing
- an input to real tuning

Do not treat ReCalc values as:

- final competition tuning
- proof that the mechanism is configured correctly
- a replacement for logging and iteration


Common mistakes
---------------

- **Using guessed mechanism data**
  Bad inputs to ReCalc produce bad outputs.
- **Expecting calculator values to be final**
  The real robot still needs validation and tuning.
- **Skipping slow first tests**
  Even good-looking values can still be unsafe if the mechanism is built differently than expected.
