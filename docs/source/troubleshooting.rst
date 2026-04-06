Troubleshooting
===============

Common issues and how to fix them.


If a mechanism isn't running
----------------------------

Check Wiring
~~~~~~~~~~~~
The first thing to rule out when implementing new code is wiring. Wiring issues are common and easy to overlook, so confirm the physical setup before changing software. Check:

- **Robot communication**
  If the driver station shows ``No Robot Code`` or ``No Communication``, the RoboRIO is not communicating correctly. This can be caused by a bad Ethernet cable, incorrect network configuration, or code crashing during startup.

  - **RoboRIO has power**
    A powered RoboRIO should show normal status lights and appear in the driver station. If the RoboRIO is not fully booted, no mechanism code will run.
  - **Ethernet cable is good**
    Try a known-good cable between the driver station and the RoboRIO. A bad cable can cause intermittent communication issues that look like software failures.
  - **Code is not crashing on startup**
    If the code exits during robot initialization, the robot will never enable teleop or autonomous correctly. Check the driver station logs for startup exceptions.
- **Motor controllers have power**:
  Verify that each controller is receiving battery power and showing its normal indicator lights. A controller on the CAN bus but without power will usually not respond to commands.
- **CAN wiring is correct**:
  Inspect CAN high and CAN low connections, termination, and connector seating. One loose or reversed connection can prevent the entire bus from behaving correctly. Make sure your loop is terminated and there are no CAN errors reported in the driver station logs.
- **Motors are connected to the correct CAN bus**:
  Make sure the robot code and the physical wiring agree on which bus each controller is on. This is especially important if your robot uses both the RoboRIO CAN bus and a CANivore.

Check Hardware Configuration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Another common issue is incorrect hardware configuration. These mistakes are usually easy to fix, so check them before moving into deeper debugging. Check:

- **Hardware config matches constants**:
  Compare the configured CAN ID of every device against the constants used in code. A single mismatched ID can make a mechanism appear completely dead. See :doc:`configure/hardware-ids`.
- **Motor type is correct in constants**:
  Confirm that each motor controller is configured for the actual motor attached to it. Using the wrong motor type will cause the controller to appear to be dead. For example, a controller configured for a NEO will not respond to commands if it is actually connected to a TalonFX.
- **Mechanism gains and limits are reasonable**:
  Check inversion, gear ratio, unit conversions, and soft limits before tuning. Bad constants will often make a healthy mechanism look unstable or unresponsive. See :doc:`configure/mechanism-constants`.
- **Motor controllers are flashed with correct firmware**:
  Update controllers to the expected firmware version and verify they were configured successfully. Old or mismatched firmware can lead to missing features or unexpected behavior.
- **CAN conflicts**:
  Ensure no two devices share the same CAN ID on the same bus. Duplicate IDs often cause intermittent communication problems that look like random software failures.

If a mechanism is running but behaving incorrectly
--------------------------------------------------

Check Controller Tuning
~~~~~~~~~~~~~~~~~~~~~~~
If a mechanism moves but does not move well, poor controller tuning is the most likely cause. Check:

- **Feedforward values**

  - **Static gain**
    This should be large enough to overcome friction without causing movement when the mechanism should be idle.
  - **Gravity gain**
    For arms and other gravity-loaded mechanisms, make sure this value is tuned with the correct zero reference. The mechanism zero must be perpendicular to the ground for the arm model used in this project.
  - **Velocity gain**
    If this is too low, the mechanism may lag behind targets. If it is too high, motion can become aggressive or unstable.
  - **Acceleration gain**
    Use this to improve response during changes in speed. Incorrect values usually show up during starts and stops rather than steady motion.
- **PID values**

  - **Proportional gain**
    Increase this to reduce steady tracking error, but stop if the mechanism begins oscillating.
  - **Integral gain**
    Use this sparingly to remove persistent error. Too much integral gain can cause slow oscillations or overshoot.
  - **Derivative gain**
    Derivative gain can reduce overshoot and smooth response, but too much will make the mechanism noisy and sluggish.
- **Motion constraints**

  - **Max velocity**
    If set too low, the mechanism will feel slow even when everything else is correct.
  - **Max acceleration**
    If set too low, the mechanism may never reach its expected speed over short moves.
  - **Minimum position**
    Confirm this limit matches the real physical safe range of the mechanism.
  - **Maximum position**
    Incorrect soft limits can make a mechanism stop early or refuse to move in one direction.
- **Encoder and sensor feedback**

  - **Inversions**
    Make sure sensor readings increase in the expected direction when the mechanism moves. If the feedback is inverted, the controller will try to correct in the wrong direction, causing instability or unresponsiveness.
  - **Gear Ratio**
    If the gear ratio is incorrect, the controller will be trying to move the mechanism much faster or slower than expected, which can cause poor performance even when tuning values are reasonable.
  - **Finish Command Tolerance**
    If the command is not exiting when expected, the problem could be a too small value for ``kTolerance``. This value controls when the command deschedules.
Check Command Logic
~~~~~~~~~~~~~~~~~~~
The other common cause is incorrect command logic. If the hardware is responsive and tuning looks reasonable, inspect the command flow. Check:

- **Command is scheduled**:
  Verify that the command actually starts when you expect it to. Logging, dashboard indicators, or temporary print statements can help confirm this.
- **Command is not being interrupted by another command**:
  If another command requires the same subsystem, it may immediately cancel the one you are testing. Look for default commands or button bindings that reschedule frequently.
- **Command is using correct subsystem methods**:
  Make sure the command calls the intended control method, such as closed-loop position control versus open-loop percent output.
- **Command is using correct parameters**:
  Confirm that target positions, velocities, and units are correct. A command can appear broken when it is simply being given an impossible or tiny target.
- **Supplier methods are correct**:
  Check joystick suppliers, lambdas, and dashboard inputs. A supplier returning the wrong sign, wrong units, or a constant zero will make the mechanism behave incorrectly even when the command is otherwise correct.


General Debugging Tips
----------------------

When a problem is not obvious, use a consistent process instead of changing several things at once.

- Test one layer at a time
  First verify physical hardware, then hardware config and constants, then subsystem behavior, and only then command behavior.
- Change one variable at a time
  If you adjust constants, wiring, and command code all at once, it becomes much harder to identify what fixed or caused the issue.
- Use logs and dashboards
  Check whether the robot reports the expected state, setpoint, and sensor values before assuming a motor controller or command is broken.
- Compare expected units to actual units
  Many mechanism issues come from mixing rotations, radians, meters, and degrees across constants, control logic, and dashboard displays.
