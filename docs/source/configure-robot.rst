Configure Robot
===============

The base project is intended to be reused across different robots, so it cannot know your exact wiring, motors, gear ratios, or mechanism limits ahead of time.

Before you test on real hardware, update the robot configuration so the code matches the physical machine. If these values are wrong, the robot may fail to move, move in the wrong direction, or behave unsafely.

If you have not decided whether your mechanism should be modeled as ``Flywheel`` or ``PositionJoint``, read :doc:`mechanism-types` first.


.. toctree::
   :hidden:

   configure/hardware-ids
   configure/mechanism-constants
   configure/pid-ff-tuning-recalc

In this step you will
---------------------

- Set hardware config: :doc:`configure/hardware-ids`
- Adjust mechanism gains and limits: :doc:`configure/mechanism-constants`

Recommended order
-----------------

Configure the robot in this order:

1. Set hardware config so the code talks to the correct devices.
2. Update mechanism gains and limits so position, velocity, inversion, limits, and starting gains match the real robot.
3. Build and deploy again after configuration changes.
4. Verify basic motion before attempting deeper controller tuning.


What to configure
-----------------

- **Hardware config**
  Assign CAN IDs, ports, and other device identifiers so each subsystem can find the correct hardware. Start here, because no other configuration matters if the code is talking to the wrong device.
- **Mechanism gains and limits**
  Update gear ratios, inversion, unit conversions, control gains, and safe motion limits for each mechanism. These values determine how the robot interprets sensor feedback and commands motor output.

Expected result
---------------

After configuration:

- The robot builds and deploys successfully.
- Devices appear on the expected bus with no ID conflicts.
- Mechanisms move in the correct direction.
- Sensor readings and commanded motion use the expected units.
- Basic subsystem behavior works before any serious tuning begins.


Common mistakes
---------------

- **Skipping hardware ID verification**
  A mechanism that does nothing is often mapped to the wrong CAN ID or port.
- **Copying constants from another robot unchanged**
  Even similar mechanisms can require different gear ratios, inversion, limits, and gains.
- **Tuning before configuration is correct**
  If IDs, units, or inversion are wrong, tuning will waste time and may make debugging harder.
- **Expecting calculator values to be final**
  ReCalc is a starting point. Real tuning and verification are still required on the robot.
- **Ignoring safe limits**
  Always set reasonable minimum and maximum positions before testing real motion on a new mechanism.
