Hardware Config
===============

Set CAN IDs, encoder IDs, bus names, and ports so the code matches your robot exactly.

If these values are wrong, the robot may fail to detect devices, read the wrong sensors, or command the wrong mechanism.

Before testing on real hardware, make sure the identifiers in code match the identifiers flashed into each device.


Where to change IDs in this project
-----------------------------------

Most hardware identifiers live under ``src/main/java/frc/robot/constants/``.

Common places to check:

- ``frc.robot.constants.drive.DriveMotorConstants``
  Drive motor CAN IDs and drive motor CAN bus settings.
- ``frc.robot.constants.drive.AzimuthMotorConstants``
  Steering motor CAN IDs, steering encoder IDs, and steering CAN bus settings.
- ``frc.robot.constants.subsystems``
  Mechanism-specific constants files that contain IDs for subsystems beyond the drivetrain.
- ``frc.robot.constants.types``
  Shared config record shapes that define which hardware identifier fields each subsystem expects.

What each identifier means
~~~~~~~~~~~~~~~~~~~~~~~~

- **CAN ID**
  The address used by a CAN device such as a TalonFX, Spark MAX, Pigeon2, or CANcoder.
- **Encoder ID**
  The identifier for an external encoder. This may be a CAN ID for a CANcoder or a DIO port for a wired absolute encoder, depending on encoder type.
- **CAN bus name**
  The network the device is on. In this project, ``"rio"`` means the RoboRIO CAN bus. If you use a CANivore, the bus name in code must match the name configured in Phoenix Tuner X.
- **Port**
  A RoboRIO port such as DIO, PWM, or USB. These must match the physical wiring exactly.


Recommended process
-------------------

Configure hardware identifiers in this order:

1. Make a complete device list before editing code.
2. Assign unique CAN IDs to every CAN device on the robot.
3. Confirm which devices are on the RoboRIO CAN bus and which are on a CANivore.
4. Update the constants in code to match those IDs and bus names.
5. Verify that external encoders and sensors use the correct ID or port.
6. Deploy and confirm the robot reports all expected devices without disconnect alerts.


Flash device IDs first
~~~~~~~~~~~~~~~

Set the IDs on the motor controllers and sensors using the vendor configuration tools.

- Use REV Hardware Client for REV devices.
- Use Phoenix Tuner or Phoenix Tuner X for CTRE devices.
- Program each device with the intended CAN ID.
- Label devices physically after programming them so future maintenance is easier.
- Optionally clear sticky faults on devices.

This step matters because the ids need to be set on the hardware before the code can find them.

Update constants in code
~~~~~~~~~~~~~~~~

After the hardware config is flashed on the motor controllers and sensors, update the constants so the code points to the correct devices.

When updating constants, make sure:

- motor controller IDs must match the real devices
- follower motor IDs must be in the correct order
- external encoder IDs must match the configured sensor
- bus names must match the actual robot network layout

Example flywheel hardware config:

.. code-block:: java

   public static final FlywheelHardwareConfig CONFIG =
       new FlywheelHardwareConfig(
           new int[] {1},         // canIds
           new boolean[] {true},  // reversed
           2.0,                   // gearRatio
           40,                    // currentLimit
           "rio");                // canBus

In this example:

- ``1`` is the flywheel motor CAN ID
- ``true`` is the reversal setting for that motor
- ``2.0`` is the mechanism gear ratio (reduction ratio)
- ``40`` is the current limit
- ``"rio"`` is the CAN bus name

Example position-joint hardware config:

.. code-block:: java

   public static final PositionJointHardwareConfig CONFIG =
       new PositionJointHardwareConfig(
           new int[] {10},                 // canIds
           new boolean[] {true},           // reversed
           85.33333 * 2 * Math.PI,         // gearRatio
           40,                             // currentLimit
           GravityType.COSINE,             // gravity
           EncoderType.EXTERNAL_CANCODER,  // encoderType
           11,                             // encoderID
           Rotation2d.fromRotations(0.5),  // encoderOffset
           "rio");                         // canBus

In this example:

- ``10`` is the motor CAN ID
- ``true`` is the reversal setting
- ``85.33333 * 2 * Math.PI`` is the gear ratio converted for a rotational joint
- ``40`` is the current limit
- ``GravityType.COSINE`` selects the gravity model
- ``EncoderType.EXTERNAL_CANCODER`` indicates the feedback sensor type
- ``11`` is the encoder CAN ID
- ``Rotation2d.fromRotations(0.5)`` is the encoder offset
- ``"rio"`` is the CAN bus name

Gravity model for position joints:

- ``GravityType.CONSTANT``
  Use for mechanisms where gravity load is approximately constant across travel, such as many elevators.
- ``GravityType.COSINE``
  Use for arms, pivots, and other rotating joints where gravity effect changes with angle.
- ``GravityType.SINE``
  Exists in the shared constants type, but the code comments note it is not supported by TalonFX.

This field belongs in the config object because it describes the physical mechanism model the subsystem should use, not a tuned gain value.

How to verify
-------------

After updating IDs:

- Build the project successfully.
- Connect with vendor tools and confirm every device appears with the expected ID.
- Deploy to the robot.
- Watch for startup alerts reporting disconnected motors or encoders.
- Verify each mechanism responds when commanded.

If a subsystem reports a disconnected device, check the exact ID in the alert and compare it to the constants for that subsystem.


Expected result
---------------

- Every CAN device has a unique ID.
- Code constants match the physical hardware.
- Bus names match the actual robot network layout.
- External encoders and sensors are read from the correct IDs or ports.
- The robot starts without CAN conflict or disconnected-device errors.


Common mistakes
---------------

- **Duplicate CAN IDs**
  Two devices on the same CAN bus cannot share the same ID.
- **Right ID on the wrong bus**
  A device may have the correct ID but still fail if the code expects it on ``"rio"`` and it is actually on a CANivore, or the reverse.
- **Motor ID and encoder ID mixed up**
  This is common on steer modules and other mechanisms with both a motor and an external encoder.
- **Wrong encoder type**
  A DIO encoder and CANcoder do not use the same identifier path.
- **Constants updated, device not reflashed**
  If code changed but the hardware still has old IDs, the robot will not find the devices.
- **Physical labels do not match software labels**
  If the module called front left in code is wired as back right on the robot, debugging becomes much harder even when the IDs are technically valid.


Good practice
-------------

- Keep a wiring sheet with every device name, CAN ID, bus name, and physical location.
- Reserve ID ranges by subsystem so additions later are easier to manage.
- Label motors and encoders on the robot with their configured IDs.
- Recheck IDs whenever hardware is replaced.
