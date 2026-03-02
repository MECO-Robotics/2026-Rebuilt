# MECO 2026 Rebuilt Robot Code

This is the 2026 Rebuilt robot code used by MECO Robotics, built around reusable IO-based subsystems and AdvantageKit logging.

## 2026 Setup Checklist

1. Clone/fork 2026 Base Project repository.
2. Set your real hardware IDs and mechanism values in:
   - `src/main/java/frc/robot/subsystems/position_joint/PositionJointConstants.java`
   - `src/main/java/frc/robot/subsystems/flywheel/FlywheelConstants.java`
3. Instantiate the subsystems in `src/main/java/frc/robot/RobotContainer.java`.
4. Bind commands in `configureButtonBindings()`.
5. Tune gains on robot using AdvantageScope/logged tunables.
6. Save gains manually in Constants files from step 2.

## Subsystem Model

- `Flywheel`: velocity/voltage control for one motor group with one velocity setpoint.
- `PositionJoint`: position/voltage control for pivots/elevators with profiling and feedforward.

If one physical assembly needs independent setpoints, split it into multiple subsystems (example: left and right shooter wheels as two flywheels).

## Fill Out Flywheel Constants

Edit `FlywheelConstants` for each mechanism:

- `FlywheelHardwareConfig`
  - `canIds`: motor IDs, index `0` is leader
  - `reversed`: inversion for leader/followers
  - `gearRatio`: motor rotations per mechanism rotation
  - `momentOfInertiaKgMetersSquared`: mechanism MOI used by `FlywheelIOSim` (`kg*m^2`)
  - `currentLimit`: motor current limit
  - `canBus`: CANivore name or `""`/rio bus as used in your project
- `FlywheelGains`
  - `kP`, `kI`, `kD`: feedback
  - `kS`, `kV`, `kA`: feedforward
  - `kMaxAccel`: profile accel limit
  - `kTolerance`: at-speed tolerance

**Flywheel angular velocity setpoints are in RPS (rotations per second).**

Example pattern:

```java
public static final FlywheelHardwareConfig TOP_INDEXER_ROLLER_CONFIG =
    new FlywheelHardwareConfig(new int[] {32}, new boolean[] {false}, 1, 0.025, 40, "");
public static final FlywheelGains INDEXER_ROLLER_GAINS =
    new FlywheelGains(0.2, 0.0, 0.0, 0.0, 0.065, 0.0, 1.0, 1.0);
```

## Fill Out PositionJoint Constants

Edit `PositionJointConstants` for each joint:

- `PositionJointHardwareConfig`
  - `canIds`: motor IDs, index `0` is leader
  - `reversed`: inversion config, first bool is for clockwise positive and the rest are leader-relative
  - `gearRatio`: motor rotations per mechanism unit
  - `momentOfInertiaKgMetersSquared`: mechanism MOI used by `PositionJointIOSim` (`kg*m^2`)
  - `currentLimit`: motor current limit
  - `gravity`: `CONSTANT`, `COSINE`, or `SINE` (SINE not supported on TalonFX path)
  - `encoderType`: `INTERNAL`, `EXTERNAL_CANCODER`, `EXTERNAL_CANCODER_PRO`, `EXTERNAL_DIO`, `EXTERNAL_SPARK`
  - `encoderID`: CAN/DIO encoder ID if external
  - `encoderOffset`: absolute encoder offset (`Rotation2d`). Use a hardware client and set the 0 position to be either horizontal (cosine) or vertical (sine gravity)
  - `canBus`: CAN bus name used by CTRE devices
- `PositionJointGains`
  - feedback: `kP`, `kI`, `kD`
  - feedforward: `kS`, `kG`, `kV`, `kA`
  - profile: `kMaxVelo`, `kMaxAccel`
  - bounds: `kMinPosition`, `kMaxPosition`
  - tolerance/default: `kTolerance`, `kDefaultSetpoint`

Example pattern:

```java
public static final PositionJointHardwareConfig INTAKE_RACK_CONFIG =
    new PositionJointHardwareConfig(
        new int[] {21, 22},
        new boolean[] {true, true},
        80/12,
        0.01,
        40,
        GravityType.COSINE,
        EncoderType.INTERNAL,
        0,
        Rotation2d.fromRotations(.),
        "");
public static final PositionJointGains INTAKE_RACK_GAINS =
    new PositionJointGains(1.5, 0.0, 0.0, 0.5, 1.0, 2.0, 0.0, 10.0, 20.0, 0.0, Math.PI, 0.2, 0.0);
```

Note:
- This config is set to use the motor ids 21 and 22.
- The first motor inversion is set to true, so the motor is counterclockwise positive
- The second motor inversion is also set to true, so the second motor will spin opposite of the leader (first) motor
- The gear ratio is set to 80/12, so that could be a 12t input pinion on a 80t output gear
- The current limit is set to 40 amps
- The gravity model is set to COSINE
- The encoder type is set to internal (motor encoder)
- The encoder ID is set to zero (unused for an internal encoder)
- The offset is set to .1234 Rotations (Found in REV Hardware Client for horizontal position parallel to the ground)
- The CAN bus is set to default (`rio`)

## Wire Constants Into RobotContainer

Instantiate each mechanism with the intended IO backend:

```java
intakeRack =
    new PositionJoint(
        new PositionJointIOSparkMax("IntakeRack", PositionJointConstants.INTAKE_RACK_CONFIG),
        PositionJointConstants.INTAKE_RACK_GAINS);

topIndexer =
    new Flywheel(
        new FlywheelIOTalonFx("TopIndexer", FlywheelConstants.TOP_INDEXER_ROLLER_CONFIG),
        FlywheelConstants.INDEXER_ROLLER_GAINS);
```

Use `...IOTalonFX`, `...IOSparkMax`, `...IOSim`, or `...IOReplay` depending on mode/hardware.

This example code creates an intake rack position joint for a SparkMax (REV) and a Flywheel TalonFX (CTRE).

## Command Usage

Compose commands using WPILib command composition.

- Flywheel:
  - `new FlywheelVelocityCommand(<subsystem>, <doubleSupplier>)`
  - `new FlywheelVoltageCommand(<subsystem>, <doubleSupplier>)`
- PositionJoint:
  - `new PositionJointPositionCommand(<subsystem>, <doubleSupplier>)`
  - `new PositionJointVelocityCommand(<subsystem>, <doubleSupplier>)`

Or use command factories already in the project:

- `Flywheel.setVelocity(<subsystem>, <doubleSupplier>)`
- `Flywheel.setVoltage(<subsystem>, <doubleSupplier>)`
- `PositionJoint.setPosition(<subsystem>, <doubleSupplier>)`

```java
controller.a().whileTrue(Flywheel.setVelocity(shooter, vision::getRPS));
controller.b().whileFalse(new PositionJointPositionCommand(rack, ()->10));
```

Note:
- In the first line, the a button on the controller is used to set a velocity setpoint command
- The setpoint RPS is provided by the `getRPS` method
- The first line schedules the command when the button is held and deschedules when it is released.
- In the second line, the b button on the controller is used as a trigger to set a position setpoint command
- The setpoint position is provided by a lambda function as the input supplier: You can set the gear ratio constant for the rack subsystem config to make the input 10 = 10 inches
- The second line schedules the command when the button is released and deschedules when it is pressed.
## Notes

- Keep units consistent with your `gearRatio` and mechanism representation.
- `PositionJoint` limits (`kMinPosition`, `kMaxPosition`) are enforced in software.
- Talon/Spark deprecation warnings in vendor APIs do not block compilation by themselves.
