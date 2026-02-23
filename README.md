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
  - `currentLimit`: motor current limit
  - `canBus`: CANivore name or `""`/rio bus as used in your project
- `FlywheelGains`
  - `kP`, `kI`, `kD`: feedback
  - `kS`, `kV`, `kA`: feedforward
  - `kMaxAccel`: profile accel limit
  - `kTolerance`: at-speed tolerance

Example pattern:

```java
public static final FlywheelHardwareConfig TOP_INDEXER_ROLLER_CONFIG =
    new FlywheelHardwareConfig(new int[] {32}, new boolean[] {false}, 1, 40, "");
public static final FlywheelGains INDEXER_ROLLER_GAINS =
    new FlywheelGains(0.2, 0.0, 0.0, 0.0, 0.065, 0.0, 1.0, 1.0);
```

## Fill Out PositionJoint Constants

Edit `PositionJointConstants` for each joint:

- `PositionJointHardwareConfig`
  - `canIds`: motor IDs, index `0` is leader
  - `reversed`: inversion config
  - `gearRatio`: motor rotations per mechanism unit
  - `currentLimit`: motor current limit
  - `gravity`: `CONSTANT`, `COSINE`, or `SINE` (SINE not supported on TalonFX path)
  - `encoderType`: `INTERNAL`, `EXTERNAL_CANCODER`, `EXTERNAL_CANCODER_PRO`, `EXTERNAL_DIO`, `EXTERNAL_SPARK`
  - `encoderID`: CAN/DIO encoder ID if external
  - `encoderOffset`: absolute encoder offset (`Rotation2d`)
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
        new int[] {21},
        new boolean[] {true},
        85.33333 * 2 * Math.PI,
        40,
        GravityType.COSINE,
        EncoderType.INTERNAL,
        0,
        Rotation2d.fromRotations(0),
        "");
public static final PositionJointGains INTAKE_RACK_GAINS =
    new PositionJointGains(1.5, 0.0, 0.0, 0.5, 1.0, 2.0, 0.0, 10.0, 20.0, 0.0, Math.PI, 0.2, 0.0);
```

## Wire Constants Into RobotContainer

Instantiate each mechanism with the intended IO backend:

```java
intakeRack =
    new PositionJoint(
        new PositionJointIOSparkMax("IntakeRack", PositionJointConstants.INTAKE_RACK_CONFIG),
        PositionJointConstants.INTAKE_RACK_GAINS);

topIndexer =
    new Flywheel(
        new FlywheelIOSparkMax("TopIndexer", FlywheelConstants.TOP_INDEXER_ROLLER_CONFIG),
        FlywheelConstants.INDEXER_ROLLER_GAINS);
```

Use `...IOTalonFX`, `...IOSparkMax`, `...IOSim`, or `...IOReplay` depending on mode/hardware.

## Command Usage

- Flywheel:
  - `flywheel.setVelocity(targetRps)`
  - `flywheel.setVoltage(volts)`
- PositionJoint:
  - `joint.setPosition(targetPosition)`
  - `joint.setVoltage(volts)`

Or use command factories already in the project:

- `Flywheel.setVelocity(...)`
- `Flywheel.setVoltage(...)`
- `PositionJoint.setPosition(...)`

## Notes

- Keep units consistent with your `gearRatio` and mechanism representation.
- `PositionJoint` limits (`kMinPosition`, `kMaxPosition`) are enforced in software.
- Talon/Spark deprecation warnings in vendor APIs do not block compilation by themselves.
