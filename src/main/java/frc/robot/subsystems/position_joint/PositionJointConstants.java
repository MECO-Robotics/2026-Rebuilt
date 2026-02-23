package frc.robot.subsystems.position_joint;

import edu.wpi.first.math.geometry.Rotation2d;

/** Shared constants and configuration records for the position-joint subsystem. */
public class PositionJointConstants {
  /** Gravity model used by feedforward/controller configuration. */
  public enum GravityType {
    CONSTANT,
    COSINE,
    // Not supported by TalonFX
    SINE
  }

  /** Supported sensor sources for mechanism position. */
  public enum EncoderType {
    INTERNAL,
    EXTERNAL_CANCODER,
    EXTERNAL_CANCODER_PRO,
    EXTERNAL_DIO,
    EXTERNAL_SPARK
  }

  /** Closed-loop tuning values and profiling constraints for a position joint. */
  public record PositionJointGains(
      double kP,
      double kI,
      double kD,
      double kS,
      double kG,
      double kV,
      double kA,
      double kMaxVelo,
      double kMaxAccel,
      double kMinPosition,
      double kMaxPosition,
      double kTolerance,
      double kDefaultSetpoint) {}

  // Position Joint Gear Ratio should be multiplied by Math.PI * 2 for rotation joints to convert
  // from rotations to radians
  /** Hardware mapping and mechanism-specific constants for one position joint instance. */
  public record PositionJointHardwareConfig(
      int[] canIds,
      boolean[] reversed,
      double gearRatio,
      double currentLimit,
      GravityType gravity,
      EncoderType encoderType,
      int encoderID,
      Rotation2d encoderOffset,
      String canBus) {}

  /** Reference tuning/config used as a template while creating new joints. */
  public static final PositionJointGains EXAMPLE_GAINS =
      new PositionJointGains(1.5, 0.0, 0.0, 0.5, 1.0, 2.0, 0.0, 10.0, 20.0, 0.0, Math.PI, 0.2, 0.0);

  /** Reference hardware config used as a template while creating new joints. */
  public static final PositionJointHardwareConfig EXAMPLE_CONFIG =
      new PositionJointHardwareConfig(
          new int[] {10},
          new boolean[] {true},
          85.33333 * 2 * Math.PI,
          40,
          GravityType.COSINE,
          EncoderType.EXTERNAL_CANCODER,
          11,
          Rotation2d.fromRotations(0.5),
          "");

  // -----------
  // Intake Constants
  // -----------
  public static final PositionJointGains INTAKE_RACK_GAINS =
      new PositionJointGains(1.5, 0.0, 0.0, 0.5, 1.0, 2.0, 0.0, 10.0, 20.0, 0.0, Math.PI, 0.2, 0.0);
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

  // -----------
  // Hood
  // -----------
  public static final PositionJointGains HOOD_GAINS =
      new PositionJointGains(1.5, 0.0, 0.0, 0.5, 1.0, 2.0, 0.0, 10.0, 20.0, 0.0, Math.PI, 0.2, 0.0);
  public static final PositionJointHardwareConfig HOOD_CONFIG =
      new PositionJointHardwareConfig(
          new int[] {33},
          new boolean[] {false},
          85.33333 * 2 * Math.PI,
          40,
          GravityType.COSINE,
          EncoderType.INTERNAL,
          0,
          Rotation2d.fromRotations(0),
          "");

  // ------------
  // Climber Constants TODO
  // ------------

}
