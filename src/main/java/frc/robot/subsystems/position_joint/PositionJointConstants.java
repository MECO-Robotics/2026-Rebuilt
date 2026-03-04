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
  /**
   * Hardware mapping and mechanism-specific constants for one position joint instance.
   *
   * @param canIds CAN IDs of the motors in the flywheel mechanism, in order from closest to
   *     furthest from the shooter.
   * @param reversed Whether each motor is reversed. First boolean corresponds to clockwise /
   *     positive, rest of the booleans correspond to whether each subsequent motor is reversed
   *     relative to the first motor.
   * @param gearRatio The gear ratio between the motor and the joint output (output speed / motor
   *     speed). For rotation joints, this gear ratio should be multiplied by 2 * Math.PI to convert
   *     from rotations to radians.
   * @param momentOfInertiaKgMetersSquared Joint mechanism moment of inertia in kg*m^2 for sim
   *     modeling.
   * @param currentLimit The current limit for the motors in amps.
   * @param gravity The gravity model used for feedforward and compensation.
   * @param encoderType The type of encoder used for position feedback.
   * @param encoderID The ID of the encoder. For external encoders, this is the CAN ID for / a
   *     CANCoder or the DIO port for a digital encoder. For internal encoders, this can be set to 0
   *     or ignored.
   * @param encoderOffset The offset to apply to the encoder reading to get the joint position / in
   *     the correct reference frame. For example, if the joint's zero position corresponds to the
   *     encoder reading of 0.5 rotations, this would be set to Rotation2d.fromRotations(0.5).
   * @param canBus The CAN bus the motors are on, or an empty string for the rio bus.
   */
  public record PositionJointHardwareConfig(
      int[] canIds,
      boolean[] reversed,
      double gearRatio,
      double momentOfInertiaKgMetersSquared,
      int currentLimit,
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
          0.01,
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
          ((48 / 16) * (26 / 16)) / (((Math.PI * 10) / 10) * 0.0254),
          0.01,
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
      new PositionJointGains(30, 0.0, 0.0, 0.5, 1.0, 0.0, 0.0, 1.0, 2, 0.0, 0.064, 0.2, 0.0);
  public static final PositionJointHardwareConfig HOOD_CONFIG =
      new PositionJointHardwareConfig(
          new int[] {33},
          new boolean[] {false},
          (21 / 1),
          0.01,
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
