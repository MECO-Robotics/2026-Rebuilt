package frc.robot.subsystems.drive.azimuth_motor;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.DriveConstants;

public class AzimuthMotorConstants {
  // by default, the drive is set to the RoboRio's CANBus (you can also make it the rio it by doing
  // "")
  // change this value if using CANivore to CANivore's Bus name, set in Phoenix Tuner X
  // (if necessary, do this in DriveMotorConstants.java if drive motors are connected
  // to CANivore as well)
  public static final String canBusName = "MECO CANIvore";

  public record AzimuthMotorGains(
      double kP, double kI, double kD, double kS, double kV, double kA) {}

  public enum EncoderType {
    INTERNAL,
    EXTERNAL_CANCODER,
    EXTERNAL_CANCODER_PRO,
    EXTERNAL_DIO,
    EXTERNAL_SPARK
  }

  public record AzimuthMotorHardwareConfig(
      int[] canIds,
      boolean[] reversed,
      double gearRatio,
      double currentLimit,
      EncoderType encoderType,
      int encoderID,
      Rotation2d encoderOffset,
      String canBus) {}

  public static final AzimuthMotorHardwareConfig FRONT_LEFT_CONFIG =
      new AzimuthMotorHardwareConfig(
          new int[] {1},
          new boolean[] {false},
          DriveConstants.steerMotorGearRatio,
          40,
          EncoderType.EXTERNAL_CANCODER_PRO,
          21,
          Rotation2d.fromRotations(-0.188965),
          canBusName);

  public static final AzimuthMotorHardwareConfig FRONT_RIGHT_CONFIG =
      new AzimuthMotorHardwareConfig(
          new int[] {3},
          new boolean[] {false},
          DriveConstants.steerMotorGearRatio,
          40,
          EncoderType.EXTERNAL_CANCODER_PRO,
          22,
          Rotation2d.fromRotations(0.302490),
          canBusName);

  public static final AzimuthMotorHardwareConfig BACK_LEFT_CONFIG =
      new AzimuthMotorHardwareConfig(
          new int[] {5},
          new boolean[] {false},
          DriveConstants.steerMotorGearRatio,
          40,
          EncoderType.EXTERNAL_CANCODER_PRO,
          23,
          Rotation2d.fromRotations(0.404053),
          canBusName);

  public static final AzimuthMotorHardwareConfig BACK_RIGHT_CONFIG =
      new AzimuthMotorHardwareConfig(
          new int[] {7},
          new boolean[] {false},
          DriveConstants.steerMotorGearRatio,
          40,
          EncoderType.EXTERNAL_CANCODER_PRO,
          24,
          Rotation2d.fromRotations(-0.348877),
          canBusName);

  public static final AzimuthMotorGains EXAMPLE_GAINS = new AzimuthMotorGains(15, 0, 0, 0.25, 2, 0);

  public static final AzimuthMotorGains EXAMPLE_GAINS_SIM =
      new AzimuthMotorGains(35, 0, 0, 0.0, 3, 0);
}
