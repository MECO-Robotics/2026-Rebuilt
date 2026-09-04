package frc.robot.constants.vision;

import frc.robot.constants.drive.DrivetrainConstants;
import frc.robot.vision.DriveMode;

public final class VisionPursuitConstants {
  public static final String CAMERA_NAME = "left";
  public static final DriveMode DRIVE_MODE = DriveMode.SWERVE;

  // Legacy tank tuning kept for compatibility with the common adapter factory.
  public static final double MAX_FORWARD_OUTPUT = 0.35;
  public static final double MAX_TURN_OUTPUT = 0.35;

  public static final double MAX_LINEAR_METERS_PER_SECOND = DrivetrainConstants.MAX_SPEED;
  public static final double MAX_ANGULAR_RADIANS_PER_SECOND = DrivetrainConstants.MAX_ANGULAR_RATE;
  public static final double MANUAL_OVERRIDE_DEADBAND = 0.15;
}
