package frc.robot.vision;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public final class VisionDriveFactory {
  private VisionDriveFactory() {}

  public static VisionDriveAdapter create(
      DriveMode mode,
      DifferentialDrive tankDrive,
      double maxForwardOutput,
      double maxTurnOutput,
      SwerveVisionDriveAdapter.RobotRelativeDrive swerveDrive,
      double maxLinearMetersPerSecond,
      double maxAngularRadiansPerSecond) {
    return switch (mode) {
      case TANK -> {
        if (tankDrive == null) {
          throw new IllegalArgumentException("TANK mode requires a DifferentialDrive instance");
        }
        yield new TankVisionDriveAdapter(tankDrive, maxForwardOutput, maxTurnOutput);
      }
      case SWERVE -> {
        if (swerveDrive == null) {
          throw new IllegalArgumentException("SWERVE mode requires a robot-relative drive callback");
        }
        yield new SwerveVisionDriveAdapter(swerveDrive, maxLinearMetersPerSecond, maxAngularRadiansPerSecond);
      }
    };
  }
}
