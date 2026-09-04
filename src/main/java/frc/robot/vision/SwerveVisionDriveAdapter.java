package frc.robot.vision;

import org.mecorobotics.gamepiecevision.GamePieceVisionClient.DriveRequest;

public final class SwerveVisionDriveAdapter implements VisionDriveAdapter {
  public interface RobotRelativeDrive {
    void drive(double forward, double strafe, double turn);
  }

  private final RobotRelativeDrive drive;
  private final double maxLinear;
  private final double maxAngular;

  public SwerveVisionDriveAdapter(RobotRelativeDrive drive, double maxLinearMetersPerSecond,
      double maxAngularRadiansPerSecond) {
    this.drive = drive;
    this.maxLinear = maxLinearMetersPerSecond;
    this.maxAngular = maxAngularRadiansPerSecond;
  }

  @Override
  public void apply(DriveRequest request) {
    drive.drive(
        request.forward() * maxLinear,
        request.strafe() * maxLinear,
        request.turn() * maxAngular);
  }

  @Override
  public void stop() {
    drive.drive(0.0, 0.0, 0.0);
  }
}
