package frc.robot.vision;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import org.mecorobotics.gamepiecevision.GamePieceVisionClient.DriveRequest;

public final class TankVisionDriveAdapter implements VisionDriveAdapter {
  private final DifferentialDrive drivetrain;
  private final double maxForward;
  private final double maxTurn;

  public TankVisionDriveAdapter(DifferentialDrive drivetrain, double maxForward, double maxTurn) {
    this.drivetrain = drivetrain;
    this.maxForward = maxForward;
    this.maxTurn = maxTurn;
  }

  @Override
  public void apply(DriveRequest request) {
    drivetrain.arcadeDrive(request.forward() * maxForward, request.turn() * maxTurn);
  }

  @Override
  public void stop() {
    drivetrain.stopMotor();
  }
}
