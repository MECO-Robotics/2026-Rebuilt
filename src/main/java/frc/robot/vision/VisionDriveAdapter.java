package frc.robot.vision;

import org.mecorobotics.gamepiecevision.GamePieceVisionClient.DriveRequest;

public interface VisionDriveAdapter {
  void apply(DriveRequest request);

  void stop();
}
