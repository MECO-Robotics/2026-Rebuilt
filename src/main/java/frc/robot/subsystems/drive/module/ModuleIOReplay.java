package frc.robot.subsystems.drive.module;

import frc.robot.subsystems.drive.azimuth_motor.AzimuthMotorIO;
import frc.robot.subsystems.drive.drive_motor.DriveMotorIO;

/** Replay-mode module IO that returns replay-backed drive and azimuth endpoints. */
public class ModuleIOReplay implements ModuleIO {
  private final DriveMotorIO driveMotorIO;
  private final AzimuthMotorIO azimuthMotorIO;

  public ModuleIOReplay(String moduleName) {
    driveMotorIO = DriveMotorIO.replayFactory(moduleName + "Drive").get();
    azimuthMotorIO = AzimuthMotorIO.replayFactory(moduleName + "Steer").get();
  }

  @Override
  public DriveMotorIO driveMotorIO() {
    return driveMotorIO;
  }

  @Override
  public AzimuthMotorIO azimuthMotorIO() {
    return azimuthMotorIO;
  }
}

