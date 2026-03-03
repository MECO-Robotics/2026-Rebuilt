package frc.robot.subsystems.drive.module;

import frc.robot.subsystems.drive.azimuth_motor.AzimuthMotorConstants.AzimuthMotorHardwareConfig;
import frc.robot.subsystems.drive.azimuth_motor.AzimuthMotorIO;
import frc.robot.subsystems.drive.azimuth_motor.AzimuthMotorIOSim;
import frc.robot.subsystems.drive.drive_motor.DriveMotorConstants.DriveMotorHardwareConfig;
import frc.robot.subsystems.drive.drive_motor.DriveMotorIO;
import frc.robot.subsystems.drive.drive_motor.DriveMotorIOSim;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

/**
 * Shared simulation bundle for one swerve module.
 *
 * <p>Provides the SIM implementation of {@link ModuleIO} by bundling split drive/azimuth IO
 * endpoints. If/when MapleSim-specific drive/steer IO implementations are added, wire them in here
 * so both endpoints share the same module simulation state.
 */
public class ModuleIOSim implements ModuleIO {
  private final DriveMotorIO driveMotorIO;
  private final AzimuthMotorIO azimuthMotorIO;

  private static String driveName(String moduleName) {
    return moduleName + "Drive";
  }

  private static String azimuthName(String moduleName) {
    return moduleName + "Steer";
  }

  /**
   * Creates a module simulation bundle from the current split sim IO implementations.
   *
   * @param driveName drive motor logging name
   * @param driveConfig drive motor hardware/sim config
   * @param azimuthName azimuth motor logging name
   * @param azimuthConfig azimuth motor hardware/sim config
   */
  public ModuleIOSim(
      String driveName,
      DriveMotorHardwareConfig driveConfig,
      String azimuthName,
      AzimuthMotorHardwareConfig azimuthConfig) {
    driveMotorIO = new DriveMotorIOSim(driveName, driveConfig);
    azimuthMotorIO = new AzimuthMotorIOSim(azimuthName, azimuthConfig);
  }

  /**
   * Creates a module simulation bundle from MapleSim module state.
   *
   * <p>This overload is the integration point for Maple-backed drive/azimuth IO implementations.
   * For now, it falls back to the standard split sim IO path until Maple-specific IO classes are
   * introduced.
   */
  public ModuleIOSim(
      SwerveModuleSimulation moduleSimulation,
      String driveName,
      DriveMotorHardwareConfig driveConfig,
      String azimuthName,
      AzimuthMotorHardwareConfig azimuthConfig) {
    this(driveName, driveConfig, azimuthName, azimuthConfig);
  }

  /** Creates a module simulation bundle using derived drive/steer names from a module name. */
  public static ModuleIOSim forModule(
      String moduleName,
      DriveMotorHardwareConfig driveConfig,
      AzimuthMotorHardwareConfig azimuthConfig) {
    return new ModuleIOSim(
        driveName(moduleName), driveConfig, azimuthName(moduleName), azimuthConfig);
  }

  /** Returns the drive motor IO endpoint for this module simulation. */
  @Override
  public DriveMotorIO driveMotorIO() {
    return driveMotorIO;
  }

  /** Returns the azimuth motor IO endpoint for this module simulation. */
  @Override
  public AzimuthMotorIO azimuthMotorIO() {
    return azimuthMotorIO;
  }
}
