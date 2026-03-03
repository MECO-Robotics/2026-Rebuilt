package frc.robot.subsystems.drive.module;

import frc.robot.Constants;
import frc.robot.subsystems.drive.azimuth_motor.AzimuthMotorConstants.AzimuthMotorHardwareConfig;
import frc.robot.subsystems.drive.azimuth_motor.AzimuthMotorIO;
import frc.robot.subsystems.drive.drive_motor.DriveMotorConstants.DriveMotorHardwareConfig;
import frc.robot.subsystems.drive.drive_motor.DriveMotorIO;
import java.util.function.Supplier;

/** Hardware abstraction bundle for one swerve module (drive + azimuth IO endpoints). */
public interface ModuleIO {
  /** Returns the drive motor IO endpoint for this module. */
  public DriveMotorIO driveMotorIO();

  /** Returns the azimuth motor IO endpoint for this module. */
  public AzimuthMotorIO azimuthMotorIO();

  private static String driveName(String moduleName) {
    return moduleName + "Drive";
  }

  private static String azimuthName(String moduleName) {
    return moduleName + "Steer";
  }

  /** Creates a module IO supplier from independently selected drive and azimuth factories. */
  public static Supplier<ModuleIO> factory(
      Supplier<DriveMotorIO> driveFactory, Supplier<AzimuthMotorIO> azimuthFactory) {
    return () ->
        new ModuleIO() {
          private final DriveMotorIO drive = driveFactory.get();
          private final AzimuthMotorIO azimuth = azimuthFactory.get();

          @Override
          public DriveMotorIO driveMotorIO() {
            return drive;
          }

          @Override
          public AzimuthMotorIO azimuthMotorIO() {
            return azimuth;
          }
        };
  }

  /** Creates a TalonFX-backed module IO supplier for real hardware. */
  public static Supplier<ModuleIO> talonFXFactory(
      String moduleName,
      DriveMotorHardwareConfig driveConfig,
      AzimuthMotorHardwareConfig azimuthConfig) {
    return factory(
        DriveMotorIO.talonFXFactory(driveName(moduleName), driveConfig),
        AzimuthMotorIO.talonFXFactory(azimuthName(moduleName), azimuthConfig));
  }

  /** Creates a default simulation module IO supplier. */
  public static Supplier<ModuleIO> simFactory(
      String moduleName,
      DriveMotorHardwareConfig driveConfig,
      AzimuthMotorHardwareConfig azimuthConfig) {
    return () -> ModuleIOSim.forModule(moduleName, driveConfig, azimuthConfig);
  }

  /** Creates a replay module IO supplier. */
  public static Supplier<ModuleIO> replayFactory(String moduleName) {
    return factory(
        DriveMotorIO.replayFactory(moduleName + "Drive"),
        AzimuthMotorIO.replayFactory(moduleName + "Steer"));
  }

  /**
   * Creates a mode-appropriate module IO.
   *
   * <p>Returns the supplied real implementation on real hardware, simulated IO in sim, and replay
   * IO during log replay.
   */
  public static ModuleIO fromMode(
      String moduleName,
      DriveMotorHardwareConfig driveConfig,
      AzimuthMotorHardwareConfig azimuthConfig,
      Supplier<ModuleIO> realFactory) {
    return fromMode(
        moduleName,
        driveConfig,
        azimuthConfig,
        realFactory,
        simFactory(moduleName, driveConfig, azimuthConfig));
  }

  /**
   * Creates a mode-appropriate module IO with a custom sim factory.
   *
   * <p>Allows callers to inject shared module simulation endpoints.
   */
  public static ModuleIO fromMode(
      String moduleName,
      DriveMotorHardwareConfig driveConfig,
      AzimuthMotorHardwareConfig azimuthConfig,
      Supplier<ModuleIO> realFactory,
      Supplier<ModuleIO> simFactory) {
    return switch (Constants.currentMode) {
      case REAL -> realFactory.get();
      case SIM -> simFactory.get();
      default -> replayFactory(moduleName).get();
    };
  }

  /** Creates mode-appropriate module IO using TalonFX for real hardware. */
  public static ModuleIO fromTalonFX(
      String moduleName,
      DriveMotorHardwareConfig driveConfig,
      AzimuthMotorHardwareConfig azimuthConfig) {
    return fromMode(
        moduleName,
        driveConfig,
        azimuthConfig,
        talonFXFactory(moduleName, driveConfig, azimuthConfig));
  }

  /** Creates mode-appropriate module IO using TalonFX for real hardware and custom sim IO. */
  public static ModuleIO fromTalonFX(
      String moduleName,
      DriveMotorHardwareConfig driveConfig,
      AzimuthMotorHardwareConfig azimuthConfig,
      Supplier<ModuleIO> simFactory) {
    return fromMode(
        moduleName,
        driveConfig,
        azimuthConfig,
        talonFXFactory(moduleName, driveConfig, azimuthConfig),
        simFactory);
  }
}
