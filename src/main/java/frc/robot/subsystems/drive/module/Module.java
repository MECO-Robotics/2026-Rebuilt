package frc.robot.subsystems.drive.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.drive.AzimuthMotorConstants.AzimuthMotorGains;
import frc.robot.constants.drive.AzimuthMotorConstants.AzimuthMotorHardwareConfig;
import frc.robot.constants.drive.DriveConstants;
import frc.robot.constants.drive.DriveMotorConstants.DriveMotorGains;
import frc.robot.constants.drive.DriveMotorConstants.DriveMotorHardwareConfig;
import frc.robot.subsystems.drive.azimuth_motor.AzimuthMotorIO;
import frc.robot.subsystems.drive.azimuth_motor.AzimuthMotorIOInputsAutoLogged;
import frc.robot.subsystems.drive.drive_motor.DriveMotorIO;
import frc.robot.subsystems.drive.drive_motor.DriveMotorIOInputsAutoLogged;
import frc.robot.util.OnboardModuleState;
import java.util.function.BiFunction;
import java.util.function.Supplier;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.littletonrobotics.junction.Logger;

/** Wrapper for one swerve module (drive motor + azimuth motor). */
public class Module {
  private final DriveMotorIO driveMotor;
  private final DriveMotorIOInputsAutoLogged driveInputs = new DriveMotorIOInputsAutoLogged();

  private final AzimuthMotorIO azimuthMotor;
  private final AzimuthMotorIOInputsAutoLogged azimuthInputs = new AzimuthMotorIOInputsAutoLogged();

  private final String driveName;
  private final String azimuthName;

  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  /**
   * Creates a swerve module wrapper.
   *
   * @param driveMotorIO drive motor IO implementation
   * @param azimuthMotorIO azimuth motor IO implementation
   */
  public Module(DriveMotorIO driveMotorIO, AzimuthMotorIO azimuthMotorIO) {
    driveMotor = driveMotorIO;
    azimuthMotor = azimuthMotorIO;

    driveName = driveMotorIO.getName();
    azimuthName = azimuthMotorIO.getName();
  }

  /** Creates a swerve module wrapper from a module IO bundle. */
  public Module(ModuleIO moduleIO) {
    this(moduleIO.driveMotorIO(), moduleIO.azimuthMotorIO());
  }

  /** Creates a mode-appropriate module from a real-hardware module factory. */
  public static Module fromMode(
      String moduleName,
      DriveMotorHardwareConfig driveConfig,
      AzimuthMotorHardwareConfig azimuthConfig,
      Supplier<ModuleIO> realFactory) {
    return new Module(ModuleIO.fromMode(moduleName, driveConfig, azimuthConfig, realFactory));
  }

  /**
   * Creates a mode-appropriate module from independently selected drive/azimuth real factory
   * builders. Names and configs are provided once and reused for both factories.
   */
  public static Module fromMode(
      String moduleName,
      DriveMotorHardwareConfig driveConfig,
      AzimuthMotorHardwareConfig azimuthConfig,
      BiFunction<String, DriveMotorHardwareConfig, Supplier<DriveMotorIO>> driveFactoryBuilder,
      BiFunction<String, AzimuthMotorHardwareConfig, Supplier<AzimuthMotorIO>>
          azimuthFactoryBuilder) {
    return fromMode(
        moduleName,
        driveConfig,
        azimuthConfig,
        ModuleIO.factory(
            driveFactoryBuilder.apply(moduleName + "Drive", driveConfig),
            azimuthFactoryBuilder.apply(moduleName + "Steer", azimuthConfig)));
  }

  /**
   * Creates a mode-appropriate module from independently selected drive/azimuth real factory
   * builders and a custom sim factory.
   */
  public static Module fromMode(
      String moduleName,
      DriveMotorHardwareConfig driveConfig,
      AzimuthMotorHardwareConfig azimuthConfig,
      BiFunction<String, DriveMotorHardwareConfig, Supplier<DriveMotorIO>> driveFactoryBuilder,
      BiFunction<String, AzimuthMotorHardwareConfig, Supplier<AzimuthMotorIO>>
          azimuthFactoryBuilder,
      Supplier<ModuleIO> simFactory) {
    return fromMode(
        moduleName,
        driveConfig,
        azimuthConfig,
        ModuleIO.factory(
            driveFactoryBuilder.apply(moduleName + "Drive", driveConfig),
            azimuthFactoryBuilder.apply(moduleName + "Steer", azimuthConfig)),
        simFactory);
  }

  /**
   * Creates a mode-appropriate module from independently selected drive/azimuth real factory
   * builders and optional Maple module simulation state.
   */
  public static Module fromMode(
      String moduleName,
      DriveMotorHardwareConfig driveConfig,
      AzimuthMotorHardwareConfig azimuthConfig,
      BiFunction<String, DriveMotorHardwareConfig, Supplier<DriveMotorIO>> driveFactoryBuilder,
      BiFunction<String, AzimuthMotorHardwareConfig, Supplier<AzimuthMotorIO>>
          azimuthFactoryBuilder,
      SwerveModuleSimulation moduleSimulation) {
    Supplier<ModuleIO> simFactory =
        moduleSimulation != null
            ? () ->
                new ModuleIOSim(
                    moduleSimulation,
                    moduleName + "Drive",
                    driveConfig,
                    moduleName + "Steer",
                    azimuthConfig)
            : ModuleIO.simFactory(moduleName, driveConfig, azimuthConfig);
    return fromMode(
        moduleName,
        driveConfig,
        azimuthConfig,
        driveFactoryBuilder,
        azimuthFactoryBuilder,
        simFactory);
  }

  /** Creates a mode-appropriate module from real-hardware and custom sim module factories. */
  public static Module fromMode(
      String moduleName,
      DriveMotorHardwareConfig driveConfig,
      AzimuthMotorHardwareConfig azimuthConfig,
      Supplier<ModuleIO> realFactory,
      Supplier<ModuleIO> simFactory) {
    return new Module(
        ModuleIO.fromMode(moduleName, driveConfig, azimuthConfig, realFactory, simFactory));
  }

  /**
   * Creates a TalonFX-based module with default sim/replay behavior and derived drive/steer names.
   */
  public static Module fromTalonFX(
      String moduleName,
      DriveMotorHardwareConfig driveConfig,
      AzimuthMotorHardwareConfig azimuthConfig) {
    return new Module(ModuleIO.fromTalonFX(moduleName, driveConfig, azimuthConfig));
  }

  /**
   * Creates a TalonFX-based module with custom simulation endpoints and derived drive/steer names.
   */
  public static Module fromTalonFX(
      String moduleName,
      DriveMotorHardwareConfig driveConfig,
      AzimuthMotorHardwareConfig azimuthConfig,
      ModuleIOSim simBundle) {
    Supplier<ModuleIO> simFactory = () -> simBundle;
    return new Module(ModuleIO.fromTalonFX(moduleName, driveConfig, azimuthConfig, simFactory));
  }

  public void periodic() {
    driveMotor.updateInputs(driveInputs);
    Logger.processInputs("Drive/" + driveName, driveInputs);

    azimuthMotor.updateInputs(azimuthInputs);
    Logger.processInputs("Drive/" + azimuthName, azimuthInputs);

    // Calculate positions for odometry
    int sampleCount = driveInputs.odometryTimestamps.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters =
          driveInputs.odometryDrivePositionsRad[i] * DriveConstants.driveWheelRadiusMeters;
      Rotation2d angle = azimuthInputs.odometryTurnPositions[i];
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }
  }

  /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
  public void runSetpoint(SwerveModuleState state, double azimuthVelocityFF) {
    // Optimize velocity setpoint
    state = OnboardModuleState.optimize(state, getAngle());
    Logger.recordOutput("Drive/" + azimuthName + "/goal", state.angle.getRotations());
    state.cosineScale(Rotation2d.fromRotations(azimuthInputs.outputPositionRotations));

    driveMotor.setVelocity(
        Units.radiansToRotations(
            state.speedMetersPerSecond / DriveConstants.driveWheelRadiusMeters));

    azimuthMotor.setPosition(state.angle.getRotations(), azimuthVelocityFF);

    // if (Math.abs(azimuthSetpoint.position - azimuthGoal.position) > 0.5) {
    //   azimuthSetpoint = azimuthGoal;
    // }
  }

  /** Runs the module with the specified output while controlling to zero degrees. */
  public void runCharacterization(double output) {
    driveMotor.setVoltage(output);
    azimuthMotor.setPosition(0.0, 0.0);
  }

  /** Disables all outputs to motors. */
  public void stop() {
    driveMotor.setVoltage(0.0);
    azimuthMotor.setVoltage(0.0);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(azimuthInputs.outputPositionRotations);
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return Units.rotationsToRadians(driveInputs.positionRotations)
        * DriveConstants.driveWheelRadiusMeters;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return Units.rotationsToRadians(driveInputs.velocityRotationsPerSecond)
        * DriveConstants.driveWheelRadiusMeters;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the module positions received this cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return driveInputs.odometryTimestamps;
  }

  /** Returns the module position in radians. */
  public double getWheelRadiusCharacterizationPosition() {
    return Units.rotationsToRadians(driveInputs.positionRotations);
  }

  /** Returns the module velocity in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    return driveInputs.velocityRotationsPerSecond;
  }

  public void setGains(DriveMotorGains driveGains, AzimuthMotorGains azimuthGains) {
    driveMotor.setGains(driveGains);
    azimuthMotor.setGains(azimuthGains);
  }
}
