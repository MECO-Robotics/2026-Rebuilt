package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.azimuth_motor.AzimuthMotorConstants.AzimuthMotorGains;
import frc.robot.subsystems.drive.azimuth_motor.AzimuthMotorIO;
import frc.robot.subsystems.drive.azimuth_motor.AzimuthMotorIOInputsAutoLogged;
import frc.robot.subsystems.drive.drive_motor.DriveMotorConstants.DriveMotorGains;
import frc.robot.subsystems.drive.drive_motor.DriveMotorIO;
import frc.robot.subsystems.drive.drive_motor.DriveMotorIOInputsAutoLogged;
import frc.robot.util.OnboardModuleState;
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
    return driveInputs.positionRotations;
  }

  public void setGains(DriveMotorGains driveGains, AzimuthMotorGains azimuthGains) {
    driveMotor.setGains(driveGains);
    azimuthMotor.setGains(azimuthGains);
  }
}
