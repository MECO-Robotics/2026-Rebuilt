package frc.robot.subsystems.drive.drive_motor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import frc.robot.constants.DriveMotorConstants.DriveMotorGains;
import frc.robot.constants.DriveMotorConstants.DriveMotorHardwareConfig;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.feedforwards.TunableSimpleMotorFeedforward;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController.GenericMotorController;

/** Maple-backed simulation implementation of drive motor IO. */
public class DriveMotorIOSimMaple implements DriveMotorIO {
  private final SwerveModuleSimulation moduleSimulation;
  private final GenericMotorController driveController;
  private final String name;
  private final DriveMotorHardwareConfig config;

  private final PIDController controller = new PIDController(0, 0, 0);
  private final TunableSimpleMotorFeedforward feedforward =
      new TunableSimpleMotorFeedforward(0, 0, 0);

  private final boolean[] motorsConnected;
  private final double[] motorPositions;
  private final double[] motorVelocities;
  private final double[] motorVoltages;
  private final double[] motorCurrents;

  private double driveAppliedVolts = 0.0;
  private double ffVolts = 0.0;
  private double velocitySetpoint = 0.0;
  private boolean driveClosedLoop = false;

  public DriveMotorIOSimMaple(
      SwerveModuleSimulation moduleSimulation, String name, DriveMotorHardwareConfig config) {
    this.moduleSimulation = moduleSimulation;
    this.driveController =
        moduleSimulation
            .useGenericMotorControllerForDrive()
            .withCurrentLimit(Amps.of(config.currentLimit()));
    this.name = name;
    this.config = config;

    int numMotors = config.canIds().length;
    motorsConnected = new boolean[numMotors];
    motorPositions = new double[numMotors];
    motorVelocities = new double[numMotors];
    motorVoltages = new double[numMotors];
    motorCurrents = new double[numMotors];
  }

  @Override
  public void updateInputs(DriveMotorIOInputs inputs) {
    double measuredVelocity = moduleSimulation.getDriveWheelFinalSpeed().in(RotationsPerSecond);

    if (driveClosedLoop) {
      driveAppliedVolts = controller.calculate(measuredVelocity, velocitySetpoint) + ffVolts;
    } else {
      controller.reset();
    }

    driveController.requestVoltage(Volts.of(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0)));

    inputs.velocityRotationsPerSecond = measuredVelocity;
    inputs.desiredVelocityRotationsPerSecond = velocitySetpoint;
    inputs.positionRotations = moduleSimulation.getDriveWheelFinalPosition().in(Rotations);

    for (int i = 0; i < config.canIds().length; i++) {
      motorsConnected[i] = true;
      motorPositions[i] = moduleSimulation.getDriveEncoderUnGearedPosition().in(Rotations);
      motorVelocities[i] = moduleSimulation.getDriveEncoderUnGearedSpeed().in(RotationsPerSecond);
      motorVoltages[i] = moduleSimulation.getDriveMotorAppliedVoltage().in(Volts);
      motorCurrents[i] = moduleSimulation.getDriveMotorStatorCurrent().in(Amps);
    }

    inputs.motorsConnected = motorsConnected;
    inputs.motorPositions = motorPositions;
    inputs.motorVelocities = motorVelocities;
    inputs.motorVoltages = motorVoltages;
    inputs.motorCurrents = motorCurrents;

    double[] timestamps = PhoenixUtil.getSimulationOdometryTimeStamps();
    Angle[] cachedPositions = moduleSimulation.getCachedDriveWheelFinalPositions();
    int sampleCount = Math.min(timestamps.length, cachedPositions.length);
    inputs.odometryTimestamps = new double[sampleCount];
    inputs.odometryDrivePositionsRad = new double[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      inputs.odometryTimestamps[i] = timestamps[i];
      inputs.odometryDrivePositionsRad[i] = cachedPositions[i].in(Radians);
    }
  }

  @Override
  public void setVoltage(double output) {
    driveClosedLoop = false;
    driveAppliedVolts = output;
  }

  @Override
  public void setVelocity(double velocityRotationsPerSecond) {
    driveClosedLoop = true;
    ffVolts = feedforward.calculateWithVelocities(velocitySetpoint, velocityRotationsPerSecond);
    velocitySetpoint = velocityRotationsPerSecond;
  }

  @Override
  public void setGains(DriveMotorGains gains) {
    controller.setPID(gains.kP(), gains.kI(), gains.kD());
    feedforward.setGains(gains.kS(), gains.kV(), gains.kA());
  }

  @Override
  public String getName() {
    return name;
  }
}
