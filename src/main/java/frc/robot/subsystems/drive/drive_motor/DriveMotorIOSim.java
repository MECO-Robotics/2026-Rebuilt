package frc.robot.subsystems.drive.drive_motor;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.DriveMotorConstants.DriveMotorGains;
import frc.robot.constants.DriveMotorConstants.DriveMotorHardwareConfig;
import frc.robot.util.feedforwards.TunableSimpleMotorFeedforward;

/**
 * Physics sim implementation of module IO. The sim models are configured using a set of module
 * constants from Phoenix. Simulation is always based on voltage control.
 */
public class DriveMotorIOSim implements DriveMotorIO {
  private final String name;

  private final DriveMotorHardwareConfig config;

  private final DCMotor gearBox;

  private final DCMotorSim sim;

  private final PIDController controller;
  private final TunableSimpleMotorFeedforward feedforward;

  private final boolean[] motorsConnected;

  private final double[] motorPositions;
  private final double[] motorVelocities;

  private final double[] motorVoltages;
  private final double[] motorCurrents;

  private double driveAppliedVolts = 0.0;
  private double ffVolts = 0.0;

  private double velocitySetpoint = 0;

  private boolean driveClosedLoop = false;

  public DriveMotorIOSim(String name, DriveMotorHardwareConfig config) {
    this.name = name;

    this.config = config;

    int numMotors = config.canIds().length;

    assert numMotors > 0 && (numMotors == config.reversed().length);

    motorsConnected = new boolean[numMotors];
    motorPositions = new double[numMotors];
    motorVelocities = new double[numMotors];
    motorVoltages = new double[numMotors];
    motorCurrents = new double[numMotors];

    gearBox = DCMotor.getKrakenX60Foc(config.canIds().length);

    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(gearBox, 0.01, config.gearRatio()), gearBox);

    controller = new PIDController(0, 0, 0);
    feedforward = new TunableSimpleMotorFeedforward(0, 0, 0);
  }

  @Override
  public void updateInputs(DriveMotorIOInputs inputs) {
    // Run closed-loop control
    if (driveClosedLoop) {
      driveAppliedVolts =
          controller.calculate(sim.getAngularVelocity().in(RotationsPerSecond), velocitySetpoint)
              + ffVolts;
    } else {
      controller.reset();
    }

    // Update simulation state
    sim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
    sim.update(0.02);

    // Update drive inputs
    inputs.velocityRotationsPerSecond = sim.getAngularVelocity().in(RotationsPerSecond);
    inputs.desiredVelocityRotationsPerSecond = velocitySetpoint;

    inputs.positionRotations = sim.getAngularPositionRotations();

    for (int i = 0; i < config.canIds().length; i++) {
      motorsConnected[i] = true;
      motorPositions[i] = sim.getAngularPositionRotations();
      motorVelocities[i] = sim.getAngularVelocity().in(RotationsPerSecond);
      motorVoltages[i] = driveAppliedVolts;
      motorCurrents[i] = sim.getCurrentDrawAmps();
    }

    inputs.motorsConnected = motorsConnected;

    inputs.motorPositions = motorPositions;
    inputs.motorVelocities = motorVelocities;

    inputs.motorVoltages = motorVoltages;
    inputs.motorCurrents = motorCurrents;

    // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't matter)
    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryDrivePositionsRad =
        new double[] {Units.rotationsToRadians(inputs.positionRotations)};
  }

  @Override
  public void setVoltage(double output) {
    driveClosedLoop = false;
    driveAppliedVolts = output;
  }

  @Override
  public void setVelocity(double velocityRadPerSec) {
    driveClosedLoop = true;

    ffVolts = feedforward.calculateWithVelocities(velocitySetpoint, velocityRadPerSec);

    velocitySetpoint = velocityRadPerSec;
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
