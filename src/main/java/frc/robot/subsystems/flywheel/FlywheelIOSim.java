package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.FlywheelConstants.FlywheelGains;
import frc.robot.constants.FlywheelConstants.FlywheelHardwareConfig;
import frc.robot.util.feedforwards.TunableSimpleMotorFeedforward;

/** Physics-simulation implementation of {@link FlywheelIO}. */
public class FlywheelIOSim implements FlywheelIO {
  private final String name;

  private final FlywheelHardwareConfig config;

  private final DCMotor gearBox;

  private final DCMotorSim sim;

  private final PIDController controller;
  private final TunableSimpleMotorFeedforward feedforward;

  private final double[] motorPositions;
  private final double[] motorVelocities;
  private final double[] motorAccelerations;

  private final double[] motorVoltages;
  private final double[] motorCurrents;

  private double velocitySetpoint = 0;
  private double voltageSetpoint = 0;
  private boolean closedLoop = true;

  /**
   * Creates a DC motor simulation for a flywheel mechanism.
   *
   * @param name subsystem/logging name
   * @param config hardware constants used to shape the simulation model
   */
  public FlywheelIOSim(String name, FlywheelHardwareConfig config) {
    this.name = name;

    this.config = config;

    int numMotors = config.canIds().length;

    assert numMotors > 0 && (numMotors == config.reversed().length);

    motorPositions = new double[numMotors];
    motorVelocities = new double[numMotors];
    motorAccelerations = new double[numMotors];

    motorVoltages = new double[numMotors];
    motorCurrents = new double[numMotors];
    gearBox = DCMotor.getKrakenX60Foc(config.canIds().length);

    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                gearBox, config.momentOfInertiaKgMetersSquared(), config.gearRatio()),
            gearBox);

    controller = new PIDController(0, 0, 0);
    feedforward = new TunableSimpleMotorFeedforward(0, 0, 0);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    double inputVoltage =
        closedLoop
            ? controller.calculate(sim.getAngularVelocityRPM(), velocitySetpoint)
                + feedforward.calculateWithVelocities(sim.getAngularVelocityRPM(), velocitySetpoint)
            : voltageSetpoint;
    sim.setInputVoltage(inputVoltage);
    sim.update(0.02);

    inputs.velocity = sim.getAngularVelocityRPM();
    inputs.position = sim.getAngularPositionRotations();
    inputs.desiredVelocity = velocitySetpoint;

    for (int i = 0; i < config.canIds().length; i++) {
      motorPositions[i] = sim.getAngularPositionRotations();
      motorVelocities[i] = sim.getAngularVelocity().in(RotationsPerSecond);
      motorAccelerations[i] = sim.getAngularAcceleration().in(RotationsPerSecondPerSecond);

      motorVoltages[i] = inputVoltage;
      motorCurrents[i] = sim.getCurrentDrawAmps();
    }

    inputs.motorPositions = motorPositions;
    inputs.motorVelocities = motorVelocities;
    inputs.motorAccelerations = motorAccelerations;

    inputs.motorVoltages = motorVoltages;
    inputs.motorCurrents = motorCurrents;
  }

  @Override
  public void setVelocity(double velocity) {
    closedLoop = true;
    velocitySetpoint = velocity;
  }

  @Override
  public void setVoltage(double voltage) {
    closedLoop = false;
    voltageSetpoint = voltage;
  }

  @Override
  public void setGains(FlywheelGains gains) {
    controller.setPID(gains.kP(), gains.kI(), gains.kD());
    feedforward.setGains(gains.kS(), gains.kV(), gains.kA());

    System.out.println(name + " gains set to " + gains);
  }

  /** Returns this flywheel's loggable subsystem name. */
  @Override
  public String getName() {
    return name;
  }
}
