package frc.robot.subsystems.position_joint;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.position_joint.PositionJointConstants.PositionJointGains;
import frc.robot.subsystems.position_joint.PositionJointConstants.PositionJointHardwareConfig;
import frc.robot.util.feedforwards.PositionJointFeedforward;
import frc.robot.util.feedforwards.TunableElevatorFeedforward;

/** Physics-simulation implementation of {@link PositionJointIO}. */
public class PositionJointIOSim implements PositionJointIO {
  private final String name;

  private final PositionJointHardwareConfig config;

  private final DCMotor gearBox;

  private final DCMotorSim sim;

  private final PIDController controller;
  private final PositionJointFeedforward feedforward;

  private final boolean[] motorsConnected;

  private final double[] motorPositions;
  private final double[] motorVelocities;

  private final double[] motorVoltages;
  private final double[] motorCurrents;

  private double positionSetpoint = 0.0;
  private double velocitySetpoint = 0.0;
  private double inputVoltage = 0.0;
  private double voltageSetpoint = 0.0;
  private boolean closedLoop = true;

  /**
   * Creates a simple DC-motor simulation for a position-controlled joint.
   *
   * @param name subsystem/logging name
   * @param config hardware constants used to shape the simulation model
   */
  public PositionJointIOSim(String name, PositionJointHardwareConfig config) {
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
    feedforward = new TunableElevatorFeedforward(0.0, 0.0, 0.0, 0.0);
  }

  @Override
  public void updateInputs(PositionJointIOInputs inputs) {
    inputVoltage =
        closedLoop
            ? controller.calculate(sim.getAngularPosition().in(Rotations), positionSetpoint)
                + feedforward.calculate(
                    sim.getAngularPositionRotations(),
                    sim.getAngularVelocity().in(RotationsPerSecond),
                    velocitySetpoint,
                    0.02)
            : voltageSetpoint;
    sim.setInputVoltage(inputVoltage);
    sim.update(0.02);

    inputs.outputPosition = sim.getAngularPosition().in(Rotations);
    inputs.desiredPosition = positionSetpoint;
    inputs.velocity = sim.getAngularVelocity().in(RotationsPerSecond);
    inputs.desiredVelocity = velocitySetpoint;

    for (int i = 0; i < config.canIds().length; i++) {
      motorsConnected[i] = true;

      motorPositions[i] = sim.getAngularPosition().in(Rotations);
      motorVelocities[i] = sim.getAngularVelocity().in(RotationsPerSecond);

      motorVoltages[i] = sim.getInputVoltage();
      motorCurrents[i] = sim.getCurrentDrawAmps();
    }

    inputs.motorsConnected = motorsConnected;

    inputs.motorPositions = motorPositions;
    inputs.motorVelocities = motorVelocities;

    inputs.motorVoltages = motorVoltages;
    inputs.motorCurrents = motorCurrents;
  }

  @Override
  public void setPosition(double position, double velocity) {
    closedLoop = true;
    positionSetpoint = position;
    velocitySetpoint = velocity;
  }

  @Override
  public void setVoltage(double voltage) {
    closedLoop = false;
    voltageSetpoint = voltage;
  }

  @Override
  public void setGains(PositionJointGains gains) {
    controller.setPID(gains.kP(), gains.kI(), gains.kD());
    feedforward.setGains(gains.kS(), gains.kG(), gains.kV(), gains.kA());

    System.out.println(name + " gains set to " + gains);
  }

  /** Returns this joint's loggable subsystem name. */
  @Override
  public String getName() {
    return name;
  }
}
