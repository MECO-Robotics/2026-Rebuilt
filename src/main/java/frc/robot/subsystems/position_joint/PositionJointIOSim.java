package frc.robot.subsystems.position_joint;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.constants.types.PositionJointConstants.GravityType;
import frc.robot.constants.types.PositionJointConstants.MechanismType;
import frc.robot.constants.types.PositionJointConstants.PositionJointGains;
import frc.robot.constants.types.PositionJointConstants.PositionJointHardwareConfig;
import frc.robot.util.feedforwards.PositionJointFeedforward;
import frc.robot.util.feedforwards.TunableArmFeedforward;
import frc.robot.util.feedforwards.TunableElevatorFeedforward;

/** Physics-simulation implementation of {@link PositionJointIO}. */
public class PositionJointIOSim implements PositionJointIO {
	private static final double DEFAULT_LINEAR_MIN_POSITION_METERS = -1.0;
	private static final double DEFAULT_LINEAR_MAX_POSITION_METERS = 1.0;

	private final String name;

	private final PositionJointHardwareConfig config;

	private final DCMotor gearBox;

	private final DCMotorSim rotationalSim;
	private final ElevatorSim linearSim;

	private final PIDController controller;
	private final PositionJointFeedforward feedforward;
	private final double feedforwardPositionAddition;

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
	 * @param name
	 *            subsystem/logging name
	 * @param config
	 *            hardware constants used to shape the simulation model
	 */
	public PositionJointIOSim(String name, PositionJointHardwareConfig config) {
		this(name, config, DCMotor.getKrakenX60Foc(config.canIds().length));
	}

	public PositionJointIOSim(String name, PositionJointHardwareConfig config, DCMotor simMotorModel) {
		this.name = name;

		this.config = config;

		int numMotors = config.canIds().length;

		assert numMotors > 0 && (numMotors == config.reversed().length);

		motorsConnected = new boolean[numMotors];
		motorPositions = new double[numMotors];
		motorVelocities = new double[numMotors];
		motorVoltages = new double[numMotors];
		motorCurrents = new double[numMotors];

		gearBox = simMotorModel;
		if (config.mechanismType() == MechanismType.LINEAR) {
			double drumRadiusMeters = config.outputRadiusMeters();
			if (drumRadiusMeters <= 0.0) {
				throw new IllegalArgumentException("Linear mechanism requires a positive output radius");
			}

			double motorRotationsPerMeter = config.gearRatio();
			double motorRadiansPerMeter = motorRotationsPerMeter * 2.0 * Math.PI;
			double carriageMassKg = config.momentOfInertiaKgMetersSquared() * motorRadiansPerMeter
					* motorRadiansPerMeter;

			rotationalSim = null;
			linearSim = new ElevatorSim(gearBox, motorRotationsPerMeter * 2.0 * Math.PI * drumRadiusMeters,
					carriageMassKg, drumRadiusMeters, DEFAULT_LINEAR_MIN_POSITION_METERS,
					DEFAULT_LINEAR_MAX_POSITION_METERS, config.gravityType() == GravityType.CONSTANT, 0.0);
		} else {
			double outputSideMoiKgMetersSquared = config.momentOfInertiaKgMetersSquared() * config.gearRatio()
					* config.gearRatio();

			rotationalSim = new DCMotorSim(
					LinearSystemId.createDCMotorSystem(gearBox, outputSideMoiKgMetersSquared, config.gearRatio()),
					gearBox);
			linearSim = null;
		}

		controller = new PIDController(0, 0, 0);
		if (config.gravityType() == GravityType.CONSTANT) {
			feedforward = new TunableElevatorFeedforward(0.0, 0.0, 0.0, 0.0);
			feedforwardPositionAddition = 0.0;
		} else {
			feedforward = new TunableArmFeedforward(0.0, 0.0, 0.0, 0.0);
			feedforwardPositionAddition = config.gravityType() == GravityType.SINE ? -Math.PI / 2.0 : 0.0;
		}
	}

	@Override
	public void updateInputs(PositionJointIOInputs inputs) {
		double mechanismPosition = getMechanismPosition();
		double mechanismVelocity = getMechanismVelocity();
		double ffPosition = mechanismPosition + feedforwardPositionAddition;
		inputVoltage = closedLoop
				? controller.calculate(mechanismPosition, positionSetpoint)
						+ feedforward.calculate(ffPosition, mechanismVelocity, velocitySetpoint, 0.02)
				: voltageSetpoint;
		setSimulationInputVoltage(inputVoltage);
		updateSimulation();

		mechanismPosition = getMechanismPosition();
		mechanismVelocity = getMechanismVelocity();

		inputs.outputPosition = mechanismPosition;
		inputs.desiredPosition = positionSetpoint;
		inputs.velocity = mechanismVelocity;
		inputs.desiredVelocity = velocitySetpoint;
		inputs.rotorPosition = mechanismPosition * config.gearRatio();

		for (int i = 0; i < config.canIds().length; i++) {
			motorsConnected[i] = true;

			motorPositions[i] = inputs.rotorPosition;
			motorVelocities[i] = mechanismVelocity * config.gearRatio();

			motorVoltages[i] = inputVoltage;
			motorCurrents[i] = getSimulationCurrentDrawAmps();
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
		// Sim plant does not model static friction; ignore kS in sim feedforward.
		feedforward.setGains(0.0, gains.kG(), gains.kV(), gains.kA());

		System.out.println(name + " gains set to " + gains);
	}

	/** Returns this joint's loggable subsystem name. */
	@Override
	public String getName() {
		return name;
	}

	private double getMechanismPosition() {
		if (config.mechanismType() == MechanismType.LINEAR) {
			return linearSim.getPositionMeters();
		}
		return rotationalSim.getAngularPosition().in(Rotations);
	}

	private double getMechanismVelocity() {
		if (config.mechanismType() == MechanismType.LINEAR) {
			return linearSim.getVelocityMetersPerSecond();
		}
		return rotationalSim.getAngularVelocity().in(RotationsPerSecond);
	}

	private void setSimulationInputVoltage(double voltage) {
		if (config.mechanismType() == MechanismType.LINEAR) {
			linearSim.setInputVoltage(voltage);
			return;
		}
		rotationalSim.setInputVoltage(voltage);
	}

	private void updateSimulation() {
		if (config.mechanismType() == MechanismType.LINEAR) {
			linearSim.update(0.02);
			return;
		}
		rotationalSim.update(0.02);
	}

	private double getSimulationCurrentDrawAmps() {
		if (config.mechanismType() == MechanismType.LINEAR) {
			return linearSim.getCurrentDrawAmps();
		}
		return rotationalSim.getCurrentDrawAmps();
	}
}
