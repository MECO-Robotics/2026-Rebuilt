package frc.robot.sim.position_joint;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.constants.types.PositionJointConstants.GravityType;
import frc.robot.constants.types.PositionJointConstants.MechanismType;
import frc.robot.constants.types.PositionJointConstants.PositionJointGains;
import frc.robot.constants.types.PositionJointConstants.PositionJointHardwareConfig;
import frc.robot.subsystems.position_joint.PositionJointIO;
import frc.robot.util.feedforwards.PositionJointFeedforward;
import frc.robot.util.feedforwards.TunableArmFeedforward;
import frc.robot.util.feedforwards.TunableElevatorFeedforward;

/** SparkMax-backed simulation implementation of {@link PositionJointIO}. */
public class PositionJointIOSimSparkMax implements PositionJointIO {
	private static final double DEFAULT_LINEAR_MIN_POSITION_METERS = -1.0;
	private static final double DEFAULT_LINEAR_MAX_POSITION_METERS = 1.0;

	private final String name;
	private final PositionJointHardwareConfig config;
	private final DCMotorSim rotationalSim;
	private final ElevatorSim linearSim;
	private final SparkMax[] motors;
	private final SparkMaxConfig leaderConfig;
	private final SparkMaxSim leaderSim;
	private final PositionJointFeedforward feedforward;
	private final double feedforwardPositionAddition;
	private final boolean[] motorsConnected;
	private final double[] motorPositions;
	private final double[] motorVelocities;
	private final double[] motorVoltages;
	private final double[] motorCurrents;
	private double positionSetpoint = 0.0;
	private double velocitySetpoint = 0.0;
	private double currentPosition = 0.0;
	private double currentVelocity = 0.0;
	private double maxMotionVelocity = Double.NaN;
	private double maxMotionAcceleration = Double.NaN;

	/**
	 * Creates a Spark Max simulation-backed joint using either an arm or elevator
	 * plant based on mechanism type.
	 */
	public PositionJointIOSimSparkMax(String name, PositionJointHardwareConfig config, DCMotor simMotorModel) {
		this.name = name;
		this.config = config;
		int numMotors = config.canIds().length;
		motors = new SparkMax[numMotors];
		motorsConnected = new boolean[numMotors];
		motorPositions = new double[numMotors];
		motorVelocities = new double[numMotors];
		motorVoltages = new double[numMotors];
		motorCurrents = new double[numMotors];

		if (config.mechanismType() == MechanismType.LINEAR) {
			double drumRadiusMeters = config.outputRadiusMeters();
			double motorRotationsPerMeter = config.gearRatio();
			double motorRadiansPerMeter = motorRotationsPerMeter * 2.0 * Math.PI;
			double carriageMassKg = config.momentOfInertiaKgMetersSquared() * motorRadiansPerMeter
					* motorRadiansPerMeter;
			rotationalSim = null;
			linearSim = new ElevatorSim(simMotorModel, motorRotationsPerMeter * 2.0 * Math.PI * drumRadiusMeters,
					carriageMassKg, drumRadiusMeters, DEFAULT_LINEAR_MIN_POSITION_METERS,
					DEFAULT_LINEAR_MAX_POSITION_METERS, config.gravityType() == GravityType.CONSTANT, 0.0);
		} else {
			double outputSideMoiKgMetersSquared = config.momentOfInertiaKgMetersSquared() * config.gearRatio()
					* config.gearRatio();
			rotationalSim = new DCMotorSim(
					LinearSystemId.createDCMotorSystem(simMotorModel, outputSideMoiKgMetersSquared, config.gearRatio()),
					simMotorModel);
			linearSim = null;
		}

		if (config.gravityType() == GravityType.CONSTANT) {
			feedforward = new TunableElevatorFeedforward(0.0, 0.0, 0.0, 0.0);
			feedforwardPositionAddition = 0.0;
		} else {
			feedforward = new TunableArmFeedforward(0.0, 0.0, 0.0, 0.0);
			feedforwardPositionAddition = config.gravityType() == GravityType.SINE ? -Math.PI / 2.0 : 0.0;
		}

		motors[0] = new SparkMax(config.canIds()[0], MotorType.kBrushless);
		leaderConfig = createLeaderConfig(config);
		motors[0].configure(leaderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
		for (int i = 1; i < numMotors; i++) {
			motors[i] = new SparkMax(config.canIds()[i], MotorType.kBrushless);
			motors[i].configure(new SparkMaxConfig().follow(motors[0], config.reversed()[i]).idleMode(IdleMode.kBrake),
					ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
		}
		leaderSim = new SparkMaxSim(motors[0], simMotorModel);
	}

	/** Advances the REV simulation model and publishes synthetic mechanism telemetry. */
	@Override
	public void updateInputs(PositionJointIOInputs inputs) {
		currentPosition = getMechanismPosition();
		currentVelocity = getMechanismVelocity();
		double availableVoltage = RobotController.getBatteryVoltage();
		leaderSim.iterate(currentVelocity, availableVoltage, 0.02);
		double appliedVoltage = leaderSim.getAppliedOutput() * availableVoltage;
		setSimulationInputVoltage(appliedVoltage);
		updateSimulation();

		double loadedBatteryVoltage = BatterySim.calculateDefaultBatteryLoadedVoltage(getSimulationCurrentDrawAmps());
		RoboRioSim.setVInVoltage(loadedBatteryVoltage);

		currentPosition = getMechanismPosition();
		currentVelocity = getMechanismVelocity();

		inputs.outputPosition = currentPosition;
		inputs.rotorPosition = currentPosition * config.gearRatio();
		inputs.desiredPosition = positionSetpoint;
		inputs.velocity = currentVelocity;
		inputs.desiredVelocity = velocitySetpoint;
		inputs.encoderConnected = config
				.encoderType() != frc.robot.constants.types.PositionJointConstants.EncoderType.INTERNAL;

		for (int i = 0; i < config.canIds().length; i++) {
			motorsConnected[i] = true;
			motorPositions[i] = inputs.rotorPosition;
			motorVelocities[i] = currentVelocity * config.gearRatio();
			motorVoltages[i] = appliedVoltage;
			motorCurrents[i] = getSimulationCurrentDrawAmps();
		}

		inputs.motorsConnected = motorsConnected;
		inputs.motorPositions = motorPositions;
		inputs.motorVelocities = motorVelocities;
		inputs.motorVoltages = motorVoltages;
		inputs.motorCurrents = motorCurrents;
	}

	/** Commands the sim joint to a position using the configured MAXMotion profile. */
	@Override
	public void setPosition(double position, double velocity) {
		positionSetpoint = position;
		velocitySetpoint = velocity;
		ensureMaxMotionConfig(maxMotionVelocity, maxMotionAcceleration);
		double ffPosition = currentPosition + feedforwardPositionAddition;
		motors[0].getClosedLoopController().setSetpoint(positionSetpoint, ControlType.kMAXMotionPositionControl,
				ClosedLoopSlot.kSlot0, feedforward.calculate(ffPosition, currentVelocity, velocity, 0.02));
	}

	/** Commands the sim joint with temporary MAXMotion cruise constraints. */
	@Override
	public boolean setPositionDynamic(double position, double maxVelocity, double maxAcceleration) {
		positionSetpoint = position;
		velocitySetpoint = 0.0;
		ensureMaxMotionConfig(Math.abs(maxVelocity), Math.abs(maxAcceleration));
		double ffPosition = currentPosition + feedforwardPositionAddition;
		motors[0].getClosedLoopController().setSetpoint(positionSetpoint, ControlType.kMAXMotionPositionControl,
				ClosedLoopSlot.kSlot0, feedforward.calculate(ffPosition, currentVelocity, 0.0, 0.02));
		return true;
	}

	/** Applies open-loop voltage directly to the simulated leader Spark Max. */
	@Override
	public void setVoltage(double voltage) {
		motors[0].setVoltage(voltage);
	}

	/** Updates PID, feedforward, and MAXMotion limits on the simulated controller. */
	@Override
	public void setGains(PositionJointGains gains) {
		feedforward.setGains(0.0, gains.kG(), gains.kV(), gains.kA());
		maxMotionVelocity = gains.kMaxVelo();
		maxMotionAcceleration = gains.kMaxAccel();
		motors[0].configure(leaderConfig.apply(new ClosedLoopConfig().pid(gains.kP(), gains.kI(), gains.kD())
				.apply(new MAXMotionConfig().cruiseVelocity(maxMotionVelocity).maxAcceleration(maxMotionAcceleration)
						.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal))),
				ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
		System.out.println(name + " gains set to " + gains);
	}

	/** Returns the logging name associated with this simulated joint. */
	@Override
	public String getName() {
		return name;
	}

	/** Builds the shared base configuration for the leader Spark Max. */
	private SparkMaxConfig createLeaderConfig(PositionJointHardwareConfig config) {
		SparkMaxConfig leader = new SparkMaxConfig();
		leader.apply(new EncoderConfig().positionConversionFactor(1.0 / config.gearRatio())
				.velocityConversionFactor(1.0 / (60.0 * config.gearRatio())));
		leader.inverted(config.reversed()[0]).smartCurrentLimit(config.currentLimit()).idleMode(IdleMode.kBrake);
		if (config.encoderType() == frc.robot.constants.types.PositionJointConstants.EncoderType.EXTERNAL_SPARK) {
			leader.apply(new AbsoluteEncoderConfig().positionConversionFactor(1.0).velocityConversionFactor(1.0)
					.zeroOffset(config.encoderOffset().getRotations()).averageDepth(2));
		}
		return leader;
	}

	/** Reconfigures MAXMotion only when the requested limits actually change. */
	private void ensureMaxMotionConfig(double velocity, double acceleration) {
		if (Double.compare(maxMotionVelocity, velocity) == 0
				&& Double.compare(maxMotionAcceleration, acceleration) == 0) {
			return;
		}
		maxMotionVelocity = velocity;
		maxMotionAcceleration = acceleration;
		motors[0].configureAsync(
				leaderConfig.apply(new ClosedLoopConfig().apply(
						new MAXMotionConfig().cruiseVelocity(maxMotionVelocity).maxAcceleration(maxMotionAcceleration)
								.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal))),
				ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
	}

	/** Returns the simulated mechanism position in subsystem units. */
	private double getMechanismPosition() {
		if (config.mechanismType() == MechanismType.LINEAR) {
			return linearSim.getPositionMeters();
		}
		return rotationalSim.getAngularPosition().in(Rotations);
	}

	/** Returns the simulated mechanism velocity in subsystem units per second. */
	private double getMechanismVelocity() {
		if (config.mechanismType() == MechanismType.LINEAR) {
			return linearSim.getVelocityMetersPerSecond();
		}
		return rotationalSim.getAngularVelocity().in(RotationsPerSecond);
	}

	/** Routes the current applied voltage into the appropriate simulation model. */
	private void setSimulationInputVoltage(double voltage) {
		if (config.mechanismType() == MechanismType.LINEAR) {
			linearSim.setInputVoltage(voltage);
			return;
		}
		rotationalSim.setInputVoltage(voltage);
	}

	/** Advances the currently active arm/elevator simulation by one robot loop. */
	private void updateSimulation() {
		if (config.mechanismType() == MechanismType.LINEAR) {
			linearSim.update(0.02);
			return;
		}
		rotationalSim.update(0.02);
	}

	/** Returns the simulated current draw for battery loading calculations. */
	private double getSimulationCurrentDrawAmps() {
		if (config.mechanismType() == MechanismType.LINEAR) {
			return linearSim.getCurrentDrawAmps();
		}
		return rotationalSim.getCurrentDrawAmps();
	}
}
