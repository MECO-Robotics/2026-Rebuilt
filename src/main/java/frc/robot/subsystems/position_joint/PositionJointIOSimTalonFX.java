package frc.robot.subsystems.position_joint;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
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
import frc.robot.util.feedforwards.PositionJointFeedforward;
import frc.robot.util.feedforwards.TunableArmFeedforward;
import frc.robot.util.feedforwards.TunableElevatorFeedforward;

/** TalonFX-backed simulation implementation of {@link PositionJointIO}. */
public class PositionJointIOSimTalonFX implements PositionJointIO {
	private static final double DEFAULT_LINEAR_MIN_POSITION_METERS = -1.0;
	private static final double DEFAULT_LINEAR_MAX_POSITION_METERS = 1.0;

	private final String name;
	private final PositionJointHardwareConfig config;
	private final DCMotorSim rotationalSim;
	private final ElevatorSim linearSim;
	private final TalonFX[] motors;
	private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);
	private final DynamicMotionMagicVoltage dynamicPositionRequest = new DynamicMotionMagicVoltage(0, 0, 0);
	private final VoltageOut voltageRequest = new VoltageOut(0);
	private final CANcoder externalCancoder;
	private final PositionJointFeedforward feedforward;
	private final double feedforwardPositionAddition;
	private final boolean[] motorsConnected;
	private final double[] motorPositions;
	private final double[] motorVelocities;
	private final double[] motorVoltages;
	private final double[] motorCurrents;
	private double positionSetpoint = 0.0;
	private double velocitySetpoint = 0.0;

	public PositionJointIOSimTalonFX(String name, PositionJointHardwareConfig config, DCMotor simMotorModel) {
		this.name = name;
		this.config = config;
		int numMotors = config.canIds().length;
		motors = new TalonFX[numMotors];
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

		CANBus canBus = new CANBus(config.canBus());
		motors[0] = new TalonFX(config.canIds()[0], canBus);
		TalonFXConfiguration leaderConfig = createLeaderConfig(config);
		externalCancoder = createSimCancoder(config);
		if (externalCancoder != null) {
			leaderConfig.withFeedback(new FeedbackConfigs().withFeedbackRemoteSensorID(config.encoderID())
					.withSensorToMechanismRatio(1.0).withRotorToSensorRatio(config.gearRatio())
					.withFeedbackSensorSource(config
							.encoderType() == frc.robot.constants.types.PositionJointConstants.EncoderType.EXTERNAL_CANCODER
									? FeedbackSensorSourceValue.RemoteCANcoder
									: FeedbackSensorSourceValue.FusedCANcoder));
		}
		motors[0].getConfigurator().apply(leaderConfig);
		motors[0].getSimState().setMotorType(com.ctre.phoenix6.sim.TalonFXSimState.MotorType.KrakenX60);
		for (int i = 1; i < numMotors; i++) {
			motors[i] = new TalonFX(config.canIds()[i], canBus);
			MotorAlignmentValue alignment = config.reversed()[i]
					? MotorAlignmentValue.Opposed
					: MotorAlignmentValue.Aligned;
			motors[i].setControl(new Follower(motors[0].getDeviceID(), alignment));
			motors[i].getSimState().setMotorType(com.ctre.phoenix6.sim.TalonFXSimState.MotorType.KrakenX60);
		}
	}

	@Override
	public void updateInputs(PositionJointIOInputs inputs) {
		double currentPosition = getMechanismPosition();
		double currentVelocity = getMechanismVelocity();
		double availableVoltage = RobotController.getBatteryVoltage();
		syncTalonSimState(currentPosition, currentVelocity, availableVoltage);
		double appliedVoltage = motors[0].getSimState().getMotorVoltage();
		setSimulationInputVoltage(appliedVoltage);
		updateSimulation();

		double loadedBatteryVoltage = BatterySim.calculateDefaultBatteryLoadedVoltage(getSimulationCurrentDrawAmps());
		RoboRioSim.setVInVoltage(loadedBatteryVoltage);

		currentPosition = getMechanismPosition();
		currentVelocity = getMechanismVelocity();
		syncTalonSimState(currentPosition, currentVelocity, loadedBatteryVoltage);

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

	@Override
	public void setPosition(double position, double velocity) {
		positionSetpoint = position;
		velocitySetpoint = velocity;
		motors[0].setControl(positionRequest.withPosition(position));
	}

	@Override
	public boolean setPositionDynamic(double position, double maxVelocity, double maxAcceleration) {
		positionSetpoint = position;
		velocitySetpoint = 0.0;
		motors[0].setControl(dynamicPositionRequest.withPosition(position).withVelocity(Math.abs(maxVelocity))
				.withAcceleration(Math.abs(maxAcceleration)));
		return true;
	}

	@Override
	public void setVoltage(double voltage) {
		motors[0].setControl(voltageRequest.withOutput(voltage));
	}

	@Override
	public void setGains(PositionJointGains gains) {
		feedforward.setGains(0.0, gains.kG(), gains.kV(), gains.kA());
		GravityTypeValue gravity = config.gravityType() == GravityType.CONSTANT
				? GravityTypeValue.Elevator_Static
				: GravityTypeValue.Arm_Cosine;
		motors[0].getConfigurator().apply(new Slot0Configs().withKP(gains.kP()).withKI(gains.kI()).withKD(gains.kD())
				.withKV(gains.kV()).withKA(gains.kA()).withKS(gains.kS()).withKG(gains.kG()).withGravityType(gravity));
		motors[0].getConfigurator().apply(new MotionMagicConfigs().withMotionMagicCruiseVelocity(gains.kMaxVelo())
				.withMotionMagicAcceleration(gains.kMaxAccel()));
		System.out.println(name + " gains set to " + gains);
	}

	@Override
	public String getName() {
		return name;
	}

	private TalonFXConfiguration createLeaderConfig(PositionJointHardwareConfig config) {
		TalonFXConfiguration leader = new TalonFXConfiguration()
				.withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)
						.withInverted(config.reversed()[0]
								? InvertedValue.Clockwise_Positive
								: InvertedValue.CounterClockwise_Positive))
				.withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(config.currentLimit())
						.withSupplyCurrentLimitEnable(true));
		if (config.encoderType() == frc.robot.constants.types.PositionJointConstants.EncoderType.INTERNAL
				|| config.encoderType() == frc.robot.constants.types.PositionJointConstants.EncoderType.EXTERNAL_DIO) {
			leader.withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(config.gearRatio())
					.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor));
		}
		return leader;
	}

	private CANcoder createSimCancoder(PositionJointHardwareConfig config) {
		if (config.encoderType() != frc.robot.constants.types.PositionJointConstants.EncoderType.EXTERNAL_CANCODER
				&& config
						.encoderType() != frc.robot.constants.types.PositionJointConstants.EncoderType.EXTERNAL_CANCODER_PRO) {
			return null;
		}
		CANcoder encoder = new CANcoder(config.encoderID(), new CANBus(config.canBus()));
		encoder.getConfigurator()
				.apply(new CANcoderConfiguration().withMagnetSensor(
						new MagnetSensorConfigs().withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
								.withMagnetOffset(config.encoderOffset().getMeasure())));
		return encoder;
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

	private double getMechanismAcceleration() {
		if (config.mechanismType() == MechanismType.LINEAR) {
			return 0.0;
		}
		return rotationalSim.getAngularAcceleration().in(RotationsPerSecondPerSecond);
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

	private void syncTalonSimState(double mechanismPosition, double mechanismVelocity, double supplyVoltage) {
		double rotorPosition = mechanismPosition * config.gearRatio();
		double rotorVelocity = mechanismVelocity * config.gearRatio();
		double rotorAcceleration = getMechanismAcceleration() * config.gearRatio();
		for (TalonFX motor : motors) {
			motor.getSimState().setSupplyVoltage(supplyVoltage);
			motor.getSimState().setRawRotorPosition(rotorPosition);
			motor.getSimState().setRotorVelocity(rotorVelocity);
			motor.getSimState().setRotorAcceleration(rotorAcceleration);
		}
		if (externalCancoder != null) {
			externalCancoder.getSimState().setSupplyVoltage(supplyVoltage);
			externalCancoder.getSimState().setRawPosition(mechanismPosition);
			externalCancoder.getSimState().setVelocity(mechanismVelocity);
		}
	}
}
