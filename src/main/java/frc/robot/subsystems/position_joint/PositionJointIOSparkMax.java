package frc.robot.subsystems.position_joint;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

import frc.robot.constants.types.PositionJointConstants.GravityType;
import frc.robot.constants.types.PositionJointConstants.EncoderType;
import frc.robot.constants.types.PositionJointConstants.PositionJointGains;
import frc.robot.constants.types.PositionJointConstants.PositionJointHardwareConfig;
import frc.robot.util.encoder.AbsoluteCancoder;
import frc.robot.util.encoder.AbsoluteMagEncoder;
import frc.robot.util.encoder.IAbsoluteEncoder;
import frc.robot.util.feedforwards.PositionJointFeedforward;
import frc.robot.util.feedforwards.TunableArmFeedforward;
import frc.robot.util.feedforwards.TunableElevatorFeedforward;
import frc.robot.systemcheck.MotorHealthData;
import java.util.function.DoubleSupplier;

/** SparkMax-backed implementation of {@link PositionJointIO}. */
public class PositionJointIOSparkMax implements PositionJointIO {
	private final String name;

	private final PositionJointHardwareConfig hardwareConfig;

	private final DoubleSupplier externalFeedforward;

	private final SparkMax[] motors;
	private final SparkBaseConfig leaderConfig;

	private final IAbsoluteEncoder externalEncoder;

	private final boolean[] motorsConnected;
	private boolean encoderConnected;

	private final double[] motorPositions;
	private final double[] motorVelocities;

	private final double[] motorVoltages;
	private final double[] motorCurrents;
	private final REVLibError[] configurationResults;
	private REVLibError controlResult = REVLibError.kOk;
	private String controlStatus = "Not requested";
	private String encoderResetStatus = "Not requested";
	private String profileStatus = "Not requested";
	private String brakeStatus = "Not requested";

	private final Alert[] motorAlerts;
	private final Alert encoderAlert;
	private final double currentLimitAmps;
	private final boolean externalEncoderExpected;

	private final PositionJointFeedforward feedforward;
	private final double feedforward_position_addition;

	private double currentPosition = 0.0;
	private double positionSetpoint = 0.0;
	private double velocitySetpoint = 0.0;
	private double maxMotionVelocity = 0.0;
	private double maxMotionAcceleration = 0.0;
	private double minPosition = Double.NEGATIVE_INFINITY;
	private double maxPosition = Double.POSITIVE_INFINITY;
	private boolean brakeModeEnabled = true;

	/**
	 * Creates a SparkMax position-joint IO implementation.
	 *
	 * @param name
	 *            subsystem/logging name
	 * @param config
	 *            hardware configuration and encoder source
	 * @param externalFeedforward
	 *            additional feedforward term supplied by higher-level code
	 * @param isBrushless
	 *            true for NEO/brushless mode, false for brushed mode
	 */
	public PositionJointIOSparkMax(String name, PositionJointHardwareConfig config, DoubleSupplier externalFeedforward,
			boolean isBrushless) {
		this.name = name;
		hardwareConfig = config;
		currentLimitAmps = config.currentLimit();
		externalEncoderExpected = config.encoderType() != EncoderType.INTERNAL;
		this.externalFeedforward = externalFeedforward;

		int numMotors = config.canIds().length;

		assert numMotors > 0 && (numMotors == config.reversed().length);

		motors = new SparkMax[numMotors];
		motorsConnected = new boolean[numMotors];
		motorPositions = new double[numMotors];
		motorVelocities = new double[numMotors];
		motorVoltages = new double[numMotors];
		motorCurrents = new double[numMotors];
		configurationResults = new REVLibError[numMotors];
		java.util.Arrays.fill(configurationResults, REVLibError.kError);
		motorAlerts = new Alert[numMotors];

		motors[0] = new SparkMax(config.canIds()[0], isBrushless ? MotorType.kBrushless : MotorType.kBrushed);

		if (isBrushless) {
			leaderConfig = new SparkMaxConfig()
					.apply(new EncoderConfig().positionConversionFactor(1.0 / config.gearRatio())
							.velocityConversionFactor(1.0 / (60.0 * config.gearRatio())))
					.inverted(config.reversed()[0]).smartCurrentLimit(config.currentLimit()).idleMode(IdleMode.kBrake);

		} else {
			// NOTE: Brushed Motors does not support current limits! BE CAREFUL
			leaderConfig = new SparkMaxConfig()
					.apply(new EncoderConfig().positionConversionFactor(1.0 / config.gearRatio())
							.velocityConversionFactor(1.0 / (60.0 * config.gearRatio())).inverted(config.reversed()[0]))
					.idleMode(IdleMode.kBrake);
		}

		switch (config.encoderType()) {
			case INTERNAL :
				externalEncoder = new IAbsoluteEncoder() {
				};

				encoderAlert = new Alert(name, name + " does not use an external encoder ðŸ’€", AlertType.kInfo);

				configureMotor(0, leaderConfig, ResetMode.kResetSafeParameters);
				break;
			case EXTERNAL_CANCODER :
				externalEncoder = new AbsoluteCancoder(config.encoderID(), config.canBus(),
						new CANcoderConfiguration().withMagnetSensor(new MagnetSensorConfigs()
								.withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
								.withMagnetOffset(config.encoderOffset().getMeasure())));

				encoderAlert = new Alert(name, name + " CANCoder Disconnected! CAN ID: " + config.encoderID(),
						AlertType.kError);

				configureMotor(0, leaderConfig, ResetMode.kResetSafeParameters);
				encoderResetStatus = motors[0].getEncoder()
						.setPosition(externalEncoder.getAbsoluteAngle().getRotations()).name();
				break;
			case EXTERNAL_CANCODER_PRO :
				throw new IllegalArgumentException("EXTERNAL_CANCODER_PRO not supported on SparkMax");
			case EXTERNAL_DIO :
				externalEncoder = new AbsoluteMagEncoder(config.encoderID());

				encoderAlert = new Alert(name, name + " DIO Encoder Disconnected! DIO ID: " + config.encoderID(),
						AlertType.kWarning);

				configureMotor(0, leaderConfig, ResetMode.kResetSafeParameters);
				encoderResetStatus = motors[0].getEncoder()
						.setPosition(externalEncoder.getAbsoluteAngle().plus(config.encoderOffset()).getRotations())
						.name();
				break;
			case EXTERNAL_SPARK :
				externalEncoder = new IAbsoluteEncoder() {
				};

				encoderAlert = new Alert(name, name + " Internal SPARK Encoder Disconnected", AlertType.kWarning);

				leaderConfig
						.apply(new AbsoluteEncoderConfig().positionConversionFactor(1.0).velocityConversionFactor(1.0)
								.zeroOffset(config.encoderOffset().getRotations()).averageDepth(2));

				configureMotor(0, leaderConfig, ResetMode.kResetSafeParameters);
				encoderResetStatus = motors[0].getEncoder()
						.setPosition(
								motors[0].getAbsoluteEncoder().getPosition() + config.encoderOffset().getRotations())
						.name();
				break;

			default :
				externalEncoder = new IAbsoluteEncoder() {
				};
				encoderAlert = new Alert(name, name + " does not use an external encoder ðŸ’€", AlertType.kInfo);
				break;
		}

		motorAlerts[0] = new Alert(name, name + " Leader Motor Disconnected! CAN ID: " + config.canIds()[0],
				AlertType.kError);

		for (int i = 1; i < config.canIds().length; i++) {
			motors[i] = new SparkMax(config.canIds()[i], isBrushless ? MotorType.kBrushless : MotorType.kBrushed);
			configureMotor(i, new SparkMaxConfig().follow(motors[0], config.reversed()[i]).idleMode(IdleMode.kBrake),
					ResetMode.kResetSafeParameters);

			motorAlerts[i] = new Alert(name,
					name + " Follower Motor " + i + " Disconnected! CAN ID: " + config.canIds()[i], AlertType.kError);
		}

		if (config.gravityType() == GravityType.CONSTANT) {
			feedforward = new TunableElevatorFeedforward();
			feedforward_position_addition = 0.0;
		} else {
			feedforward = new TunableArmFeedforward();
			if (config.gravityType() == GravityType.SINE) {
				feedforward_position_addition = -Math.PI / 2;
			} else {
				feedforward_position_addition = 0.0;
			}
		}
	}

	@Override
	public void updateHealth(MotorHealthData health) {
		double[] temperatures = new double[motors.length];
		String[] faults = new String[motors.length];
		for (int i = 0; i < motors.length; i++) {
			temperatures[i] = motors[i].getMotorTemperature();
			StringBuilder description = new StringBuilder();
			if (motors[i].hasActiveFault()) {
				description.append("Faults: ").append(motors[i].getFaults());
			}
			if (motors[i].hasActiveWarning()) {
				if (!description.isEmpty()) {
					description.append("; ");
				}
				description.append("Warnings: ").append(motors[i].getWarnings());
			}
			faults[i] = description.toString();
		}
		health.temperaturesCelsius = temperatures;
		health.activeFaults = faults;
	}

	@Override
	public double getCurrentLimitAmps() {
		return currentLimitAmps;
	}

	@Override
	public int[] getDeviceIds() {
		return PositionJointIO.deviceIds(hardwareConfig);
	}

	@Override
	public boolean isExternalEncoderExpected() {
		return externalEncoderExpected;
	}

	public PositionJointIOSparkMax(String name, PositionJointHardwareConfig config) {
		this(name, config, () -> 0, true);
	}

	@Override
	public void updateInputs(PositionJointIOInputs inputs) {
		currentPosition = motors[0].getEncoder().getPosition();

		inputs.outputPosition = currentPosition;
		inputs.rotorPosition = currentPosition * hardwareConfig.gearRatio();
		inputs.desiredPosition = positionSetpoint;

		inputs.velocity = motors[0].getEncoder().getVelocity();
		inputs.desiredVelocity = velocitySetpoint;

		for (int i = 0; i < motors.length; i++) {
			motorsConnected[i] = motors[i].getLastError() == REVLibError.kOk;

			motorPositions[i] = motors[i].getEncoder().getPosition();
			motorVelocities[i] = motors[i].getEncoder().getVelocity();

			motorVoltages[i] = motors[i].getAppliedOutput() * motors[i].getBusVoltage();
			motorCurrents[i] = motors[i].getOutputCurrent();

			motorAlerts[i].set(!motorsConnected[i]);
		}

		inputs.motorsConnected = motorsConnected;

		inputs.motorPositions = motorPositions;
		inputs.motorVelocities = motorVelocities;

		inputs.motorVoltages = motorVoltages;
		inputs.motorCurrents = motorCurrents;

		switch (hardwareConfig.encoderType()) {
			case INTERNAL :
				encoderConnected = false;
				break;
			case EXTERNAL_CANCODER :
				encoderConnected = externalEncoder.isConnected();
				break;
			case EXTERNAL_CANCODER_PRO :
				encoderConnected = false;
				break;
			case EXTERNAL_DIO :
				encoderConnected = externalEncoder.isConnected();
				break;
			case EXTERNAL_SPARK :
				encoderConnected = motors[0].getLastError() == REVLibError.kOk;
				break;
		}

		encoderAlert.set(externalEncoderExpected && !encoderConnected);
		inputs.encoderConnected = encoderConnected;
		inputs.controllerDiagnosticsSupported = true;
		inputs.configurationHealthy = configurationHealthy();
		inputs.configurationStatus = java.util.Arrays.stream(configurationResults).map(Enum::name)
				.toArray(String[]::new);
		inputs.controlStatus = controlStatus;
		inputs.controlRequestHealthy = controlResult == REVLibError.kOk;
		inputs.encoderResetStatus = encoderResetStatus;
		inputs.profileStatus = profileStatus;
		inputs.brakeStatus = brakeStatus;
		inputs.positionUnits = hardwareConfig.mechanismType().name().equals("LINEAR")
				? "metres"
				: "configured rotations";
		inputs.positionConversionFactor = 1.0 / hardwareConfig.gearRatio();
		inputs.velocityConversionFactor = 1.0 / (60.0 * hardwareConfig.gearRatio());
		inputs.configuredMinPosition = minPosition;
		inputs.configuredMaxPosition = maxPosition;
		inputs.configuredMaxVelocity = maxMotionVelocity;
		inputs.configuredMaxAcceleration = maxMotionAcceleration;
		inputs.atReverseLimit = currentPosition <= minPosition;
		inputs.atForwardLimit = currentPosition >= maxPosition;
	}

	@Override
	public void setPosition(double desiredPosition, double desiredVelocity) {
		positionSetpoint = desiredPosition;
		ensureMaxMotionConfig(maxMotionVelocity, maxMotionAcceleration);
		velocitySetpoint = desiredVelocity;
		if (!configurationHealthy()) {
			motors[0].stopMotor();
			controlResult = REVLibError.kError;
			controlStatus = "Position blocked: controller configuration failed";
			return;
		}
		controlResult = motors[0].getClosedLoopController().setSetpoint(positionSetpoint,
				ControlType.kMAXMotionPositionControl);
		controlStatus = "MAXMotion position: " + controlResult.name();
	}

	@Override
	public boolean setPositionDynamic(double position, double maxVelocity, double maxAcceleration) {
		positionSetpoint = position;
		velocitySetpoint = 0.0;
		ensureMaxMotionConfig(Math.abs(maxVelocity), Math.abs(maxAcceleration));
		setPosition(position, 0);
		return true;
	}

	@Override
	public void setVoltage(double voltage) {
		if (voltage != 0 && !configurationHealthy()) {
			motors[0].stopMotor();
			controlResult = REVLibError.kError;
			controlStatus = "Voltage blocked: controller configuration failed";
			return;
		}
		motors[0].setVoltage(voltage);
		controlResult = motors[0].getLastError();
		controlStatus = "Voltage request: " + controlResult.name();
	}

	@Override
	public void setBrakeMode(boolean enabled) {
		if (brakeModeEnabled == enabled) {
			return;
		}

		brakeModeEnabled = enabled;
		IdleMode idleMode = enabled ? IdleMode.kBrake : IdleMode.kCoast;
		REVLibError queued = motors[0].configureAsync(new SparkMaxConfig().idleMode(idleMode),
				ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
		brakeStatus = "Async brake enqueue only (not controller acknowledgement): CAN " + hardwareConfig.canIds()[0]
				+ " " + queued.name();
		if (queued != REVLibError.kOk)
			configurationResults[0] = queued;
		for (int i = 1; i < motors.length; i++) {
			queued = motors[i].configureAsync(
					new SparkMaxConfig().follow(motors[0], hardwareConfig.reversed()[i]).idleMode(idleMode),
					ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
			brakeStatus += "; CAN " + hardwareConfig.canIds()[i] + " " + queued.name();
			if (queued != REVLibError.kOk)
				configurationResults[i] = queued;
		}
	}

	@Override
	public void setGains(PositionJointGains gains) {
		feedforward.setGains(gains.kS(), gains.kG(), gains.kV(), gains.kA());
		maxMotionVelocity = gains.kMaxVelo();
		maxMotionAcceleration = gains.kMaxAccel();
		minPosition = gains.kMinPosition();
		maxPosition = gains.kMaxPosition();

		configureMotor(0, leaderConfig.apply(new ClosedLoopConfig()
				.feedbackSensor(hardwareConfig
						.encoderType() == frc.robot.constants.types.PositionJointConstants.EncoderType.EXTERNAL_SPARK
								? FeedbackSensor.kAbsoluteEncoder
								: FeedbackSensor.kPrimaryEncoder)
				.pid(gains.kP(), gains.kI(), gains.kD()).outputRange(-1.0, 1.0)
				.apply(createBuiltInFeedforwardConfig(gains))
				.apply(new MAXMotionConfig().cruiseVelocity(maxMotionVelocity).maxAcceleration(maxMotionAcceleration)
						.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)))
				.apply(new SoftLimitConfig().forwardSoftLimit(maxPosition).forwardSoftLimitEnabled(true)
						.reverseSoftLimit(minPosition).reverseSoftLimitEnabled(true)),
				ResetMode.kResetSafeParameters);
		profileStatus = "Synchronous gains/profile apply: " + configurationResults[0].name();

		System.out.println(name + " gains set to " + gains);
	}

	@Override
	public void resetPosition() {
		for (int i = 0; i < motors.length; i++) {
			encoderResetStatus = "CAN " + hardwareConfig.canIds()[i] + ": "
					+ motors[i].getEncoder().setPosition(0.0).name();
		}
	}

	/** Returns this joint's loggable subsystem name. */
	@Override
	public String getName() {
		return name;
	}

	private void ensureMaxMotionConfig(double velocity, double acceleration) {
		if (Double.compare(maxMotionVelocity, velocity) == 0
				&& Double.compare(maxMotionAcceleration, acceleration) == 0) {
			return;
		}

		maxMotionVelocity = velocity;
		maxMotionAcceleration = acceleration;
		REVLibError queued = motors[0].configureAsync(
				leaderConfig.apply(new ClosedLoopConfig().apply(
						new MAXMotionConfig().cruiseVelocity(maxMotionVelocity).maxAcceleration(maxMotionAcceleration)
								.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal))),
				ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
		profileStatus = "Async profile enqueue only (not controller acknowledgement): " + queued.name();
		if (queued != REVLibError.kOk)
			configurationResults[0] = queued;
	}

	private boolean configurationHealthy() {
		return java.util.Arrays.stream(configurationResults).allMatch(result -> result == REVLibError.kOk);
	}

	private void configureMotor(int index, SparkBaseConfig config, ResetMode reset) {
		configurationResults[index] = motors[index].configure(config, reset, PersistMode.kNoPersistParameters);
	}

	private FeedForwardConfig createBuiltInFeedforwardConfig(PositionJointGains gains) {
		FeedForwardConfig config = new FeedForwardConfig().kS(gains.kS()).kA(gains.kA());
		if (hardwareConfig.gravityType() == GravityType.CONSTANT) {
			return config.kG(gains.kG());
		}
		return config.kCos(gains.kG()).kCosRatio(1.0);
	}
}
