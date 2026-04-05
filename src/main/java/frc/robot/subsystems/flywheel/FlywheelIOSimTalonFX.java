package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.constants.types.FlywheelConstants.FlywheelGains;
import frc.robot.constants.types.FlywheelConstants.FlywheelHardwareConfig;

/** TalonFX-backed simulation implementation of {@link FlywheelIO}. */
public class FlywheelIOSimTalonFX implements FlywheelIO {
	private final String name;
	private final FlywheelHardwareConfig config;
	private final DCMotorSim plant;
	private final TalonFX[] motors;
	private final MotionMagicVelocityVoltage velocityRequest = new MotionMagicVelocityVoltage(0);
	private final VoltageOut voltageRequest = new VoltageOut(0);
	private final boolean[] motorsConnected;
	private final double[] motorPositions;
	private final double[] motorVelocities;
	private final double[] motorAccelerations;
	private final double[] motorVoltages;
	private final double[] motorCurrents;
	private double velocitySetpoint = 0.0;

	public FlywheelIOSimTalonFX(String name, FlywheelHardwareConfig config, DCMotor simMotorModel) {
		this.name = name;
		this.config = config;
		int numMotors = config.canIds().length;
		motors = new TalonFX[numMotors];
		motorsConnected = new boolean[numMotors];
		motorPositions = new double[numMotors];
		motorVelocities = new double[numMotors];
		motorAccelerations = new double[numMotors];
		motorVoltages = new double[numMotors];
		motorCurrents = new double[numMotors];

		plant = new DCMotorSim(LinearSystemId.createDCMotorSystem(simMotorModel,
				config.momentOfInertiaKgMetersSquared(), config.gearRatio()), simMotorModel);

		CANBus canBus = new CANBus(config.canBus());
		motors[0] = new TalonFX(config.canIds()[0], canBus);
		TalonFXConfiguration leaderConfig = new TalonFXConfiguration()
				.withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast)
						.withInverted(config.reversed()[0]
								? InvertedValue.Clockwise_Positive
								: InvertedValue.CounterClockwise_Positive))
				.withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(config.gearRatio())
						.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor))
				.withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(config.currentLimit())
						.withSupplyCurrentLimitEnable(true));
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
	public void updateInputs(FlywheelIOInputs inputs) {
		double availableVoltage = RobotController.getBatteryVoltage();
		double measuredPosition = plant.getAngularPositionRotations();
		double measuredVelocity = plant.getAngularVelocity().in(RotationsPerSecond);
		syncTalonSimState(measuredPosition, measuredVelocity, availableVoltage);
		double appliedVoltage = motors[0].getSimState().getMotorVoltage();
		plant.setInputVoltage(appliedVoltage);
		plant.update(0.02);

		double loadedBatteryVoltage = BatterySim.calculateDefaultBatteryLoadedVoltage(plant.getCurrentDrawAmps());
		RoboRioSim.setVInVoltage(loadedBatteryVoltage);

		measuredPosition = plant.getAngularPositionRotations();
		measuredVelocity = plant.getAngularVelocity().in(RotationsPerSecond);
		double measuredAcceleration = plant.getAngularAcceleration().in(RotationsPerSecondPerSecond);
		syncTalonSimState(measuredPosition, measuredVelocity, loadedBatteryVoltage);

		inputs.velocity = measuredVelocity;
		inputs.position = measuredPosition;
		inputs.desiredVelocity = velocitySetpoint;
		inputs.motorsConnected = motorsConnected;

		for (int i = 0; i < config.canIds().length; i++) {
			motorsConnected[i] = true;
			motorPositions[i] = measuredPosition;
			motorVelocities[i] = measuredVelocity;
			motorAccelerations[i] = measuredAcceleration;
			motorVoltages[i] = appliedVoltage;
			motorCurrents[i] = plant.getCurrentDrawAmps();
		}

		inputs.motorPositions = motorPositions;
		inputs.motorVelocities = motorVelocities;
		inputs.motorAccelerations = motorAccelerations;
		inputs.motorVoltages = motorVoltages;
		inputs.motorCurrents = motorCurrents;
	}

	@Override
	public void setVelocity(double velocity) {
		velocitySetpoint = velocity;
		motors[0].setControl(velocityRequest.withVelocity(velocity));
	}

	@Override
	public void setVoltage(double voltage) {
		motors[0].setControl(voltageRequest.withOutput(voltage));
	}

	@Override
	public void setGains(FlywheelGains gains) {
		motors[0].getConfigurator().apply(new Slot0Configs().withKP(gains.kP()).withKI(gains.kI()).withKD(gains.kD())
				.withKV(gains.kV()).withKA(gains.kA()).withKS(gains.kS()));
		motors[0].getConfigurator().apply(new MotionMagicConfigs().withMotionMagicAcceleration(gains.kMaxAccel()));
		System.out.println(name + " gains set to " + gains);
	}

	@Override
	public String getName() {
		return name;
	}

	private void syncTalonSimState(double mechanismPosition, double mechanismVelocity, double supplyVoltage) {
		double rotorPosition = mechanismPosition * config.gearRatio();
		double rotorVelocity = mechanismVelocity * config.gearRatio();
		double rotorAcceleration = plant.getAngularAcceleration().in(RotationsPerSecondPerSecond) * config.gearRatio();
		for (TalonFX motor : motors) {
			motor.getSimState().setSupplyVoltage(supplyVoltage);
			motor.getSimState().setRawRotorPosition(rotorPosition);
			motor.getSimState().setRotorVelocity(rotorVelocity);
			motor.getSimState().setRotorAcceleration(rotorAcceleration);
		}
	}
}
