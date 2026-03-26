package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.constants.types.FlywheelConstants.FlywheelGains;
import frc.robot.constants.types.FlywheelConstants.FlywheelHardwareConfig;
import frc.robot.util.feedforwards.TunableSimpleMotorFeedforward;

/** SparkMax-backed simulation implementation of {@link FlywheelIO}. */
public class FlywheelIOSimSparkMax implements FlywheelIO {
	private final String name;
	private final FlywheelHardwareConfig config;
	private final DCMotorSim plant;
	private final SparkMax[] motors;
	private final SparkBaseConfig leaderConfig;
	private final SparkMaxSim leaderSim;
	private final TunableSimpleMotorFeedforward feedforward = new TunableSimpleMotorFeedforward(0, 0, 0);
	private final boolean[] motorsConnected;
	private final double[] motorPositions;
	private final double[] motorVelocities;
	private final double[] motorAccelerations;
	private final double[] motorVoltages;
	private final double[] motorCurrents;
	private double velocitySetpoint = 0.0;
	private double measuredVelocity = 0.0;

	public FlywheelIOSimSparkMax(String name, FlywheelHardwareConfig config, DCMotor simMotorModel) {
		this.name = name;
		this.config = config;
		int numMotors = config.canIds().length;
		motors = new SparkMax[numMotors];
		motorsConnected = new boolean[numMotors];
		motorPositions = new double[numMotors];
		motorVelocities = new double[numMotors];
		motorAccelerations = new double[numMotors];
		motorVoltages = new double[numMotors];
		motorCurrents = new double[numMotors];

		plant = new DCMotorSim(LinearSystemId.createDCMotorSystem(simMotorModel,
				config.momentOfInertiaKgMetersSquared(), config.gearRatio()), simMotorModel);

		motors[0] = new SparkMax(config.canIds()[0], MotorType.kBrushless);
		leaderConfig = new SparkMaxConfig()
				.apply(new EncoderConfig().positionConversionFactor(1.0 / config.gearRatio())
						.velocityConversionFactor(1.0 / (60.0 * config.gearRatio())))
				.inverted(config.reversed()[0]).smartCurrentLimit(config.currentLimit());
		motors[0].configure(leaderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
		for (int i = 1; i < numMotors; i++) {
			motors[i] = new SparkMax(config.canIds()[i], MotorType.kBrushless);
			motors[i].configure(new SparkMaxConfig().follow(motors[0], config.reversed()[i]).smartCurrentLimit(
					config.currentLimit()), ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
		}
		leaderSim = new SparkMaxSim(motors[0], simMotorModel);
	}

	@Override
	public void updateInputs(FlywheelIOInputs inputs) {
		double availableVoltage = RobotController.getBatteryVoltage();
		double measuredPosition = plant.getAngularPositionRotations();
		measuredVelocity = plant.getAngularVelocity().in(RotationsPerSecond);

		leaderSim.iterate(measuredVelocity, availableVoltage, 0.02);
		double appliedVoltage = leaderSim.getAppliedOutput() * availableVoltage;
		plant.setInputVoltage(appliedVoltage);
		plant.update(0.02);

		double loadedBatteryVoltage = BatterySim.calculateDefaultBatteryLoadedVoltage(plant.getCurrentDrawAmps());
		RoboRioSim.setVInVoltage(loadedBatteryVoltage);

		measuredPosition = plant.getAngularPositionRotations();
		measuredVelocity = plant.getAngularVelocity().in(RotationsPerSecond);
		double measuredAcceleration = plant.getAngularAcceleration().in(RotationsPerSecondPerSecond);

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
		motors[0].getClosedLoopController().setSetpoint(velocitySetpoint, ControlType.kMAXMotionVelocityControl,
				ClosedLoopSlot.kSlot0, feedforward.calculateWithVelocities(measuredVelocity, velocity));
	}

	@Override
	public void setVoltage(double voltage) {
		motors[0].setVoltage(voltage);
	}

	@Override
	public void setGains(FlywheelGains gains) {
		motors[0].configure(
				leaderConfig.apply(new ClosedLoopConfig().pid(gains.kP(), gains.kI(), gains.kD())
						.apply(new MAXMotionConfig().maxAcceleration(gains.kMaxAccel()))),
				ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
		feedforward.setGains(gains.kS(), gains.kV(), gains.kA());
		System.out.println(name + " gains set to " + gains);
	}

	@Override
	public String getName() {
		return name;
	}
}
