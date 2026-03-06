package frc.robot.subsystems.drive.azimuth_motor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.drive.AzimuthMotorConstants.AzimuthMotorGains;
import frc.robot.constants.drive.AzimuthMotorConstants.AzimuthMotorHardwareConfig;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.feedforwards.TunableSimpleMotorFeedforward;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController.GenericMotorController;

/** Maple-backed simulation implementation of azimuth motor IO. */
public class AzimuthMotorIOSimMaple implements AzimuthMotorIO {
	private final SwerveModuleSimulation moduleSimulation;
	private final GenericMotorController steerController;
	private final String name;
	private final AzimuthMotorHardwareConfig config;

	private final PIDController controller = new PIDController(0, 0, 0);
	private final TunableSimpleMotorFeedforward feedforward = new TunableSimpleMotorFeedforward(0, 0, 0);

	private final boolean[] motorsConnected;
	private final double[] motorPositions;
	private final double[] motorVelocities;
	private final double[] motorVoltages;
	private final double[] motorCurrents;

	private double appliedVolts = 0.0;
	private double ffVolts = 0.0;
	private double positionSetpoint = 0.0;
	private double velocitySetpoint = 0.0;
	private boolean closedLoop = false;

	public AzimuthMotorIOSimMaple(SwerveModuleSimulation moduleSimulation, String name,
			AzimuthMotorHardwareConfig config) {
		this.moduleSimulation = moduleSimulation;
		this.steerController = moduleSimulation.useGenericControllerForSteer()
				.withCurrentLimit(Amps.of(config.currentLimit()));
		this.name = name;
		this.config = config;

		int numMotors = config.canIds().length;
		motorsConnected = new boolean[numMotors];
		motorPositions = new double[numMotors];
		motorVelocities = new double[numMotors];
		motorVoltages = new double[numMotors];
		motorCurrents = new double[numMotors];

		controller.enableContinuousInput(-0.5, 0.5);
	}

	@Override
	public void updateInputs(AzimuthMotorIOInputs inputs) {
		double measuredPosition = moduleSimulation.getSteerAbsoluteAngle().in(Rotations);
		double measuredVelocity = moduleSimulation.getSteerAbsoluteEncoderSpeed().in(RotationsPerSecond);

		if (closedLoop) {
			appliedVolts = controller.calculate(measuredPosition, positionSetpoint) + ffVolts;
		} else {
			controller.reset();
		}

		steerController.requestVoltage(Volts.of(MathUtil.clamp(appliedVolts, -12.0, 12.0)));

		inputs.velocityRotationsPerSecond = measuredVelocity;
		inputs.desiredVelocityRotationsPerSecond = velocitySetpoint;
		inputs.outputPositionRotations = measuredPosition;
		inputs.rotorPositionRotations = moduleSimulation.getSteerRelativeEncoderPosition().in(Rotations);
		inputs.desiredPositionRotations = positionSetpoint;
		inputs.encoderConnected = true;

		for (int i = 0; i < config.canIds().length; i++) {
			motorsConnected[i] = true;
			motorPositions[i] = inputs.rotorPositionRotations;
			motorVelocities[i] = moduleSimulation.getSteerRelativeEncoderVelocity().in(RotationsPerSecond);
			motorVoltages[i] = moduleSimulation.getSteerMotorAppliedVoltage().in(Volts);
			motorCurrents[i] = moduleSimulation.getSteerMotorStatorCurrent().in(Amps);
		}

		inputs.motorsConnected = motorsConnected;
		inputs.motorPositions = motorPositions;
		inputs.motorVelocities = motorVelocities;
		inputs.motorVoltages = motorVoltages;
		inputs.motorCurrents = motorCurrents;

		double[] timestamps = PhoenixUtil.getSimulationOdometryTimeStamps();
		Rotation2d[] cachedSteerPositions = moduleSimulation.getCachedSteerAbsolutePositions();
		int sampleCount = Math.min(timestamps.length, cachedSteerPositions.length);
		inputs.odometryTimestamps = new double[sampleCount];
		inputs.odometryTurnPositions = new Rotation2d[sampleCount];
		for (int i = 0; i < sampleCount; i++) {
			inputs.odometryTimestamps[i] = timestamps[i];
			inputs.odometryTurnPositions[i] = cachedSteerPositions[i];
		}
	}

	@Override
	public void setVoltage(double output) {
		closedLoop = false;
		appliedVolts = output;
	}

	@Override
	public void setPosition(double position, double velocity) {
		closedLoop = true;
		ffVolts = feedforward.calculateWithVelocities(velocitySetpoint, velocity);
		positionSetpoint = position;
		velocitySetpoint = velocity;
	}

	@Override
	public void setGains(AzimuthMotorGains gains) {
		controller.setPID(gains.kP(), gains.kI(), gains.kD());
		feedforward.setGains(gains.kS(), gains.kV(), gains.kA());
	}

	@Override
	public String getName() {
		return name;
	}
}
