package frc.robot.subsystems.position_joint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.position_joint.PositionJointPositionCommand;
import frc.robot.commands.position_joint.PositionJointVelocityCommand;
import frc.robot.constants.types.PositionJointConstants.PositionJointGains;
import frc.robot.util.mechanical_advantage.LoggedTunableNumber;
import frc.robot.systemcheck.DeviceHealth;
import frc.robot.systemcheck.MotorHealthData;
import frc.robot.systemcheck.SystemCheckConstants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem wrapper for a single position-controlled mechanism joint.
 *
 * <p>
 * This class owns motion profiling and tunable gains, while the backing
 * {@link PositionJointIO} handles device-specific hardware control.
 */
public class PositionJoint extends SubsystemBase {
	private final PositionJointIO positionJoint;
	private final PositionJointIOInputsAutoLogged inputs = new PositionJointIOInputsAutoLogged();
	private final MotorHealthData motorHealth = new MotorHealthData();

	private final String name;

	private final LoggedTunableNumber kP;
	private final LoggedTunableNumber kI;
	private final LoggedTunableNumber kD;
	private final LoggedTunableNumber kS;
	private final LoggedTunableNumber kG;
	private final LoggedTunableNumber kV;
	private final LoggedTunableNumber kA;

	private final LoggedTunableNumber kMaxVelo;
	private final LoggedTunableNumber kMaxAccel;

	private final LoggedTunableNumber kMinPosition;
	private final LoggedTunableNumber kMaxPosition;

	private final LoggedTunableNumber kTolerance;

	private final LoggedTunableNumber kSetpoint;
	private Double profileMaxVelocityOverride = null;
	private double requestedPosition;
	private double goalPosition;
	private boolean complianceAfterTarget = false;
	private boolean complianceActive = false;
	private double complianceEnterTolerance;
	private double complianceExitTolerance;
	private boolean openLoopMode = false;
	private double commandedVoltage = 0.0;
	private double lastHealthPollSeconds = Double.NEGATIVE_INFINITY;

	/**
	 * Creates a position-joint subsystem.
	 *
	 * @param io
	 *            hardware implementation for this joint
	 * @param gains
	 *            default gains and profile constraints
	 */
	public PositionJoint(PositionJointIO io, PositionJointGains gains) {
		super(io.getName());

		positionJoint = io;
		name = positionJoint.getName();

		kP = new LoggedTunableNumber(name + "/Gains/kP", gains.kP());
		kI = new LoggedTunableNumber(name + "/Gains/kI", gains.kI());
		kD = new LoggedTunableNumber(name + "/Gains/kD", gains.kD());
		kS = new LoggedTunableNumber(name + "/Gains/kS", gains.kS());
		kG = new LoggedTunableNumber(name + "/Gains/kG", gains.kG());
		kV = new LoggedTunableNumber(name + "/Gains/kV", gains.kV());
		kA = new LoggedTunableNumber(name + "/Gains/kA", gains.kA());

		kMaxVelo = new LoggedTunableNumber(name + "/Gains/kMaxVelo", gains.kMaxVelo());
		kMaxAccel = new LoggedTunableNumber(name + "/Gains/kMaxAccel", gains.kMaxAccel());

		kMinPosition = new LoggedTunableNumber(name + "/Gains/kMinPosition", gains.kMinPosition());
		kMaxPosition = new LoggedTunableNumber(name + "/Gains/kMaxPosition", gains.kMaxPosition());

		kTolerance = new LoggedTunableNumber(name + "/Gains/kTolerance", gains.kTolerance());

		kSetpoint = new LoggedTunableNumber(name + "/Gains/kSetpoint", gains.kDefaultSetpoint());
		goalPosition = gains.kDefaultSetpoint();
		complianceEnterTolerance = gains.kTolerance();
		complianceExitTolerance = gains.kTolerance();

		// Load the configured gains immediately so sim IO PID/FF are initialized at
		// startup.
		positionJoint.setGains(gains);

		SmartDashboard.putData(name, this);
	}

	@Override
	public void periodic() {
		positionJoint.updateInputs(inputs);
		Logger.processInputs(name, inputs);
		double nowSeconds = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
		if (nowSeconds - lastHealthPollSeconds >= SystemCheckConstants.HEALTH_POLL_PERIOD_SECONDS) {
			positionJoint.updateHealth(motorHealth);
			lastHealthPollSeconds = nowSeconds;
		}

		double positionError = Math.abs(inputs.outputPosition - goalPosition);
		boolean atTarget = positionError < kTolerance.get();
		boolean withinComplianceBand = complianceActive
				? positionError <= complianceExitTolerance
				: positionError <= complianceEnterTolerance;
		if (openLoopMode) {
			positionJoint.setVoltage(commandedVoltage);
		} else if (complianceAfterTarget && withinComplianceBand) {
			if (!complianceActive) {
				positionJoint.setBrakeMode(false);
				positionJoint.setVoltage(0.0);
				complianceActive = true;
			}
		} else {
			disableComplianceHold();
			boolean usingDynamicOverride = profileMaxVelocityOverride != null
					&& positionJoint.setPositionDynamic(goalPosition, profileMaxVelocityOverride, kMaxAccel.get());
			if (!usingDynamicOverride) {
				positionJoint.setPosition(goalPosition, 0.0);
			}
		}

		LoggedTunableNumber.ifChanged(hashCode(), (values) -> {
			positionJoint.setGains(new PositionJointGains(values[0], values[1], values[2], values[3], values[4],
					values[5], values[6], values[7], values[8], values[9], values[10], values[11], values[12]));
		}, kP, kI, kD, kS, kG, kV, kA, kMaxVelo, kMaxAccel, kMinPosition, kMaxPosition, kTolerance, kSetpoint);
		LoggedTunableNumber.ifChanged(hashCode() + 1, (values) -> {
			Command currentCommand = getCurrentCommand();
			if (currentCommand == null || currentCommand == getDefaultCommand()) {
				goalPosition = MathUtil.clamp(values[0], values[1], values[2]);
			}
		}, kSetpoint, kMinPosition, kMaxPosition);

		Logger.recordOutput(name + "/RequestedPosition", requestedPosition);
		Logger.recordOutput(name + "/ControlMode",
				openLoopMode ? "VOLTAGE" : complianceActive ? "COMPLIANCE" : "POSITION");
		Logger.recordOutput(name + "/CommandOwner",
				getCurrentCommand() == null ? "None" : getCurrentCommand().getName());
		Logger.recordOutput(name + "/ControllerDiagnostic", controllerDiagnosticFailure());
		Logger.recordOutput(name + "/ConfiguredDeviceIds", positionJoint.getDeviceIds());
		Logger.recordOutput(name + "/GoalPosition", goalPosition);
		Logger.recordOutput(name + "/isFinished", atTarget);
		Logger.recordOutput(name + "/ComplianceAfterTarget", complianceAfterTarget);
		Logger.recordOutput(name + "/ComplianceActive", complianceActive);
		Logger.recordOutput(name + "/OpenLoopMode", openLoopMode);
		Logger.recordOutput(name + "/CommandedVoltage", commandedVoltage);
		Logger.recordOutput(name + "/Health/TemperaturesCelsius", motorHealth.temperaturesCelsius);
		Logger.recordOutput(name + "/Health/ActiveFaults", motorHealth.activeFaults);
	}

	/** Sets a new goal position, clamped to configured mechanism limits. */
	public void setPosition(double position) {
		requestedPosition = position;
		openLoopMode = false;
		commandedVoltage = 0.0;
		double clampedPosition = MathUtil.clamp(position, kMinPosition.get(), kMaxPosition.get());
		if (Math.abs(clampedPosition - goalPosition) > 1e-9) {
			disableComplianceHold();
		}
		goalPosition = clampedPosition;
	}

	/** Sets a new goal position with a temporary max-velocity override. */
	public void setPosition(double position, double maxVelocity) {
		profileMaxVelocityOverride = Math.max(0.0, maxVelocity);
		setPosition(position);
	}

	/**
	 * Clears any temporary profile constraint override and restores tunable
	 * defaults.
	 */
	public void clearProfileConstraintsOverride() {
		if (profileMaxVelocityOverride == null) {
			return;
		}

		profileMaxVelocityOverride = null;
	}

	/** Enables/disables post-target compliance mode for impact absorption. */
	public void setComplianceAfterTarget(boolean enabled) {
		complianceAfterTarget = enabled;
		if (!enabled) {
			disableComplianceHold();
		}
	}

	/** Sets the enter/exit error bands used by compliant post-target control. */
	public void setComplianceThresholds(double enterTolerance, double exitTolerance) {
		complianceEnterTolerance = Math.max(0.0, enterTolerance);
		complianceExitTolerance = Math.max(complianceEnterTolerance, exitTolerance);
	}

	/** Adds an offset to the current goal position. */
	public void incrementPosition(double deltaPosition) {
		setPosition(goalPosition + deltaPosition);
	}

	/** Re-seeds the goal from the current measured mechanism position. */
	public void syncGoalToCurrentPosition() {
		setPosition(inputs.outputPosition);
	}

	/** Applies open-loop voltage to the joint leader motor. */
	public void setVoltage(double voltage) {
		openLoopMode = true;
		commandedVoltage = voltage;
		disableComplianceHold();
		positionJoint.setVoltage(voltage);
	}

	/** Leaves the joint in persistent open-loop neutral until a new goal is set. */
	public void stop() {
		setVoltage(0.0);
	}

	/** Returns true while direct-voltage control is active. */
	public boolean isOpenLoopMode() {
		return openLoopMode;
	}

	/** Returns the direct voltage most recently requested for the joint. */
	public double getCommandedVoltage() {
		return commandedVoltage;
	}

	/** Returns a defensive snapshot of fast telemetry and low-rate diagnostics. */
	public DeviceHealth getHealthSnapshot() {
		double[] normalizedVelocities = new double[inputs.motorVelocities.length];
		for (int i = 0; i < normalizedVelocities.length; i++) {
			normalizedVelocities[i] = Math.abs(inputs.motorVelocities[i]);
		}
		double[] currentLimits = new double[inputs.motorCurrents.length];
		java.util.Arrays.fill(currentLimits, positionJoint.getCurrentLimitAmps());
		return new DeviceHealth(name, inputs.motorsConnected, normalizedVelocities, inputs.motorCurrents, currentLimits,
				motorHealth.temperaturesCelsius, positionJoint.isExternalEncoderExpected(), inputs.encoderConnected,
				motorHealth.activeFaults, positionJoint.getDeviceIds());
	}

	/**
	 * Vendor diagnostics are separate from sensor connectivity and are not CAN
	 * freshness proof.
	 */
	public String controllerDiagnosticFailure() {
		if (!inputs.controllerDiagnosticsSupported)
			return "";
		if (!inputs.configurationHealthy)
			return "Controller configuration failed: " + String.join(", ", inputs.configurationStatus);
		if (!inputs.controlRequestHealthy)
			return "Controller request failed: " + inputs.controlStatus;
		return "";
	}

	public java.util.Map<String, Double> diagnosticMeasurements() {
		var values = new java.util.LinkedHashMap<String, Double>();
		values.put("RequestedPosition", requestedPosition);
		values.put("ClampedTarget", goalPosition);
		values.put("OpenLoopMode", openLoopMode ? 1.0 : 0.0);
		values.put("OpenLoopRequestedVolts", commandedVoltage);
		values.put("MaxAppliedVolts", java.util.Arrays.stream(inputs.motorVoltages).map(Math::abs).max().orElse(0));
		values.put("MaxCurrentAmps", java.util.Arrays.stream(inputs.motorCurrents).max().orElse(0));
		if (inputs.controllerDiagnosticsSupported) {
			values.put("ConfigurationAccepted", inputs.configurationHealthy ? 1.0 : 0.0);
			values.put("ControlRequestAccepted", inputs.controlRequestHealthy ? 1.0 : 0.0);
			values.put("AtReverseLimit", inputs.atReverseLimit ? 1.0 : 0.0);
			values.put("AtForwardLimit", inputs.atForwardLimit ? 1.0 : 0.0);
			values.put("MinPosition", inputs.configuredMinPosition);
			values.put("MaxPosition", inputs.configuredMaxPosition);
			values.put("ProfileMaxVelocity", inputs.configuredMaxVelocity);
			values.put("ProfileMaxAcceleration", inputs.configuredMaxAcceleration);
			values.put("PositionConversionFactor", inputs.positionConversionFactor);
			values.put("VelocityConversionFactor", inputs.velocityConversionFactor);
		}
		return values;
	}

	/** Returns current measured mechanism position. */
	public double getPosition() {
		return inputs.outputPosition;
	}

	/** Returns current measured mechanism velocity. */
	public double getVelocity() {
		return inputs.velocity;
	}

	/** Returns the last requested position setpoint reported by the IO layer. */
	public double getDesiredPosition() {
		return inputs.desiredPosition;
	}

	/** Returns true when measured position is within tolerance of the goal. */
	public boolean isFinished() {
		return Math.abs(inputs.outputPosition - goalPosition) < kTolerance.get();
	}

	/**
	 * Returns true when measured position is within a caller-provided tolerance.
	 */
	public boolean atPosition(double position, double tolerance) {
		return Math.abs(inputs.outputPosition - position) <= tolerance;
	}

	/** Resets sensor position and clears the active goal to zero. */
	public void resetPosition() {
		positionJoint.resetPosition();
		goalPosition = 0;
		disableComplianceHold();
	}

	/** Builds a command that continuously sets position from a supplier. */
	public static Command setPosition(PositionJoint positionJoint, DoubleSupplier positionSupplier) {
		return new PositionJointPositionCommand(positionJoint, positionSupplier);
	}

	/** Holds a supplied position until the command is interrupted. */
	public static Command holdPosition(PositionJoint positionJoint, DoubleSupplier positionSupplier) {
		return Commands.run(() -> positionJoint.setPosition(positionSupplier.getAsDouble()), positionJoint)
				.beforeStarting(() -> positionJoint.setComplianceAfterTarget(false));
	}

	/** Holds a supplied position with optional post-target compliance. */
	public static Command holdPosition(PositionJoint positionJoint, DoubleSupplier positionSupplier,
			boolean complianceAfterTarget) {
		return Commands.run(() -> positionJoint.setPosition(positionSupplier.getAsDouble()), positionJoint)
				.beforeStarting(() -> positionJoint.setComplianceAfterTarget(complianceAfterTarget));
	}

	/** Builds a command that sets position using a temporary max-velocity limit. */
	public static Command setPosition(PositionJoint positionJoint, DoubleSupplier positionSupplier,
			DoubleSupplier maxVelocitySupplier) {
		return new PositionJointPositionCommand(positionJoint, positionSupplier, maxVelocitySupplier);
	}

	/**
	 * Builds a command that sets position with a temporary max-velocity limit and
	 * optional compliance mode after reaching target.
	 */
	public static Command setPosition(PositionJoint positionJoint, DoubleSupplier positionSupplier,
			DoubleSupplier maxVelocitySupplier, boolean complianceAfterTarget) {
		return new PositionJointPositionCommand(positionJoint, positionSupplier, maxVelocitySupplier,
				complianceAfterTarget);
	}

	/**
	 * Builds a command that optionally enters compliance once target tolerance is
	 * reached.
	 */
	public static Command setPosition(PositionJoint positionJoint, DoubleSupplier positionSupplier,
			boolean complianceAfterTarget) {
		return new PositionJointPositionCommand(positionJoint, positionSupplier, null, complianceAfterTarget);
	}

	/** Builds a command that continuously sets velocity from a supplier. */
	public static Command setVelocity(PositionJoint positionJoint, DoubleSupplier velocitySupplier) {
		return new PositionJointVelocityCommand(positionJoint, velocitySupplier);
	}

	private void disableComplianceHold() {
		if (complianceActive) {
			positionJoint.setBrakeMode(true);
			complianceActive = false;
		}
	}
}
