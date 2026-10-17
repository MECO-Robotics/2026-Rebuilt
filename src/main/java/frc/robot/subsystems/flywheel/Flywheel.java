package frc.robot.subsystems.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.flywheel.FlywheelVelocityCommand;
import frc.robot.commands.flywheel.FlywheelVoltageCommand;
import frc.robot.constants.types.FlywheelConstants.FlywheelGains;
import frc.robot.systemcheck.DeviceHealth;
import frc.robot.systemcheck.MotorHealthData;
import frc.robot.systemcheck.SystemCheckConstants;
import frc.robot.util.mechanical_advantage.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem wrapper for a velocity-controlled flywheel/roller.
 *
 * <p>
 * This class owns tunable gains and setpoints while {@link FlywheelIO} handles
 * hardware-specific closed-loop control (Motion Magic/MAX Motion).
 */
public class Flywheel extends SubsystemBase {
	private final FlywheelIO flywheel;
	private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
	private final MotorHealthData motorHealth = new MotorHealthData();

	private final String name;

	private final LoggedTunableNumber kP;
	private final LoggedTunableNumber kI;
	private final LoggedTunableNumber kD;
	private final LoggedTunableNumber kS;
	private final LoggedTunableNumber kV;
	private final LoggedTunableNumber kA;

	private final LoggedTunableNumber kMaxAccel;

	private final LoggedTunableNumber kTolerance;

	private final LoggedTunableNumber kSetpoint;

	private double velocitySetpoint;
	private double voltageSetpoint;

	// Start neutral. Closed-loop zero velocity can actively drive/brake a real
	// mechanism, so an uncommanded flywheel must remain in explicit 0 V mode.
	private boolean voltageMode = true;
	private double lastHealthPollSeconds = Double.NEGATIVE_INFINITY;

	/**
	 * Creates a flywheel subsystem.
	 *
	 * @param io
	 *            hardware implementation for this mechanism
	 * @param gains
	 *            default gains and profile limits
	 */
	public Flywheel(FlywheelIO io, FlywheelGains gains) {
		super(io.getName());

		flywheel = io;

		name = flywheel.getName();

		kP = new LoggedTunableNumber(name + "/Gains/kP", gains.kP());
		kI = new LoggedTunableNumber(name + "/Gains/kI", gains.kI());
		kD = new LoggedTunableNumber(name + "/Gains/kD", gains.kD());
		kS = new LoggedTunableNumber(name + "/Gains/kS", gains.kS());
		kV = new LoggedTunableNumber(name + "/Gains/kV", gains.kV());
		kA = new LoggedTunableNumber(name + "/Gains/kA", gains.kA());

		kMaxAccel = new LoggedTunableNumber(name + "/Gains/kMaxAccel", gains.kMaxAccel());

		kTolerance = new LoggedTunableNumber(name + "/Gains/kTolerance", gains.kTolerance());

		kSetpoint = new LoggedTunableNumber(name + "/Gains/kSetpoint", 0.0);

		// Load the configured gains immediately so sim IO PID/FF are initialized at
		// startup.
		flywheel.setGains(gains);
		flywheel.setVoltage(0.0);
	}

	@Override
	public void periodic() {
		flywheel.updateInputs(inputs);
		Logger.processInputs(name, inputs);
		double nowSeconds = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
		if (nowSeconds - lastHealthPollSeconds >= SystemCheckConstants.HEALTH_POLL_PERIOD_SECONDS) {
			flywheel.updateHealth(motorHealth);
			lastHealthPollSeconds = nowSeconds;
		}
		Logger.recordOutput(name + "/CommandedVoltage", voltageSetpoint);
		Logger.recordOutput(name + "/VelocityControlActive", !voltageMode);
		Logger.recordOutput(name + "/Health/TemperaturesCelsius", motorHealth.temperaturesCelsius);
		Logger.recordOutput(name + "/Health/ActiveFaults", motorHealth.activeFaults);

		Command currentCommand = getCurrentCommand();
		if (currentCommand == null || currentCommand == getDefaultCommand()) {
			velocitySetpoint = kSetpoint.get();
		}

		if (!voltageMode) {
			flywheel.setVelocity(velocitySetpoint);
		}

		LoggedTunableNumber.ifChanged(hashCode(), (values) -> {
			flywheel.setGains(new FlywheelGains(values[0], values[1], values[2], values[3], values[4], values[5],
					values[6], values[7]));

			if (currentCommand == null || currentCommand == getDefaultCommand()) {
				velocitySetpoint = values[8];
			}
		}, kP, kI, kD, kS, kV, kA, kMaxAccel, kTolerance, kSetpoint);
	}

	/** Sets a new velocity goal for closed-loop control. */
	public void setVelocity(double velocity) {
		voltageMode = false;
		voltageSetpoint = 0.0;
		velocitySetpoint = velocity;
	}

	/** Enables open-loop control and applies a direct voltage command. */
	public void setVoltage(double voltage) {
		voltageMode = true;
		voltageSetpoint = voltage;
		velocitySetpoint = 0.0;
		flywheel.setVoltage(voltage);
	}

	/** Leaves the motor controller in open-loop neutral output. */
	public void stop() {
		setVoltage(0.0);
	}

	/** Returns current measured velocity. */
	public double getVelocity() {
		return inputs.velocity;
	}

	/** Returns current measured mechanism position in rotations. */
	public double getPosition() {
		return inputs.position;
	}

	/** Returns the last requested velocity setpoint from the IO layer. */
	public double getVelocitySetpoint() {
		return inputs.desiredVelocity;
	}

	/** Returns the direct voltage most recently requested by the subsystem. */
	public double getCommandedVoltage() {
		return voltageSetpoint;
	}

	/** Returns whether the subsystem is currently using velocity control. */
	public boolean isVelocityControlActive() {
		return !voltageMode;
	}

	/** Returns a defensive snapshot of fast telemetry and low-rate diagnostics. */
	public DeviceHealth getHealthSnapshot() {
		double[] normalizedVelocities = new double[inputs.motorVelocities.length];
		for (int i = 0; i < normalizedVelocities.length; i++) {
			normalizedVelocities[i] = Math.abs(inputs.motorVelocities[i]);
		}
		double[] currentLimits = new double[inputs.motorCurrents.length];
		java.util.Arrays.fill(currentLimits, flywheel.getCurrentLimitAmps());
		return new DeviceHealth(name, inputs.motorsConnected, normalizedVelocities, inputs.motorCurrents, currentLimits,
				motorHealth.temperaturesCelsius, false, true, motorHealth.activeFaults, flywheel.getDeviceIds());
	}

	/**
	 * Returns true when measured velocity is within configured tolerance of
	 * setpoint.
	 */
	public boolean isFinished() {
		return Math.abs(inputs.velocity - inputs.desiredVelocity) < kTolerance.get();
	}

	/**
	 * Returns true when measured velocity is within a caller-provided tolerance.
	 */
	public boolean atVelocity(double velocity, double tolerance) {
		return Math.abs(inputs.velocity - velocity) <= tolerance;
	}

	/**
	 * Builds a command that continuously sets flywheel velocity from a supplier.
	 */
	public static Command setVelocity(Flywheel flywheel, DoubleSupplier velocity) {
		return new FlywheelVelocityCommand(flywheel, velocity);
	}

	/** Holds a supplied velocity until the command is interrupted. */
	public static Command holdVelocity(Flywheel flywheel, DoubleSupplier velocity) {
		return Commands.run(() -> flywheel.setVelocity(velocity.getAsDouble()), flywheel)
				.finallyDo(interrupted -> flywheel.stop());
	}

	/** Builds a command that continuously sets flywheel voltage from a supplier. */
	public static Command setVoltage(Flywheel flywheel, DoubleSupplier voltage) {
		return new FlywheelVoltageCommand(flywheel, voltage);
	}

	/** Continuously holds an explicit 0 V neutral output. */
	public static Command idle(Flywheel flywheel) {
		return Commands.run(flywheel::stop, flywheel).finallyDo(interrupted -> flywheel.stop());
	}
}
