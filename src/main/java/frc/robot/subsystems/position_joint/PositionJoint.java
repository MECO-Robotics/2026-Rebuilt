package frc.robot.subsystems.position_joint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.position_joint.PositionJointPositionCommand;
import frc.robot.commands.position_joint.PositionJointVelocityCommand;
import frc.robot.constants.types.PositionJointConstants.PositionJointGains;
import frc.robot.util.mechanical_advantage.LoggedTunableNumber;
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
	private double goalPosition;

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

		// Load the configured gains immediately so sim IO PID/FF are initialized at
		// startup.
		positionJoint.setGains(gains);

		SmartDashboard.putData(name, this);
	}

	@Override
	public void periodic() {
		positionJoint.updateInputs(inputs);
		Logger.processInputs(name, inputs);

		boolean usingDynamicOverride = profileMaxVelocityOverride != null
				&& positionJoint.setPositionDynamic(goalPosition, profileMaxVelocityOverride, kMaxAccel.get());
		if (!usingDynamicOverride) {
			positionJoint.setPosition(goalPosition, 0.0);
		}

		LoggedTunableNumber.ifChanged(hashCode(), (values) -> {
			positionJoint.setGains(new PositionJointGains(values[0], values[1], values[2], values[3], values[4],
					values[5], values[6], values[7], values[8], values[9], values[10], values[11], values[12]));

			goalPosition = MathUtil.clamp(values[12], kMinPosition.get(), kMaxPosition.get());
		}, kP, kI, kD, kS, kG, kV, kA, kMaxVelo, kMaxAccel, kMinPosition, kMaxPosition, kTolerance, kSetpoint);

		Logger.recordOutput(name + "/isFinished", isFinished());
	}

	/** Sets a new goal position, clamped to configured mechanism limits. */
	public void setPosition(double position) {
		goalPosition = MathUtil.clamp(position, kMinPosition.get(), kMaxPosition.get());
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

	/** Adds an offset to the current goal position. */
	public void incrementPosition(double deltaPosition) {
		setPosition(goalPosition + deltaPosition);
	}

	/** Applies open-loop voltage to the joint leader motor. */
	public void setVoltage(double voltage) {
		positionJoint.setVoltage(voltage);
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

	/** Resets sensor position and clears the active goal to zero. */
	public void resetPosition() {
		positionJoint.resetPosition();
		goalPosition = 0;
	}

	/** Builds a command that continuously sets position from a supplier. */
	public static Command setPosition(PositionJoint positionJoint, DoubleSupplier positionSupplier) {
		return new PositionJointPositionCommand(positionJoint, positionSupplier);
	}

	/** Builds a command that sets position using a temporary max-velocity limit. */
	public static Command setPosition(PositionJoint positionJoint, DoubleSupplier positionSupplier,
			DoubleSupplier maxVelocitySupplier) {
		return new PositionJointPositionCommand(positionJoint, positionSupplier, maxVelocitySupplier);
	}

	/** Builds a command that continuously sets velocity from a supplier. */
	public static Command setVelocity(PositionJoint positionJoint, DoubleSupplier velocitySupplier) {
		return new PositionJointVelocityCommand(positionJoint, velocitySupplier);
	}
}
