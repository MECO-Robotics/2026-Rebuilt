package frc.robot.commands.position_joint;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.position_joint.PositionJoint;
import java.util.function.DoubleSupplier;

/**
 * One-shot command that sets a joint position goal and finishes on tolerance.
 */
public class PositionJointPositionCommand extends Command {
	private final PositionJoint positionJoint;
	private final DoubleSupplier position;
	private final DoubleSupplier maxVelocity;
	private final boolean complianceAfterTarget;

	/**
	 * Creates a position setpoint command for a position joint.
	 *
	 * @param positionJoint
	 *            target subsystem
	 * @param position
	 *            supplier for desired position
	 */
	public PositionJointPositionCommand(PositionJoint positionJoint, DoubleSupplier position) {
		this(positionJoint, position, null, false);
	}

	/**
	 * Creates a position setpoint command for a position joint with a temporary
	 * max-velocity limit.
	 *
	 * @param positionJoint
	 *            target subsystem
	 * @param position
	 *            supplier for desired position
	 * @param maxVelocity
	 *            supplier for temporary max profile velocity
	 */
	public PositionJointPositionCommand(PositionJoint positionJoint, DoubleSupplier position,
			DoubleSupplier maxVelocity) {
		this(positionJoint, position, maxVelocity, false);
	}

	/**
	 * Creates a position setpoint command for a position joint with optional
	 * compliance mode after reaching target.
	 */
	public PositionJointPositionCommand(PositionJoint positionJoint, DoubleSupplier position,
			DoubleSupplier maxVelocity, boolean complianceAfterTarget) {
		this.positionJoint = positionJoint;
		this.position = position;
		this.maxVelocity = maxVelocity;
		this.complianceAfterTarget = complianceAfterTarget;

		addRequirements(positionJoint);
	}

	@Override
	public void initialize() {
		positionJoint.setComplianceAfterTarget(complianceAfterTarget);
		updateSetpoint();
	}

	@Override
	public void execute() {
		updateSetpoint();
	}

	private void updateSetpoint() {
		if (maxVelocity != null) {
			positionJoint.setPosition(position.getAsDouble(), maxVelocity.getAsDouble());
			return;
		}

		positionJoint.setPosition(position.getAsDouble());
	}

	@Override
	public void end(boolean interrupted) {
		if (maxVelocity != null) {
			positionJoint.clearProfileConstraintsOverride();
		}
	}

	@Override
	public boolean isFinished() {
		return positionJoint.isFinished();
	}
}
