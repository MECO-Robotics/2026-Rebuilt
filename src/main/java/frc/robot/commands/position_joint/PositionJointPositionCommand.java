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

	/**
	 * Creates a position setpoint command for a position joint.
	 *
	 * @param positionJoint
	 *            target subsystem
	 * @param position
	 *            supplier for desired position
	 */
	public PositionJointPositionCommand(PositionJoint positionJoint, DoubleSupplier position) {
		this(positionJoint, position, null);
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
		this.positionJoint = positionJoint;
		this.position = position;
		this.maxVelocity = maxVelocity;

		addRequirements(positionJoint);
	}

	@Override
	public void initialize() {
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
