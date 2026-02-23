package frc.robot.commands.position_joint;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.position_joint.PositionJoint;
import java.util.function.DoubleSupplier;

/** One-shot command that sets a joint position goal and finishes on tolerance. */
public class PositionJointPositionCommand extends Command {
  private final PositionJoint positionJoint;
  private final DoubleSupplier position;

  /**
   * Creates a position setpoint command for a position joint.
   *
   * @param positionJoint target subsystem
   * @param position supplier for desired position
   */
  public PositionJointPositionCommand(PositionJoint positionJoint, DoubleSupplier position) {
    this.positionJoint = positionJoint;
    this.position = position;

    addRequirements(positionJoint);
  }

  @Override
  public void initialize() {
    positionJoint.setPosition(position.getAsDouble());
  }

  @Override
  public boolean isFinished() {
    return positionJoint.isFinished();
  }
}
