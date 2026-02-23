package frc.robot.subsystems.position_joint;

/**
 * Replay-mode stub for {@link PositionJointIO}.
 *
 * <p>This implementation does not command hardware and relies on logged inputs.
 */
public class PositionJointIOReplay implements PositionJointIO {
  private final String name;

  /** Creates a replay IO shim with a stable subsystem name for logging. */
  public PositionJointIOReplay(String name) {
    this.name = name;
  }

  /** Returns this joint's loggable subsystem name. */
  @Override
  public String getName() {
    return name;
  }
}
