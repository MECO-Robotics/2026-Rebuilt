package frc.robot.subsystems.flywheel;

/**
 * Replay-mode stub for {@link FlywheelIO}.
 *
 * <p>This implementation does not command hardware and relies on logged inputs.
 */
public class FlywheelIOReplay implements FlywheelIO {
  private final String name;

  /** Creates a replay IO shim with a stable subsystem name for logging. */
  public FlywheelIOReplay(String name) {
    this.name = name;
  }

  /** Returns this flywheel's loggable subsystem name. */
  @Override
  public String getName() {
    return name;
  }
}
