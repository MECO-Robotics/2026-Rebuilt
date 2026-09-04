package org.mecorobotics.gamepiecevision;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedInteger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;

/**
 * Hardware-independent reader for normalized game-piece drive requests.
 *
 * <p>This class never controls motors. The caller must map {@link DriveRequest}
 * to its drivetrain and must retain command requirements for that drivetrain.
 */
public final class GamePieceVisionClient implements AutoCloseable {
  public static final int SCHEMA_VERSION = 1;
  private static final long DEFAULT_MAX_AGE_MICROSECONDS = 250_000;

  private final long maxAgeMicroseconds;
  private final IntegerSubscriber schemaVersion;
  private final BooleanSubscriber connected;
  private final BooleanSubscriber active;
  private final BooleanSubscriber atGoal;
  private final DoubleSubscriber forward;
  private final DoubleSubscriber strafe;
  private final DoubleSubscriber turn;
  private final IntegerSubscriber frameSequence;

  public GamePieceVisionClient(String camera) {
    this(NetworkTableInstance.getDefault(), camera, DEFAULT_MAX_AGE_MICROSECONDS);
  }

  public GamePieceVisionClient(
      NetworkTableInstance instance, String camera, long maxAgeMicroseconds) {
    this.maxAgeMicroseconds = maxAgeMicroseconds;
    NetworkTable cameraTable =
        instance.getTable("GamePieceVision/v1").getSubTable(camera);
    NetworkTable driveRequest = cameraTable.getSubTable("driveRequest");

    schemaVersion = cameraTable.getIntegerTopic("schemaVersion").subscribe(-1);
    connected = cameraTable.getBooleanTopic("connected").subscribe(false);
    active = driveRequest.getBooleanTopic("active").subscribe(false);
    atGoal = driveRequest.getBooleanTopic("atGoal").subscribe(false);
    forward = driveRequest.getDoubleTopic("forward").subscribe(0.0);
    strafe = driveRequest.getDoubleTopic("strafe").subscribe(0.0);
    turn = driveRequest.getDoubleTopic("turn").subscribe(0.0);
    frameSequence =
        driveRequest.getIntegerTopic("frameSequence").subscribe(-1);
  }

  /**
   * Returns a request only while all local and remote safety gates are valid.
   *
   * @param driverAllows true only while the driver's dead-man/assist control is held
   * @param manualOverride true when a driver stick or dedicated kill control requests manual driving
   */
  public DriveRequest getRequest(boolean driverAllows, boolean manualOverride) {
    TimestampedInteger sequenceValue = frameSequence.getAtomic();
    long ageMicroseconds =
        RobotController.getFPGATime() - sequenceValue.timestamp;
    boolean fresh =
        sequenceValue.timestamp != 0
            && ageMicroseconds >= 0
            && ageMicroseconds <= maxAgeMicroseconds;

    boolean allowed =
        DriverStation.isEnabled()
            && driverAllows
            && !manualOverride
            && fresh
            && connected.get()
            && schemaVersion.get() == SCHEMA_VERSION
            && active.get();

    if (!allowed) {
      return DriveRequest.inactive();
    }

    return new DriveRequest(
        true,
        atGoal.get(),
        clampNormalized(forward.get()),
        clampNormalized(strafe.get()),
        clampNormalized(turn.get()),
        sequenceValue.value);
  }

  @Override
  public void close() {
    schemaVersion.close();
    connected.close();
    active.close();
    atGoal.close();
    forward.close();
    strafe.close();
    turn.close();
    frameSequence.close();
  }

  private static double clampNormalized(double value) {
    if (!Double.isFinite(value)) {
      return 0.0;
    }
    return Math.max(-1.0, Math.min(1.0, value));
  }

  /** Normalized robot-relative chassis request. */
  public record DriveRequest(
      boolean active,
      boolean atGoal,
      double forward,
      double strafe,
      double turn,
      long frameSequence) {
    public static DriveRequest inactive() {
      return new DriveRequest(false, false, 0.0, 0.0, 0.0, -1);
    }
  }
}
