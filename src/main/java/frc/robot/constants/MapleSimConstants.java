package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Constants used by Maple projectile simulation integration. */
public final class MapleSimConstants {
  private MapleSimConstants() {}

  /** Shooter location relative to robot center in robot coordinates. */
  public static final Translation2d SHOOTER_TRANSLATION_ON_ROBOT = new Translation2d(-0.19, 0.0);

  /** Shooter yaw offset relative to robot heading. */
  public static final Rotation2d SHOOTER_YAW_OFFSET = Rotation2d.kZero;

  /** Shooter release height from floor. */
  public static final double SHOOTER_HEIGHT_METERS = 0.45;

  /** Conversion from flywheel RPS to projectile speed in m/s. */
  public static final double MPS_PER_FLYWHEEL_RPS = 4.0 * Math.PI * 0.0254; // 4" diameter wheel

  /** Minimum flywheel speed before a shot can launch in simulation. */
  public static final double MIN_FLYWHEEL_RPS_FOR_SHOT = 20.0;

  /** Minimum time between simulated launches while feeding. */
  public static final double SHOT_COOLDOWN_SECONDS = 0.15;

  /** Offset behind the alliance hub where scored fuel is respawned on the field. */
  public static final double HUB_BACK_SPAWN_X_OFFSET_METERS = 0.65;

  /** Initial speed for fuel respawned from the back of the hub. */
  public static final double HUB_BACK_SPAWN_SPEED_MPS = 3;

  /** Max random heading offset (degrees) applied to back-hub respawn velocity. */
  public static final double HUB_BACK_SPAWN_DIRECTION_RNG_DEG = 45.0;

  /** Offset applied to hood angle (radians) to align mechanism zero with shot pitch. */
  public static final double HOOD_ANGLE_OFFSET_RADIANS = 0.0;

  /** Physical clamp range for launched pitch angle. */
  public static final double MIN_LAUNCH_ANGLE_RADIANS = Math.toRadians(15.0);

  public static final double MAX_LAUNCH_ANGLE_RADIANS = Math.toRadians(80.0);
}
