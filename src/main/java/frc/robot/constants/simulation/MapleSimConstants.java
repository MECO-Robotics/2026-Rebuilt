package frc.robot.constants.simulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/** Constants used by Maple projectile simulation integration. */
public final class MapleSimConstants {
	private MapleSimConstants() {
	}

	/** Shooter location relative to robot center in robot coordinates. */
	public static final Translation2d SHOOTER_TRANSLATION_ON_ROBOT = new Translation2d(-0.19, 0.0);

	/** Shooter yaw offset relative to robot heading. */
	public static final Rotation2d SHOOTER_YAW_OFFSET = Rotation2d.kZero;

	/** Shooter release height from floor. */
	public static final double SHOOTER_HEIGHT_METERS = 0.45;

	/** Conversion from flywheel RPS to projectile speed in m/s. */
	public static final double MPS_PER_FLYWHEEL_RPS = 4.0 * Math.PI * 0.0254; // 4" diameter wheel

	/** Shooter wheel radius used for flywheel-to-ball speed transfer math. */
	public static final double SHOOTER_WHEEL_RADIUS_METERS = Units.inchesToMeters(2.0);

	/**
	 * Counter-wheel linear speed divided by main shooter-wheel linear speed.
	 * <p>
	 * A value below 1.0 models backspin by reducing net energy transfer to the
	 * ball.
	 */
	public static final double COUNTER_TO_MAIN_SHOOTER_WHEEL_SPEED_RATIO = (48.0 / 54.0) * (1.5 / 4.0);

	/** Simulated fuel mass for flywheel slowdown calculations. */
	public static final double FUEL_MASS_KG = 0.5 * 0.45359237;

	/** Enables flywheel MOI-based speed reduction during simulated launches. */
	public static final boolean ENABLE_FLYWHEEL_MOI_SIMULATION = false;

	/** Shooter system inertia used to model flywheel speed drop during launch. */
	public static final double SHOOTER_SYSTEM_MOI_KG_METERS_SQUARED = 24.0 * 0.45359237
			* Math.pow(Units.inchesToMeters(1.0), 2);

	/** Minimum flywheel speed before a shot can launch in simulation. */
	public static final double MIN_FLYWHEEL_RPS_FOR_SHOT = 5.0;

	/** Minimum time between simulated burst starts while feeding. */
	public static final double SHOT_COOLDOWN_SECONDS = 0.20;

	/** Number of fuel pieces launched per simulated firing event. */
	public static final int FUEL_PER_SHOT = 4;

	/** Maximum number of fuel gamepieces the simulated hopper can hold. */
	public static final int HOPPER_MAX_BALLS = 54;

	/** Back-to-front hopper limits in robot X coordinates. */
	public static final double HOPPER_BACK_LIMIT_X_METERS = 0;

	public static final double HOPPER_FRONT_LIMIT_X_METERS = 0.33;

	/** Right-to-left hopper limits in robot Y coordinates. */
	public static final double HOPPER_RIGHT_LIMIT_Y_METERS = -0.34;

	public static final double HOPPER_LEFT_LIMIT_Y_METERS = 0.34;

	/** Bottom-to-top hopper limits in robot Z coordinates. */
	public static final double HOPPER_BOTTOM_LIMIT_Z_METERS = 0.18;

	public static final double HOPPER_TOP_LIMIT_Z_METERS = 0.65;

	/** Ball diameter used for hopper sphere packing. */
	public static final double HOPPER_BALL_DIAMETER_METERS = Units.inchesToMeters(5.5);

	/**
	 * Intake rack angle used to project extension onto robot X for hopper
	 * front-limit growth.
	 */
	public static final double INTAKE_ANGLE_RADIANS = Math.toRadians(7.5);

	/** Lateral spacing (robot Y axis) between fuel pieces in a burst launch. */
	public static final double FUEL_BURST_LATERAL_SPACING_METERS = 0.08;

	/** Fractional randomization applied to average intra-burst stagger interval. */
	public static final double SHOT_STAGGER_RANDOMNESS_RATIO = 0.30;

	/** Floor for intra-burst stagger delay to avoid same-tick clumping. */
	public static final double MIN_SHOT_STAGGER_SECONDS = 0.01;

	/** Max random launch-angle offset (radians) applied to nominal launch angle. */
	public static final double SHOT_ANGLE_RANDOMNESS_RADIANS = Units.degreesToRadians(1.0);

	/**
	 * Random projectile-speed variation as a fraction of nominal speed (e.g. 0.01 =
	 * ±1%).
	 */
	public static final double SHOT_VELOCITY_RANDOMNESS_RATIO = 0.01;

	/**
	 * Offset behind the alliance hub where scored fuel is respawned on the field.
	 */
	public static final double HUB_BACK_SPAWN_X_OFFSET_METERS = 0.65;

	/** Initial speed for fuel respawned from the back of the hub. */
	public static final double HUB_BACK_SPAWN_SPEED_MPS = 3;

	/** Max random heading offset (degrees) applied to back-hub respawn velocity. */
	public static final double HUB_BACK_SPAWN_DIRECTION_RNG_DEG = 45.0;

	/**
	 * Offset applied to hood angle (radians) to align mechanism zero with shot
	 * pitch.
	 */
	public static final double HOOD_ANGLE_OFFSET_RADIANS = Math.toRadians(21.0);

	/** Physical clamp range for launched pitch angle. */
	public static final double MIN_LAUNCH_ANGLE_RADIANS = HOOD_ANGLE_OFFSET_RADIANS - Math.toRadians(1.0);

	public static final double MAX_LAUNCH_ANGLE_RADIANS = HOOD_ANGLE_OFFSET_RADIANS + Math.toRadians(28.0 + 1);
}
