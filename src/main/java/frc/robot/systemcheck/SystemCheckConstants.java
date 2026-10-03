package frc.robot.systemcheck;

/** Timing, motion, and safety limits for the automated robot checks. */
public final class SystemCheckConstants {
	public static final int REPORT_WEB_PORT = 5805;
	public static final double HEALTH_POLL_PERIOD_SECONDS = 0.5;
	public static final double COUNTDOWN_SECONDS = 3.0;
	public static final double PREFLIGHT_SECONDS = 3.0;
	public static final double SWERVE_STAGE_SECONDS = 4.0;
	public static final double JOINT_STAGE_SECONDS = 5.0;
	public static final double ROLLER_STAGE_SECONDS = 3.0;
	public static final double HOOD_STAGE_SECONDS = 4.0;
	public static final double SHOOTER_STAGE_SECONDS = 5.0;
	public static final double VISION_STAGE_SECONDS = 5.0;
	public static final double NEUTRAL_STAGE_SECONDS = 3.0;

	public static final double MINIMUM_START_BATTERY_VOLTS = 11.5;
	public static final double ABORT_BATTERY_VOLTS = 9.5;
	public static final double TEMPERATURE_WARNING_CELSIUS = 70.0;
	public static final double TEMPERATURE_ABORT_CELSIUS = 85.0;
	public static final double CAN_UTILIZATION_WARNING = 0.80;
	public static final double STALL_CURRENT_FRACTION = 0.90;
	public static final double STALL_DEBOUNCE_SECONDS = 0.25;
	public static final double STALL_MINIMUM_NORMALIZED_VELOCITY = 0.01;
	public static final double MINIMUM_ROLLER_SPEED_RPS = 1.0;
	public static final double FOLLOWER_SPEED_MISMATCH_FRACTION = 0.25;

	public static final double ROLLER_TEST_VOLTS = 2.0;
	public static final double SHOOTER_TEST_RPS = 10.0;
	public static final double SHOOTER_TOLERANCE_RPS = 2.0;
	public static final double HOOD_TEST_ROTATIONS = 0.020;
	public static final double HOOD_TOLERANCE_ROTATIONS = 0.003;
	public static final double RACK_TOLERANCE_METERS = 0.02;
	public static final double SIM_RACK_WARNING_TOLERANCE_METERS = 0.03;
	public static final double SWERVE_TEST_METERS_PER_SECOND = 0.5;
	public static final double SWERVE_ANGLE_TOLERANCE_DEGREES = 5.0;
	public static final double SWERVE_VELOCITY_TOLERANCE_METERS_PER_SECOND = 0.15;
	public static final double SWERVE_DRIVE_CURRENT_LIMIT_AMPS = 55.0;
	public static final double SWERVE_STEER_CURRENT_LIMIT_AMPS = 30.0;

	public static final double PRACTICE_READY_SECONDS = 5.0;
	public static final double PRACTICE_FEED_SECONDS = 3.0;
	public static final double PRACTICE_TOTAL_SECONDS = 8.0;

	private SystemCheckConstants() {
	}
}
