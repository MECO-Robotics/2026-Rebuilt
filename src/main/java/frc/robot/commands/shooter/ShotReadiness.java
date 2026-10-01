package frc.robot.commands.shooter;

/**
 * The highest-priority reason a coordinated shot is not currently feeding.
 */
public enum ShotReadiness {
	POSE_NOT_READY, OUT_OF_RANGE, UNSAFE_FERRY_PATH, MOVING_TOO_FAST, AIMING, HOOD_MOVING, FLYWHEEL_SPINNING_UP, CALIBRATION_MODE, READY, FORCE_FEED_OVERRIDE
}
