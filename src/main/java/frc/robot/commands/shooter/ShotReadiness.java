package frc.robot.commands.shooter;

/**
 * The highest-priority reason a coordinated hub shot is not currently feeding.
 */
public enum ShotReadiness {
	POSE_NOT_READY, OUT_OF_RANGE, MOVING_TOO_FAST, AIMING, HOOD_MOVING, FLYWHEEL_SPINNING_UP, READY, FORCE_FEED_OVERRIDE
}
