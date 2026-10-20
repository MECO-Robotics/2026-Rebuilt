package frc.robot.navigation;

import edu.wpi.first.math.util.Units;

/**
 * Team measurements, assuming the bumper rectangle is centered on the
 * drivetrain origin.
 */
public final class RobotNavigationFootprint {
	public static final double BUMPER_LENGTH_METERS = Units.inchesToMeters(34.033);
	public static final double BUMPER_WIDTH_METERS = Units.inchesToMeters(33.5);
	public static final double DEPLOYED_LENGTH_METERS = Units.inchesToMeters(42.141);
	public static final double CLEARANCE_MARGIN_METERS = .05;
	// The intake extends only forward, so half of the total deployed length would
	// underestimate the distance from the drivetrain origin to the intake tip.
	public static final double FRONT_EXTENT_METERS = DEPLOYED_LENGTH_METERS - BUMPER_LENGTH_METERS / 2;
	public static final double DEPLOYED_RADIUS_METERS = Math.hypot(FRONT_EXTENT_METERS, BUMPER_WIDTH_METERS / 2)
			+ CLEARANCE_MARGIN_METERS;

	private RobotNavigationFootprint() {
	}
}
