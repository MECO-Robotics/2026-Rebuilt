package frc.robot.navigation;

import frc.robot.util.mechanical_advantage.LoggedTunableNumber;

/**
 * SI units. Provisional conservative settings, snapshotted at command
 * construction.
 */
public record AutoSafetyConfig(double maxSpeed, double maxAcceleration, double maxOmega, double maxAlpha,
		double crossTrackTrigger, double severeError, double lagTrigger, double debounce, double stallSeconds,
		double dwellSeconds, double modeTimeout, double plannerTimeout, double missionTimeout, int maxAttempts,
		double positionTolerance, double headingTolerance, double speedTolerance) {
	public static final double STALL_CLEARANCE_WAIT_SECONDS = .75;
	public static final double DETOUR_MAX_SPEED = 1.0;
	private static final LoggedTunableNumber CROSS_TRACK = new LoggedTunableNumber("Hybrid/Config/CrossTrackMeters",
			.25);
	private static final LoggedTunableNumber SEVERE = new LoggedTunableNumber("Hybrid/Config/SevereMeters", .5);
	private static final LoggedTunableNumber LAG = new LoggedTunableNumber("Hybrid/Config/LagMeters", .65);
	private static final LoggedTunableNumber DEBOUNCE = new LoggedTunableNumber("Hybrid/Config/DebounceSeconds", .2);
	private static final LoggedTunableNumber STALL = new LoggedTunableNumber("Hybrid/Config/StallSeconds", .4);
	/**
	 * Read at autonomous construction, never change safety policy halfway through a
	 * mission.
	 */
	public static AutoSafetyConfig tunedDefaults() {
		var d = defaults();
		return new AutoSafetyConfig(d.maxSpeed, d.maxAcceleration, d.maxOmega, d.maxAlpha, CROSS_TRACK.get(),
				SEVERE.get(), LAG.get(), DEBOUNCE.get(), STALL.get(), d.dwellSeconds, d.modeTimeout, d.plannerTimeout,
				d.missionTimeout, d.maxAttempts, d.positionTolerance, d.headingTolerance, d.speedTolerance);
	}
	public AutoSafetyConfig {
		double[] positive = {maxSpeed, maxAcceleration, maxOmega, maxAlpha, crossTrackTrigger, severeError, lagTrigger,
				debounce, stallSeconds, dwellSeconds, modeTimeout, plannerTimeout, missionTimeout, positionTolerance,
				headingTolerance, speedTolerance};
		for (double value : positive)
			if (!Double.isFinite(value) || value <= 0)
				throw new IllegalArgumentException("Invalid navigation limit");
		if (maxAttempts < 1 || severeError < crossTrackTrigger)
			throw new IllegalArgumentException("Invalid navigation policy");
	}
	public static AutoSafetyConfig defaults() {
		return new AutoSafetyConfig(2.0, 2.0, 3.0, 6.0, .25, .5, .65, .2, .4, .5, 10, 2.0, 30, 4, .10,
				Math.toRadians(4), .15);
	}
}
