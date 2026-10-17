package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Preferences;

/** Persistent, bounded rack settings sampled once per driver feeding volley. */
public record ShotIntakeSettings(double retractSpeedMetersPerSecond, double holdPositionMeters) {
	public static final String SPEED_KEY = "Intake/ShotRetractSpeedMps";
	public static final String POSITION_KEY = "Intake/ShotHoldPositionMeters";
	public static final double DEFAULT_SPEED = 0.10;
	public static final double DEFAULT_POSITION = 0.13;

	public static void initialize() {
		Preferences.initDouble(SPEED_KEY, DEFAULT_SPEED);
		Preferences.initDouble(POSITION_KEY, DEFAULT_POSITION);
	}

	public static ShotIntakeSettings load() {
		return new ShotIntakeSettings(Preferences.getDouble(SPEED_KEY, DEFAULT_SPEED),
				Preferences.getDouble(POSITION_KEY, DEFAULT_POSITION));
	}

	public ShotIntakeSettings {
		retractSpeedMetersPerSecond = Double.isFinite(retractSpeedMetersPerSecond)
				? MathUtil.clamp(retractSpeedMetersPerSecond, 0.01, 0.50)
				: DEFAULT_SPEED;
		// Do not allow dashboard tuning to retract beyond the existing safe position.
		holdPositionMeters = Double.isFinite(holdPositionMeters)
				? MathUtil.clamp(holdPositionMeters, DEFAULT_POSITION, 0.30)
				: DEFAULT_POSITION;
	}
}
