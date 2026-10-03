package frc.robot.systemcheck;

import java.util.Arrays;

/** Defensive health snapshot for one mechanism or hardware group. */
public record DeviceHealth(String name, boolean[] connections, double[] normalizedVelocities, double[] currentsAmps,
		double[] currentLimitsAmps, double[] temperaturesCelsius, boolean encoderExpected, boolean encoderConnected,
		String[] activeFaults, int[] deviceIds) {
	public DeviceHealth(String name, boolean[] connections, double[] normalizedVelocities, double[] currentsAmps,
			double[] currentLimitsAmps, double[] temperaturesCelsius, boolean encoderExpected, boolean encoderConnected,
			String[] activeFaults) {
		this(name, connections, normalizedVelocities, currentsAmps, currentLimitsAmps, temperaturesCelsius,
				encoderExpected, encoderConnected, activeFaults, new int[0]);
	}

	public DeviceHealth {
		connections = connections.clone();
		normalizedVelocities = normalizedVelocities.clone();
		currentsAmps = currentsAmps.clone();
		currentLimitsAmps = currentLimitsAmps.clone();
		temperaturesCelsius = temperaturesCelsius.clone();
		activeFaults = activeFaults.clone();
		deviceIds = deviceIds.clone();
	}

	@Override
	public boolean[] connections() {
		return connections.clone();
	}

	@Override
	public double[] normalizedVelocities() {
		return normalizedVelocities.clone();
	}

	@Override
	public double[] currentsAmps() {
		return currentsAmps.clone();
	}

	@Override
	public double[] currentLimitsAmps() {
		return currentLimitsAmps.clone();
	}

	@Override
	public double[] temperaturesCelsius() {
		return temperaturesCelsius.clone();
	}

	@Override
	public String[] activeFaults() {
		return activeFaults.clone();
	}

	@Override
	public int[] deviceIds() {
		return deviceIds.clone();
	}

	/** True when every configured connection and required encoder is healthy. */
	public boolean allConnected() {
		for (boolean connected : connections) {
			if (!connected) {
				return false;
			}
		}
		return !encoderExpected || encoderConnected;
	}

	/** True when a vendor reports at least one active fault bit. */
	public boolean hasActiveFaults() {
		return Arrays.stream(activeFaults).anyMatch(fault -> fault != null && !fault.isBlank());
	}

	/** Maximum reported temperature, or zero when unsupported. */
	public double maximumTemperatureCelsius() {
		return Arrays.stream(temperaturesCelsius).filter(Double::isFinite).max().orElse(0.0);
	}

	/** True when the fast motion/current telemetry contains invalid values. */
	public boolean hasNonFiniteFastMeasurement() {
		return Arrays.stream(normalizedVelocities).anyMatch(value -> !Double.isFinite(value))
				|| Arrays.stream(currentsAmps).anyMatch(value -> !Double.isFinite(value))
				|| Arrays.stream(temperaturesCelsius).anyMatch(value -> !Double.isFinite(value));
	}

	/** True when normalized paired speeds agree within the requested fraction. */
	public boolean normalizedSpeedsAgree(double mismatchFraction) {
		if (normalizedVelocities.length < 2) {
			return true;
		}
		double maximum = Arrays.stream(normalizedVelocities).max().orElse(0.0);
		double minimum = Arrays.stream(normalizedVelocities).min().orElse(0.0);
		return maximum < 1e-6 || maximum - minimum <= maximum * mismatchFraction;
	}

	/** Detects a current-limited mechanism that is not moving. */
	public boolean hasStalledMotor(double currentFraction, double minimumVelocity) {
		int count = Math.min(Math.min(currentsAmps.length, currentLimitsAmps.length), normalizedVelocities.length);
		for (int i = 0; i < count; i++) {
			if (Double.isFinite(currentLimitsAmps[i]) && currentLimitsAmps[i] > 0.0
					&& currentsAmps[i] >= currentLimitsAmps[i] * currentFraction
					&& Math.abs(normalizedVelocities[i]) < minimumVelocity) {
				return true;
			}
		}
		return false;
	}

}
