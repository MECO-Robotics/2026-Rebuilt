package frc.robot.simulation;

/**
 * Approximate wheel energy removed by a projectile, without changing real
 * gains.
 */
public final class FlywheelEnergy {
	private FlywheelEnergy() {
	}

	public static double afterShot(double omega, double inertia, double energyJoules) {
		if (!Double.isFinite(omega) || !Double.isFinite(inertia) || inertia <= 0 || !Double.isFinite(energyJoules)
				|| energyJoules < 0) {
			throw new IllegalArgumentException("Shot energy and inertia must be finite and nonnegative; inertia > 0");
		}
		return Math.copySign(Math.sqrt(Math.max(0.0, omega * omega - 2.0 * energyJoules / inertia)), omega);
	}
}
