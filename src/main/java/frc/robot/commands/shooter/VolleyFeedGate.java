package frc.robot.commands.shooter;

import edu.wpi.first.math.filter.Debouncer;
import frc.robot.constants.subsystems.ShooterConstants;

/** Waits for speed once per bumper hold, then keeps feeding until release. */
final class VolleyFeedGate {
	private final Debouncer atSpeed = new Debouncer(ShooterConstants.READY_DEBOUNCE_SECONDS);
	private boolean feeding;

	boolean calculate(boolean held, double measuredRps, double targetRps) {
		if (!held) {
			reset();
			return false;
		}
		if (!feeding) {
			feeding = atSpeed.calculate(Double.isFinite(measuredRps) && Double.isFinite(targetRps) && targetRps > 0
					&& Math.abs(measuredRps - targetRps) <= ShooterConstants.FLYWHEEL_READY_TOLERANCE_RPS);
		}
		return feeding;
	}

	void reset() {
		feeding = false;
		atSpeed.calculate(false);
	}
}
