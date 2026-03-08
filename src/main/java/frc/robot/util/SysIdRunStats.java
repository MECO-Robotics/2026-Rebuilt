package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;
import java.util.function.DoubleSupplier;

/**
 * Collects summary stats for a single SysId run without publishing live
 * telemetry.
 */
public class SysIdRunStats {
	private final DoubleSupplier positionSupplier;
	private final DoubleSupplier velocitySupplier;
	private final double[] appliedVoltsRef;

	private double startTimeSec;
	private double endTimeSec;
	private double startPosition;
	private double endPosition;
	private double startVelocity;
	private double endVelocity;
	private double maxAbsAppliedVolts;
	private double peakAbsVelocity;

	public SysIdRunStats(DoubleSupplier positionSupplier, DoubleSupplier velocitySupplier, double[] appliedVoltsRef) {
		this.positionSupplier = positionSupplier;
		this.velocitySupplier = velocitySupplier;
		this.appliedVoltsRef = appliedVoltsRef;
	}

	public void start() {
		startTimeSec = Timer.getFPGATimestamp();
		startPosition = positionSupplier.getAsDouble();
		startVelocity = velocitySupplier.getAsDouble();
		endPosition = startPosition;
		endVelocity = startVelocity;
		maxAbsAppliedVolts = Math.abs(appliedVoltsRef[0]);
		peakAbsVelocity = Math.abs(startVelocity);
	}

	public void sample() {
		endPosition = positionSupplier.getAsDouble();
		endVelocity = velocitySupplier.getAsDouble();
		maxAbsAppliedVolts = Math.max(maxAbsAppliedVolts, Math.abs(appliedVoltsRef[0]));
		peakAbsVelocity = Math.max(peakAbsVelocity, Math.abs(endVelocity));
	}

	public void finish() {
		endTimeSec = Timer.getFPGATimestamp();
	}

	public double getDurationSec() {
		return endTimeSec - startTimeSec;
	}

	public double getStartPosition() {
		return startPosition;
	}

	public double getEndPosition() {
		return endPosition;
	}

	public double getDeltaPosition() {
		return endPosition - startPosition;
	}

	public double getStartVelocity() {
		return startVelocity;
	}

	public double getEndVelocity() {
		return endVelocity;
	}

	public double getMaxAbsAppliedVolts() {
		return maxAbsAppliedVolts;
	}

	public double getPeakAbsVelocity() {
		return peakAbsVelocity;
	}
}
