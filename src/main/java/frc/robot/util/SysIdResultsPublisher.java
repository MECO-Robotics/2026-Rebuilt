package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayDeque;
import java.util.Deque;

/** Publishes consolidated SysId run summaries to NetworkTables. */
public class SysIdResultsPublisher {
	private static final String RESULTS_KEY = "SysId/Results";
	private static final int MAX_RESULTS = 20;
	private static final Deque<String> recentResults = new ArrayDeque<>();

	private SysIdResultsPublisher() {
	}

	public static synchronized void publish(String testName, boolean interrupted, SysIdRunStats stats) {
		String entry = String.format(
				"{\"timestampSec\":%.3f,\"test\":\"%s\",\"interrupted\":%s,\"durationSec\":%.3f,\"maxAbsAppliedVolts\":%.3f,\"startPosition\":%.6f,\"endPosition\":%.6f,\"deltaPosition\":%.6f,\"startVelocity\":%.6f,\"endVelocity\":%.6f,\"peakAbsVelocity\":%.6f}",
				Timer.getFPGATimestamp(), escape(testName), interrupted ? "true" : "false", stats.getDurationSec(),
				stats.getMaxAbsAppliedVolts(), stats.getStartPosition(), stats.getEndPosition(),
				stats.getDeltaPosition(), stats.getStartVelocity(), stats.getEndVelocity(), stats.getPeakAbsVelocity());

		recentResults.addLast(entry);
		while (recentResults.size() > MAX_RESULTS) {
			recentResults.removeFirst();
		}

		SmartDashboard.putString(RESULTS_KEY, "[" + String.join(",", recentResults) + "]");
		SmartDashboard.putString("SysId/LastResult", entry);
	}

	private static String escape(String value) {
		return value.replace("\\", "\\\\").replace("\"", "\\\"");
	}
}
