package frc.robot.systemcheck;

import java.util.Map;

/**
 * Immutable result, measurements, and failure diagnosis for one system-check
 * stage.
 */
public record CheckResult(String id, String subsystem, CheckStatus status, String explanation, double elapsedSeconds,
		Map<String, Double> measurements, FailureContext failureContext) {
	/** Compatibility constructor for results that do not have failure context. */
	public CheckResult(String id, String subsystem, CheckStatus status, String explanation, double elapsedSeconds,
			Map<String, Double> measurements) {
		this(id, subsystem, status, explanation, elapsedSeconds, measurements, FailureContext.none());
	}

	public CheckResult {
		measurements = Map.copyOf(measurements);
		failureContext = failureContext == null ? FailureContext.none() : failureContext;
	}

	/** Creates a result for a stage that has not started. */
	public static CheckResult notRun(String id, String subsystem) {
		return new CheckResult(id, subsystem, CheckStatus.NOT_RUN, "Not run", 0.0, Map.of());
	}

	/** Aggregates individual stage results into the overall displayed status. */
	public static CheckStatus aggregate(SystemCheckRunState runState, Iterable<CheckResult> results) {
		boolean warning = false;
		for (CheckResult result : results) {
			if (result.status() == CheckStatus.FAIL || result.status() == CheckStatus.ABORTED) {
				return CheckStatus.FAIL;
			}
			warning |= result.status() == CheckStatus.WARNING || result.status() == CheckStatus.SKIPPED;
		}
		if (runState == SystemCheckRunState.ABORTED) {
			return CheckStatus.FAIL;
		}
		if (warning) {
			return CheckStatus.WARNING;
		}
		return runState == SystemCheckRunState.COMPLETE ? CheckStatus.PASS : CheckStatus.NOT_RUN;
	}
}
