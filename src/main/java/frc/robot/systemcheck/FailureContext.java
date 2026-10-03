package frc.robot.systemcheck;

import java.util.List;

/** Timing, dependency, and hardware context for a failed or aborted check. */
public record FailureContext(String category, double conditionBeganStageSeconds, double declaredStageSeconds,
		double declaredRunSeconds, String diagnosticPath, List<String> failedPrerequisites,
		List<HardwareFault> hardwareFaults) {
	public FailureContext {
		failedPrerequisites = List.copyOf(failedPrerequisites);
		hardwareFaults = List.copyOf(hardwareFaults);
	}

	/** Empty context used by passing and not-yet-run checks. */
	public static FailureContext none() {
		return new FailureContext("NONE", Double.NaN, Double.NaN, Double.NaN, "", List.of(), List.of());
	}

	/** True when this result contains a failure diagnosis. */
	public boolean present() {
		return !category.equals("NONE");
	}
}
