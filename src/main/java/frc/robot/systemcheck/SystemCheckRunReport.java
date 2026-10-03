package frc.robot.systemcheck;

import java.time.Instant;
import java.util.List;

/** Immutable data written to the standalone HTML, JSON, and CSV reports. */
public record SystemCheckRunReport(String runId, Instant startedAt, Instant endedAt, SystemCheckRunState runState,
		CheckStatus overallStatus, String finalReason, double elapsedSeconds, List<CheckResult> results,
		List<DeviceHealth> deviceHealth, List<CanBusSnapshot> canBuses,
		List<SystemCheckCanTopology.CanChainDiagnosis> canChainDiagnoses) {
	public SystemCheckRunReport {
		results = List.copyOf(results);
		deviceHealth = List.copyOf(deviceHealth);
		canBuses = List.copyOf(canBuses);
		canChainDiagnoses = List.copyOf(canChainDiagnoses);
	}
}
