package frc.robot.systemcheck;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

class CheckResultTest {
	@Test
	void aggregatesPassWarningAndFailureStates() {
		CheckResult pass = result(CheckStatus.PASS);
		CheckResult warning = result(CheckStatus.WARNING);
		CheckResult skipped = result(CheckStatus.SKIPPED);
		CheckResult fail = result(CheckStatus.FAIL);

		assertEquals(CheckStatus.NOT_RUN, CheckResult.aggregate(SystemCheckRunState.RUNNING, List.of(pass)));
		assertEquals(CheckStatus.PASS, CheckResult.aggregate(SystemCheckRunState.COMPLETE, List.of(pass)));
		assertEquals(CheckStatus.WARNING, CheckResult.aggregate(SystemCheckRunState.COMPLETE, List.of(pass, warning)));
		assertEquals(CheckStatus.WARNING, CheckResult.aggregate(SystemCheckRunState.COMPLETE, List.of(pass, skipped)));
		assertEquals(CheckStatus.FAIL, CheckResult.aggregate(SystemCheckRunState.COMPLETE, List.of(pass, fail)));
		assertEquals(CheckStatus.FAIL, CheckResult.aggregate(SystemCheckRunState.ABORTED, List.of(pass)));
	}

	private static CheckResult result(CheckStatus status) {
		return new CheckResult("id", "subsystem", status, "test", 3.0, Map.of());
	}
}
