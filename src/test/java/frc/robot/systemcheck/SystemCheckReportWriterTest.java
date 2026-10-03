package frc.robot.systemcheck;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.nio.file.Files;
import java.nio.file.Path;
import java.time.Instant;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class SystemCheckReportWriterTest {
	@TempDir
	Path temporaryDirectory;

	@Test
	void writesReadableAndMachineReadableReportsWithFailureTimingAndCanContext() throws Exception {
		HardwareFault hardware = new HardwareFault("ShooterFlywheel leader", "DISCONNECTED", "No telemetry", "MECO 2",
				34, 2, "BottomIndexer motor (CAN 31)", "ShooterFlywheel follower (CAN 35)", false);
		FailureContext failure = new FailureContext("DEVICE_DISCONNECTED", 0.4, 5.0, 42.0,
				"Stage -> device health -> CAN network", List.of(), List.of(hardware));
		List<CheckResult> results = List.of(
				new CheckResult("preflight", "Robot", CheckStatus.PASS, "All devices responded", 3.0,
						Map.of("BatteryVolts", 12.4)),
				new CheckResult("shooter-flywheel", "ShooterFlywheel", CheckStatus.FAIL, "Motor disconnected", 5.0,
						Map.of("VelocityRPS", 0.0), failure));
		DeviceHealth health = new DeviceHealth("ShooterFlywheel", new boolean[]{false, true}, new double[]{0.0, 0.0},
				new double[]{0.0, 0.0}, new double[]{40.0, 40.0}, new double[]{25.0, 25.0}, false, true,
				new String[]{"", ""}, new int[]{34, 35});
		CanBusSnapshot bus = new CanBusSnapshot("MECO 2", true, "OK", 0.25, 0, 0, 0, 0);
		SystemCheckRunReport report = new SystemCheckRunReport("12345678-test", Instant.parse("2026-10-02T12:00:00Z"),
				Instant.parse("2026-10-02T12:01:30Z"), SystemCheckRunState.COMPLETE, CheckStatus.FAIL,
				"Review recorded results", 90.0, results, List.of(health), List.of(bus),
				SystemCheckCanTopology.diagnoseChains(List.of(health)));
		SystemCheckReportWriter writer = new SystemCheckReportWriter(temporaryDirectory, "http://localhost:5805/",
				false);

		SystemCheckReportWriter.ReportFiles files = writer.write(report);
		String html = Files.readString(files.htmlPath());
		String json = Files.readString(files.jsonPath());
		String csv = Files.readString(files.csvPath());

		assertTrue(html.contains("Download full JSON log"));
		assertTrue(html.contains("ShooterFlywheel leader"));
		assertTrue(html.contains("declared failed at 5.000 s"));
		assertTrue(html.contains("BottomIndexer motor (CAN 31)"));
		assertTrue(json.contains("\"status\": \"PASS\""));
		assertTrue(json.contains("\"status\": \"FAIL\""));
		assertTrue(json.contains("\"declaredRunSeconds\": 42.0"));
		assertTrue(json.contains("\"canId\": 34"));
		assertTrue(csv.contains("shooter-flywheel"));
		assertTrue(Files.exists(temporaryDirectory.resolve("index.html")));
		assertTrue(Files.exists(temporaryDirectory.resolve("latest.json")));
	}
}
