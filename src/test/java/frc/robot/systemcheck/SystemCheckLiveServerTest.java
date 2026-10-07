package frc.robot.systemcheck;

import static org.junit.jupiter.api.Assertions.*;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

class SystemCheckLiveServerTest {
	@Test
	void servesLiveTransitionsAndRejectsCommandsWithoutChangingSnapshot() throws Exception {
		try (var server = new SystemCheckLiveServer(0)) {
			var client = HttpClient.newHttpClient();
			String base = "http://localhost:" + server.port();
			var page = client.send(HttpRequest.newBuilder(URI.create(base + "/")).build(),
					HttpResponse.BodyHandlers.ofString());
			assertEquals(200, page.statusCode());
			assertTrue(page.body().contains("Live system check"));
			var initial = get(client, base);
			assertTrue(initial.contains("\"data\":null"));
			List<CheckResult> rows = List.of(new CheckResult("intake-rack-safe", "IntakeRack", CheckStatus.RUNNING,
					"Waiting for \"target\"\n<check>", 1.2,
					Map.of("Target", .13, "Position", .08, "Invalid", Double.NaN)));
			server.update("run-1", "RUNNING", "NOT_RUN", "intake-rack-safe", "", 6, 1.2, 5, 0, true, true, "NOT_RUN",
					rows);
			String running = get(client, base);
			assertTrue(running.contains("\"state\":\"RUNNING\""));
			assertTrue(running.contains("\\\"target\\\"\\n<check>"));
			assertTrue(running.contains("\"Position\": 0.08"));
			assertTrue(running.contains("\"Invalid\": null"));
			assertTrue(running.contains("\"ageSeconds\":"));
			var rejected = client.send(
					HttpRequest.newBuilder(URI.create(base + "/api/live"))
							.POST(HttpRequest.BodyPublishers.ofString("start")).build(),
					HttpResponse.BodyHandlers.ofString());
			assertEquals(405, rejected.statusCode());
			assertTrue(get(client, base).contains("\"state\":\"RUNNING\""));
			server.update("run-1", "ABORTED", "FAIL", "", "Disabled by operator", 7, 0, 0, 0, true, false, "NOT_RUN",
					List.of(new CheckResult("intake-rack-safe", "IntakeRack", CheckStatus.ABORTED,
							"Disabled by operator", 2.2, Map.of())));
			String aborted = get(client, base);
			assertTrue(aborted.contains("\"state\":\"ABORTED\""));
			assertTrue(aborted.contains("Disabled by operator"));
			server.update("run-2", "COUNTDOWN", "NOT_RUN", "", "", 0, 0, 0, 3, true, true, "NOT_RUN",
					List.of(CheckResult.notRun("intake-rack-safe", "IntakeRack")));
			assertFalse(get(client, base).contains("Disabled by operator"));
			assertEquals("no-store", page.headers().firstValue("Cache-Control").orElseThrow());
		}
	}

	private static String get(HttpClient client, String base) throws Exception {
		var response = client.send(HttpRequest.newBuilder(URI.create(base + "/api/live")).build(),
				HttpResponse.BodyHandlers.ofString());
		assertEquals(200, response.statusCode());
		return response.body();
	}
}
