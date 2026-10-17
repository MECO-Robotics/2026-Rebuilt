package frc.robot.systemcheck;

import com.sun.net.httpserver.HttpServer;
import java.io.IOException;
import java.net.InetSocketAddress;
import java.nio.charset.StandardCharsets;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ExecutorService;

/**
 * Read-only, memory-backed monitor. HTTP workers never access robot hardware.
 */
public final class SystemCheckLiveServer implements AutoCloseable {
	public static final int PORT = 5806;
	private static SystemCheckLiveServer shared;
	private final HttpServer server;
	private final ExecutorService worker;
	private volatile Snapshot snapshot = new Snapshot("null", System.nanoTime());
	private record Snapshot(String json, long updatedNanos) {
	}

	public static synchronized SystemCheckLiveServer shared() throws IOException {
		if (shared == null)
			shared = new SystemCheckLiveServer(PORT);
		return shared;
	}

	SystemCheckLiveServer(int port) throws IOException {
		byte[] page = java.nio.file.Files.readAllBytes(
				edu.wpi.first.wpilibj.Filesystem.getDeployDirectory().toPath().resolve("systemcheck/live.html"));
		server = HttpServer.create(new InetSocketAddress(port), 4);
		worker = Executors.newSingleThreadExecutor(task -> {
			Thread thread = new Thread(task, "SystemCheckLiveHTTP");
			thread.setDaemon(true);
			return thread;
		});
		server.setExecutor(worker);
		server.createContext("/", exchange -> {
			try {
				String path = exchange.getRequestURI().getPath();
				int status = 200;
				byte[] body;
				String type = "text/plain; charset=utf-8";
				if (!exchange.getRequestMethod().equals("GET")) {
					status = 405;
					exchange.getResponseHeaders().set("Allow", "GET");
					body = "Read-only monitor".getBytes(StandardCharsets.UTF_8);
				} else if (path.equals("/api/live")) {
					Snapshot current = snapshot;
					double age = (System.nanoTime() - current.updatedNanos()) / 1e9;
					body = ("{\"ageSeconds\":" + age + ",\"data\":" + current.json() + "}")
							.getBytes(StandardCharsets.UTF_8);
					type = "application/json; charset=utf-8";
				} else if (path.equals("/")) {
					body = page;
					type = "text/html; charset=utf-8";
				} else {
					status = 404;
					body = "Not found".getBytes(StandardCharsets.UTF_8);
				}
				exchange.getResponseHeaders().set("Content-Type", type);
				exchange.getResponseHeaders().set("Cache-Control", "no-store");
				exchange.getResponseHeaders().set("X-Content-Type-Options", "nosniff");
				exchange.sendResponseHeaders(status, body.length);
				exchange.getResponseBody().write(body);
			} finally {
				exchange.close();
			}
		});
		server.start();
	}

	int port() {
		return server.getAddress().getPort();
	}

	/**
	 * Called on the robot thread; immutable JSON is swapped without disk writes.
	 */
	public void update(String runId, String state, String overall, String active, String reason, double elapsed,
			double stageElapsed, double stageDuration, double countdown, boolean setup, boolean armed, String practice,
			List<CheckResult> rows) {
		StringBuilder out = new StringBuilder("{");
		out.append("\"runId\":").append(SystemCheckReportWriter.json(runId)).append(",\"state\":")
				.append(SystemCheckReportWriter.json(state)).append(",\"overall\":")
				.append(SystemCheckReportWriter.json(overall)).append(",\"active\":")
				.append(SystemCheckReportWriter.json(active)).append(",\"reason\":")
				.append(SystemCheckReportWriter.json(reason)).append(",\"elapsed\":").append(elapsed)
				.append(",\"stageElapsed\":").append(stageElapsed).append(",\"stageDuration\":").append(stageDuration)
				.append(",\"countdown\":").append(countdown).append(",\"setup\":").append(setup).append(",\"armed\":")
				.append(armed).append(",\"practice\":").append(SystemCheckReportWriter.json(practice))
				.append(",\"rows\":[");
		for (int i = 0; i < rows.size(); i++) {
			CheckResult row = rows.get(i);
			if (i > 0)
				out.append(',');
			out.append("{\"id\":").append(SystemCheckReportWriter.json(row.id())).append(",\"subsystem\":")
					.append(SystemCheckReportWriter.json(row.subsystem())).append(",\"status\":")
					.append(SystemCheckReportWriter.json(row.status().name())).append(",\"explanation\":")
					.append(SystemCheckReportWriter.json(row.explanation())).append(",\"seconds\":")
					.append(row.elapsedSeconds()).append(",\"measurements\":");
			SystemCheckReportWriter.appendMeasurements(out, row.measurements());
			out.append('}');
		}
		snapshot = new Snapshot(out.append("]}").toString(), System.nanoTime());
	}

	@Override
	public void close() {
		server.stop(0);
		worker.shutdownNow();
	}
}
