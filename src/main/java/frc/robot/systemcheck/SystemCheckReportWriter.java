package frc.robot.systemcheck;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.RobotBase;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.AtomicMoveNotSupportedException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.time.ZoneOffset;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/** Writes and serves standalone system-check reports. */
public final class SystemCheckReportWriter {
	private static final DateTimeFormatter FILE_TIME = DateTimeFormatter.ofPattern("yyyyMMdd-HHmmss")
			.withZone(ZoneOffset.UTC);
	private static boolean webServerStarted;

	private final Path reportDirectory;
	private final String baseUrl;

	/** Creates the real-robot or desktop-simulation report service. */
	public static SystemCheckReportWriter createDefault() throws IOException {
		Path directory = RobotBase.isReal()
				? Path.of("/home/lvuser/system-check-reports")
				: Path.of("build", "system-check-reports").toAbsolutePath();
		String url = RobotBase.isReal()
				? "http://roborio-8324-frc.local:" + SystemCheckConstants.REPORT_WEB_PORT + "/"
				: "http://localhost:" + SystemCheckConstants.REPORT_WEB_PORT + "/";
		return new SystemCheckReportWriter(directory, url, true);
	}

	SystemCheckReportWriter(Path reportDirectory, String baseUrl, boolean startWebServer) throws IOException {
		this.reportDirectory = reportDirectory.toAbsolutePath().normalize();
		this.baseUrl = baseUrl.endsWith("/") ? baseUrl : baseUrl + "/";
		Files.createDirectories(this.reportDirectory);
		writeIndex();
		if (startWebServer) {
			startWebServerOnce();
		}
	}

	/** Writes one HTML report plus machine-readable JSON and CSV copies. */
	public ReportFiles write(SystemCheckRunReport report) throws IOException {
		String shortId = report.runId().length() <= 8 ? report.runId() : report.runId().substring(0, 8);
		String stem = "system-check-" + FILE_TIME.format(report.startedAt()) + "-" + safeFilePart(shortId);
		Path html = reportDirectory.resolve(stem + ".html");
		Path json = reportDirectory.resolve(stem + ".json");
		Path csv = reportDirectory.resolve(stem + ".csv");
		writeAtomic(html, renderHtml(report, stem));
		writeAtomic(json, renderJson(report));
		writeAtomic(csv, renderCsv(report));
		writeAtomic(reportDirectory.resolve("latest.html"), renderLatestRedirect(stem + ".html"));
		writeAtomic(reportDirectory.resolve("latest.json"), renderJson(report));
		writeAtomic(reportDirectory.resolve("latest.csv"), renderCsv(report));
		writeIndex();
		return new ReportFiles(html, json, csv, baseUrl + stem + ".html", baseUrl + stem + ".json",
				baseUrl + stem + ".csv");
	}

	public Path reportDirectory() {
		return reportDirectory;
	}

	public String indexUrl() {
		return baseUrl;
	}

	private synchronized void startWebServerOnce() {
		if (webServerStarted) {
			return;
		}
		WebServer.start(SystemCheckConstants.REPORT_WEB_PORT, reportDirectory.toString());
		webServerStarted = true;
	}

	private void writeIndex() throws IOException {
		List<Path> reports = new ArrayList<>();
		try (var files = Files.list(reportDirectory)) {
			files.filter(path -> path.getFileName().toString().startsWith("system-check-")
					&& path.getFileName().toString().endsWith(".html"))
					.sorted(Comparator.comparing((Path path) -> path.getFileName().toString()).reversed())
					.forEach(reports::add);
		}
		StringBuilder html = new StringBuilder();
		html.append("<!doctype html><html><head><meta charset=\"utf-8\"><meta name=\"viewport\" ")
				.append("content=\"width=device-width,initial-scale=1\"><title>Robot System Check Reports</title>")
				.append(styles()).append("</head><body><main><h1>Robot System Check Reports</h1>")
				.append("<p>Open a report to inspect every stage, or download its JSON/CSV copies.</p>");
		if (reports.isEmpty()) {
			html.append("<p>No completed or aborted system-check run has been recorded yet.</p>");
		} else {
			html.append("<ul class=\"reports\">");
			for (Path report : reports) {
				String name = report.getFileName().toString();
				String stem = name.substring(0, name.length() - 5);
				html.append("<li><a href=\"").append(attribute(name)).append("\">").append(text(stem))
						.append("</a> &middot; <a download href=\"").append(attribute(stem)).append(".json\">JSON</a>")
						.append(" &middot; <a download href=\"").append(attribute(stem)).append(".csv\">CSV</a></li>");
			}
			html.append("</ul>");
		}
		html.append("</main></body></html>");
		writeAtomic(reportDirectory.resolve("index.html"), html.toString());
	}

	private static String renderHtml(SystemCheckRunReport report, String stem) {
		StringBuilder html = new StringBuilder();
		html.append("<!doctype html><html><head><meta charset=\"utf-8\"><meta name=\"viewport\" ")
				.append("content=\"width=device-width,initial-scale=1\"><title>System Check ")
				.append(text(report.runId())).append("</title>").append(styles()).append("</head><body><main>")
				.append("<p><a href=\"index.html\">&larr; all reports</a></p><h1>Robot System Check</h1>")
				.append("<div class=\"summary ").append(statusClass(report.overallStatus())).append("\">")
				.append("<strong>").append(text(report.overallStatus().toString())).append("</strong> &mdash; ")
				.append(text(report.finalReason())).append("</div>").append("<p><a download href=\"")
				.append(attribute(stem)).append(".json\">Download full JSON log</a>")
				.append(" &middot; <a download href=\"").append(attribute(stem))
				.append(".csv\">Download stage CSV</a></p>").append("<dl><dt>Run ID</dt><dd>")
				.append(text(report.runId())).append("</dd><dt>Started (UTC)</dt><dd>")
				.append(text(report.startedAt().toString())).append("</dd><dt>Ended (UTC)</dt><dd>")
				.append(text(report.endedAt().toString())).append("</dd><dt>Run state</dt><dd>")
				.append(text(report.runState().toString())).append("</dd><dt>Total elapsed</dt><dd>")
				.append(format(report.elapsedSeconds())).append(" s</dd></dl>");

		html.append("<h2>Check sequence</h2><div class=\"table-wrap\"><table><thead><tr><th>#</th><th>Check</th>")
				.append("<th>Subsystem</th><th>Status</th><th>Decision time</th><th>Explanation and measurements</th>")
				.append("<th>Failure diagnosis</th></tr></thead><tbody>");
		int number = 1;
		for (CheckResult result : report.results()) {
			html.append("<tr><td>").append(number++).append("</td><td>").append(text(result.id())).append("</td><td>")
					.append(text(result.subsystem())).append("</td><td class=\"").append(statusClass(result.status()))
					.append("\">").append(text(result.status().toString())).append("</td><td>")
					.append(format(result.elapsedSeconds())).append(" s</td><td>").append(text(result.explanation()));
			if (!result.measurements().isEmpty()) {
				html.append("<ul>");
				for (Map.Entry<String, Double> measurement : result.measurements().entrySet()) {
					html.append("<li>").append(text(measurement.getKey())).append(": ")
							.append(format(measurement.getValue())).append("</li>");
				}
				html.append("</ul>");
			}
			html.append("</td><td>").append(renderFailure(result.failureContext())).append("</td></tr>");
		}
		html.append("</tbody></table></div>");

		html.append("<h2>CAN networks</h2><div class=\"table-wrap\"><table><thead><tr><th>Bus</th><th>Status</th>")
				.append("<th>Utilization</th><th>Bus-off</th><th>TX full</th><th>REC</th><th>TEC</th></tr></thead><tbody>");
		for (CanBusSnapshot bus : report.canBuses()) {
			html.append("<tr><td>").append(text(bus.name())).append("</td><td>").append(text(bus.status()))
					.append("</td><td>").append(format(bus.utilization() * 100.0)).append("%</td><td>")
					.append(bus.busOffCount()).append("</td><td>").append(bus.txFullCount()).append("</td><td>")
					.append(bus.receiveErrorCount()).append("</td><td>").append(bus.transmitErrorCount())
					.append("</td></tr>");
		}
		html.append("</tbody></table></div><h3>CAN chain localization</h3>");
		for (SystemCheckCanTopology.CanChainDiagnosis diagnosis : report.canChainDiagnoses()) {
			html.append("<section><h4>").append(text(diagnosis.busName())).append("</h4><p>Physical order: <strong>")
					.append(diagnosis.physicalOrderVerified() ? "VERIFIED" : "NOT VERIFIED").append("</strong></p><p>")
					.append(text(diagnosis.likelyBreak())).append("</p><p>Disconnected: ")
					.append(text(diagnosis.disconnectedDevices().isEmpty()
							? "none"
							: String.join(", ", diagnosis.disconnectedDevices())))
					.append("</p></section>");
		}

		html.append("<h2>Final hardware snapshot</h2><div class=\"table-wrap\"><table><thead><tr><th>Group</th>")
				.append("<th>Connected</th><th>CAN IDs</th><th>Velocity</th><th>Current (A)</th><th>Temperature (C)</th>")
				.append("<th>Vendor faults</th></tr></thead><tbody>");
		for (DeviceHealth health : report.deviceHealth()) {
			html.append("<tr><td>").append(text(health.name())).append("</td><td>").append(health.allConnected())
					.append("</td><td>").append(text(Arrays.toString(health.deviceIds()))).append("</td><td>")
					.append(text(Arrays.toString(health.normalizedVelocities()))).append("</td><td>")
					.append(text(Arrays.toString(health.currentsAmps()))).append("</td><td>")
					.append(text(Arrays.toString(health.temperaturesCelsius()))).append("</td><td>")
					.append(text(Arrays.toString(health.activeFaults()))).append("</td></tr>");
		}
		html.append(
				"</tbody></table></div><p class=\"note\">A CAN break can only be localized to physical neighbors after ")
				.append("the order in SystemCheckCanTopology.java has been checked against the actual yellow/green wiring.</p>")
				.append("</main></body></html>");
		return html.toString();
	}

	private static String renderFailure(FailureContext failure) {
		if (!failure.present()) {
			return "&mdash;";
		}
		StringBuilder html = new StringBuilder();
		html.append("<strong>").append(text(failure.category())).append("</strong><br>Condition began at ")
				.append(format(failure.conditionBeganStageSeconds())).append(" s; declared failed at ")
				.append(format(failure.declaredStageSeconds())).append(" s into this check (")
				.append(format(failure.declaredRunSeconds())).append(" s into the run).<br>")
				.append(text(failure.diagnosticPath()));
		if (!failure.failedPrerequisites().isEmpty()) {
			html.append("<br>Failed prerequisites: ").append(text(String.join(", ", failure.failedPrerequisites())));
		}
		for (HardwareFault fault : failure.hardwareFaults()) {
			html.append("<hr><strong>").append(text(fault.hardware())).append("</strong> &mdash; ")
					.append(text(fault.faultType())).append(": ").append(text(fault.details()));
			if (fault.canId() >= 0) {
				html.append("<br>CAN bus ").append(text(fault.canBus())).append(", ID ").append(fault.canId())
						.append(", configured chain position ").append(fault.chainPosition()).append("<br>Upstream: ")
						.append(text(fault.upstreamNeighbor())).append("<br>Downstream: ")
						.append(text(fault.downstreamNeighbor())).append("<br>Physical order verified: ")
						.append(fault.physicalOrderVerified());
			}
		}
		return html.toString();
	}

	private static String renderJson(SystemCheckRunReport report) {
		StringBuilder json = new StringBuilder();
		json.append("{\n  \"schemaVersion\": 1,\n  \"runId\": ").append(json(report.runId()))
				.append(",\n  \"startedAtUtc\": ").append(json(report.startedAt().toString()))
				.append(",\n  \"endedAtUtc\": ").append(json(report.endedAt().toString())).append(",\n  \"runState\": ")
				.append(json(report.runState().toString())).append(",\n  \"overallStatus\": ")
				.append(json(report.overallStatus().toString())).append(",\n  \"finalReason\": ")
				.append(json(report.finalReason())).append(",\n  \"elapsedSeconds\": ")
				.append(number(report.elapsedSeconds())).append(",\n  \"canTopologyPhysicalOrderVerified\": ")
				.append(SystemCheckCanTopology.PHYSICAL_ORDER_VERIFIED).append(",\n  \"checks\": [\n");
		for (int i = 0; i < report.results().size(); i++) {
			CheckResult result = report.results().get(i);
			json.append("    {\"sequence\": ").append(i + 1).append(", \"id\": ").append(json(result.id()))
					.append(", \"subsystem\": ").append(json(result.subsystem())).append(", \"status\": ")
					.append(json(result.status().toString())).append(", \"explanation\": ")
					.append(json(result.explanation())).append(", \"elapsedSeconds\": ")
					.append(number(result.elapsedSeconds())).append(", \"measurements\": ");
			appendMeasurements(json, result.measurements());
			json.append(", \"failure\": ");
			appendFailureJson(json, result.failureContext());
			json.append("}").append(i + 1 < report.results().size() ? "," : "").append("\n");
		}
		json.append("  ],\n  \"canBuses\": [\n");
		for (int i = 0; i < report.canBuses().size(); i++) {
			CanBusSnapshot bus = report.canBuses().get(i);
			json.append("    {\"name\": ").append(json(bus.name())).append(", \"available\": ").append(bus.available())
					.append(", \"status\": ").append(json(bus.status())).append(", \"utilization\": ")
					.append(number(bus.utilization())).append(", \"busOffCount\": ").append(bus.busOffCount())
					.append(", \"txFullCount\": ").append(bus.txFullCount()).append(", \"receiveErrorCount\": ")
					.append(bus.receiveErrorCount()).append(", \"transmitErrorCount\": ")
					.append(bus.transmitErrorCount()).append("}").append(i + 1 < report.canBuses().size() ? "," : "")
					.append("\n");
		}
		json.append("  ],\n  \"canChainDiagnoses\": [\n");
		for (int i = 0; i < report.canChainDiagnoses().size(); i++) {
			var diagnosis = report.canChainDiagnoses().get(i);
			json.append("    {\"busName\": ").append(json(diagnosis.busName())).append(", \"physicalOrderVerified\": ")
					.append(diagnosis.physicalOrderVerified()).append(", \"disconnectedDevices\": ");
			appendStrings(json, diagnosis.disconnectedDevices());
			json.append(", \"likelyBreak\": ").append(json(diagnosis.likelyBreak())).append("}")
					.append(i + 1 < report.canChainDiagnoses().size() ? "," : "").append("\n");
		}
		json.append("  ],\n  \"finalDeviceHealth\": [\n");
		for (int i = 0; i < report.deviceHealth().size(); i++) {
			DeviceHealth health = report.deviceHealth().get(i);
			json.append("    {\"name\": ").append(json(health.name())).append(", \"allConnected\": ")
					.append(health.allConnected()).append(", \"connections\": ")
					.append(Arrays.toString(health.connections())).append(", \"encoderExpected\": ")
					.append(health.encoderExpected()).append(", \"encoderConnected\": ")
					.append(health.encoderConnected()).append(", \"deviceIds\": ")
					.append(Arrays.toString(health.deviceIds())).append(", \"normalizedVelocities\": ")
					.append(doubleArrayJson(health.normalizedVelocities())).append(", \"currentsAmps\": ")
					.append(doubleArrayJson(health.currentsAmps())).append(", \"currentLimitsAmps\": ")
					.append(doubleArrayJson(health.currentLimitsAmps())).append(", \"temperaturesCelsius\": ")
					.append(doubleArrayJson(health.temperaturesCelsius())).append(", \"activeFaults\": ");
			appendStrings(json, Arrays.asList(health.activeFaults()));
			json.append("}").append(i + 1 < report.deviceHealth().size() ? "," : "").append("\n");
		}
		return json.append("  ]\n}\n").toString();
	}

	private static void appendFailureJson(StringBuilder json, FailureContext failure) {
		if (!failure.present()) {
			json.append("null");
			return;
		}
		json.append("{\"category\": ").append(json(failure.category())).append(", \"conditionBeganStageSeconds\": ")
				.append(number(failure.conditionBeganStageSeconds())).append(", \"declaredStageSeconds\": ")
				.append(number(failure.declaredStageSeconds())).append(", \"declaredRunSeconds\": ")
				.append(number(failure.declaredRunSeconds())).append(", \"diagnosticPath\": ")
				.append(json(failure.diagnosticPath())).append(", \"failedPrerequisites\": ");
		appendStrings(json, failure.failedPrerequisites());
		json.append(", \"hardwareFaults\": [");
		for (int i = 0; i < failure.hardwareFaults().size(); i++) {
			HardwareFault fault = failure.hardwareFaults().get(i);
			json.append("{\"hardware\": ").append(json(fault.hardware())).append(", \"faultType\": ")
					.append(json(fault.faultType())).append(", \"details\": ").append(json(fault.details()))
					.append(", \"canBus\": ").append(json(fault.canBus())).append(", \"canId\": ").append(fault.canId())
					.append(", \"chainPosition\": ").append(fault.chainPosition()).append(", \"upstreamNeighbor\": ")
					.append(json(fault.upstreamNeighbor())).append(", \"downstreamNeighbor\": ")
					.append(json(fault.downstreamNeighbor())).append(", \"physicalOrderVerified\": ")
					.append(fault.physicalOrderVerified()).append("}");
			if (i + 1 < failure.hardwareFaults().size()) {
				json.append(",");
			}
		}
		json.append("]}");
	}

	private static String renderCsv(SystemCheckRunReport report) {
		StringBuilder csv = new StringBuilder();
		csv.append("sequence,check_id,subsystem,status,explanation,decision_seconds,failure_category,").append(
				"condition_began_seconds,declared_run_seconds,failed_prerequisites,hardware_faults,measurements\n");
		for (int i = 0; i < report.results().size(); i++) {
			CheckResult result = report.results().get(i);
			FailureContext failure = result.failureContext();
			String hardware = failure.hardwareFaults().stream()
					.map(fault -> fault.hardware() + " [" + fault.faultType() + "; " + fault.canBus() + " CAN "
							+ fault.canId() + "; upstream=" + fault.upstreamNeighbor() + "; downstream="
							+ fault.downstreamNeighbor() + "]")
					.collect(java.util.stream.Collectors.joining(" | "));
			csv.append(i + 1).append(',').append(csv(result.id())).append(',').append(csv(result.subsystem()))
					.append(',').append(csv(result.status().toString())).append(',').append(csv(result.explanation()))
					.append(',').append(number(result.elapsedSeconds())).append(',')
					.append(csv(failure.present() ? failure.category() : "")).append(',')
					.append(failure.present() ? number(failure.conditionBeganStageSeconds()) : "").append(',')
					.append(failure.present() ? number(failure.declaredRunSeconds()) : "").append(',')
					.append(csv(String.join(" | ", failure.failedPrerequisites()))).append(',').append(csv(hardware))
					.append(',').append(csv(result.measurements().toString())).append('\n');
		}
		return csv.toString();
	}

	private static void appendMeasurements(StringBuilder json, Map<String, Double> values) {
		json.append("{");
		int index = 0;
		for (Map.Entry<String, Double> value : values.entrySet()) {
			if (index++ > 0) {
				json.append(", ");
			}
			json.append(json(value.getKey())).append(": ").append(number(value.getValue()));
		}
		json.append("}");
	}

	private static void appendStrings(StringBuilder json, List<String> values) {
		json.append("[");
		for (int i = 0; i < values.size(); i++) {
			json.append(json(values.get(i)));
			if (i + 1 < values.size()) {
				json.append(", ");
			}
		}
		json.append("]");
	}

	private static String doubleArrayJson(double[] values) {
		StringBuilder json = new StringBuilder("[");
		for (int i = 0; i < values.length; i++) {
			json.append(number(values[i]));
			if (i + 1 < values.length) {
				json.append(", ");
			}
		}
		return json.append("]").toString();
	}

	private static String renderLatestRedirect(String target) {
		return "<!doctype html><html><head><meta charset=\"utf-8\"><meta http-equiv=\"refresh\" content=\"0;url="
				+ attribute(target) + "\"><title>Latest system check</title></head><body><a href=\"" + attribute(target)
				+ "\">Open latest system check</a></body></html>";
	}

	private static String styles() {
		return "<style>body{font-family:system-ui,sans-serif;background:#111827;color:#e5e7eb;margin:0}"
				+ "main{max-width:1500px;margin:auto;padding:24px}a{color:#60a5fa}table{border-collapse:collapse;width:100%;background:#1f2937}"
				+ "th,td{border:1px solid #4b5563;padding:8px;text-align:left;vertical-align:top}th{background:#374151}"
				+ ".table-wrap{overflow-x:auto}.summary{padding:14px;border-radius:8px;background:#374151}.pass{color:#6ee7b7}"
				+ ".warning{color:#fde68a}.fail,.aborted{color:#fca5a5}.not_run,.running,.skipped{color:#d1d5db}"
				+ "dl{display:grid;grid-template-columns:max-content 1fr;gap:6px 16px}dt{font-weight:bold}.note{color:#fcd34d}"
				+ ".reports li{margin:.6rem 0}hr{border:0;border-top:1px solid #4b5563}</style>";
	}

	private static String statusClass(CheckStatus status) {
		return status.toString().toLowerCase(Locale.ROOT);
	}

	private static String format(double value) {
		return Double.isFinite(value) ? String.format(Locale.ROOT, "%.3f", value) : "n/a";
	}

	private static String number(double value) {
		return Double.isFinite(value) ? Double.toString(value) : "null";
	}

	private static String safeFilePart(String value) {
		return value.replaceAll("[^A-Za-z0-9_-]", "_");
	}

	private static String text(String value) {
		return value == null
				? ""
				: value.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;").replace("\"", "&quot;")
						.replace("'", "&#39;");
	}

	private static String attribute(String value) {
		return text(value);
	}

	private static String json(String value) {
		if (value == null) {
			return "null";
		}
		StringBuilder escaped = new StringBuilder("\"");
		for (int i = 0; i < value.length(); i++) {
			char character = value.charAt(i);
			switch (character) {
				case '\\' -> escaped.append("\\\\");
				case '\"' -> escaped.append("\\\"");
				case '\n' -> escaped.append("\\n");
				case '\r' -> escaped.append("\\r");
				case '\t' -> escaped.append("\\t");
				default -> {
					if (character < 0x20) {
						escaped.append(String.format(Locale.ROOT, "\\u%04x", (int) character));
					} else {
						escaped.append(character);
					}
				}
			}
		}
		return escaped.append('\"').toString();
	}

	private static String csv(String value) {
		String safe = value == null ? "" : value;
		return "\"" + safe.replace("\"", "\"\"") + "\"";
	}

	private static void writeAtomic(Path target, String contents) throws IOException {
		Path temporary = target.resolveSibling(target.getFileName() + ".tmp");
		Files.writeString(temporary, contents, StandardCharsets.UTF_8);
		try {
			Files.move(temporary, target, StandardCopyOption.ATOMIC_MOVE, StandardCopyOption.REPLACE_EXISTING);
		} catch (AtomicMoveNotSupportedException exception) {
			Files.move(temporary, target, StandardCopyOption.REPLACE_EXISTING);
		}
	}

	/** Paths and URLs created for one report. */
	public record ReportFiles(Path htmlPath, Path jsonPath, Path csvPath, String htmlUrl, String jsonUrl,
			String csvUrl) {
	}
}
