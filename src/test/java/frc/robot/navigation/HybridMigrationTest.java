package frc.robot.navigation;

import static org.junit.jupiter.api.Assertions.*;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Commands;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import org.json.simple.JSONObject;
import org.json.simple.JSONArray;
import org.json.simple.parser.JSONParser;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class HybridMigrationTest {
	@BeforeAll
	static void initialize() {
		assertTrue(HAL.initialize(500, 0));
	}

	@Test
	void everyNonemptyCompetitionAutoHasAnEditableCandidateWithTheSameCommandTree() throws Exception {
		var directory = Filesystem.getDeployDirectory().toPath().resolve("pathplanner/autos");
		int candidates = 0;
		try (var files = Files.list(directory)) {
			for (var file : files.filter(f -> f.toString().endsWith(".auto")).sorted().toList()) {
				String name = file.getFileName().toString().replace(".auto", "");
				if (HybridAutos.isHybridAuto(name))
					continue;
				var original = (JSONObject) new JSONParser().parse(Files.readString(file));
				var tree = original.get("command").toString();
				if (name.equals("16th Note Auto"))
					continue; // Intentionally empty placeholder, not a navigation auto.
				var candidate = directory.resolve(HybridAutos.MIGRATION_PREFIX + name + ".auto");
				assertTrue(Files.exists(candidate), name);
				var migrated = (JSONObject) new JSONParser().parse(Files.readString(candidate));
				assertEquals(false, migrated.get("resetOdom"));
				if (!name.equals("Moses 3 Cycle"))
					assertEquals(tree,
							collapseSplitPaths((JSONObject) migrated.get("command")).toString().replace("HYBRID ", ""),
							name);
				candidates++;
			}
		}
		assertEquals(8, candidates);
	}

	/**
	 * Collapse only the explicitly migrated path groups to compare all surrounding
	 * mechanism structure.
	 */
	@SuppressWarnings("unchecked")
	private static JSONObject collapseSplitPaths(JSONObject node) {
		var data = (JSONObject) node.get("data");
		if (data.get("commands")instanceof JSONArray children) {
			if (node.get("type").equals("sequential") && !children.isEmpty()) {
				var first = (JSONObject) children.get(0);
				var name = (String) ((JSONObject) first.get("data")).get("pathName");
				if (name != null
						&& name.matches("HYBRID (Left Bump 2|Left Bump Rush Depot|Right Bump Rush Depot) 01")) {
					var base = name.substring(0, name.length() - 3);
					assertEquals(base.equals("HYBRID Left Bump 2") ? 6 : 3, children.size());
					for (int i = 0; i < children.size(); i++)
						assertEquals(jsonPath(base + " %02d".formatted(i + 1)), children.get(i));
					return jsonPath(base);
				}
			}
			for (int i = 0; i < children.size(); i++)
				children.set(i, collapseSplitPaths((JSONObject) children.get(i)));
		}
		return node;
	}

	@SuppressWarnings("unchecked")
	private static JSONObject jsonPath(String name) {
		var data = new JSONObject();
		data.put("pathName", name);
		var node = new JSONObject();
		node.put("type", "path");
		node.put("data", data);
		return node;
	}

	@Test
	void writesReadinessReportUsingActualPathGeometryAndBothAlliances() throws Exception {
		var safety = NavigationSafety.fromDeploy(RobotNavigationFootprint.DEPLOYED_RADIUS_METERS);
		var directory = Filesystem.getDeployDirectory().toPath().resolve("pathplanner/autos");
		var lines = new ArrayList<String>();
		lines.add("Auto\tMigration\tClearance\tBlocking issue");
		var known = Set.of("DeployIntake", "StowIntake", "SpinIntake", "AutoSpinUp", "AutoAim", "FeedRollers",
				"IdleRollers", "Agitate", "CoordinatedHubShot");
		try (var files = Files.list(directory)) {
			for (var file : files.filter(f -> f.getFileName().toString().startsWith("HYBRID ")).sorted().toList()) {
				String name = file.getFileName().toString().replace(".auto", "");
				var definition = HybridAutoDefinition.load(name);
				assertEquals(List.of(), definition.structuralIssues(), name);
				// Validate all named commands exist, including candidates currently blocked by
				// geometry.
				definition.build(command -> {
					assertTrue(known.contains(command), command);
					return Commands.none();
				}, path -> Commands.none());
				for (var path : definition.paths().values()) {
					assertEquals(0, path.getGoalEndState().velocityMPS());
					assertEquals(0, path.getIdealStartingState().velocityMPS());
				}
				var issues = definition.clearanceIssues(safety);
				if (issues.isEmpty())
					lines.add(name + "\tPASS\tPASS\tHardware testing required");
				for (var issue : issues)
					lines.add(name + "\tPASS\tPENDING\t" + issue);
			}
		}
		var report = Path.of("build/reports/hybrid/migration-readiness.tsv");
		Files.createDirectories(report.getParent());
		Files.write(report, lines);
		assertTrue(lines.size() > 1);
	}

	private static String auto(String tree) {
		return """
				{"version":"2025.0","resetOdom":false,"choreoAuto":false,"command":%s}
				""".formatted(tree);
	}

	@Test
	void splitPathsPreserveEveryAuthoredBezierSpanAndConnectAtRest() throws Exception {
		var dir = Filesystem.getDeployDirectory().toPath().resolve("pathplanner/paths");
		for (var name : List.of("Left Bump 2", "Left Bump Rush Depot", "Right Bump Rush Depot", "Moses 3 cycle")) {
			var source = (JSONObject) new JSONParser().parse(Files.readString(dir.resolve(name + ".path")));
			var waypoints = (JSONArray) source.get("waypoints");
			Double previousRotation = null;
			for (int i = 1; i < waypoints.size(); i++) {
				var split = (JSONObject) new JSONParser()
						.parse(Files.readString(dir.resolve("HYBRID " + name + " %02d.path".formatted(i))));
				var points = (JSONArray) split.get("waypoints");
				assertEquals(2, points.size());
				var start = (JSONObject) points.get(0);
				var end = (JSONObject) points.get(1);
				var oldStart = (JSONObject) waypoints.get(i - 1);
				var oldEnd = (JSONObject) waypoints.get(i);
				assertEquals(oldStart.get("anchor"), start.get("anchor"));
				assertEquals(oldStart.get("nextControl"), start.get("nextControl"));
				assertEquals(oldEnd.get("anchor"), end.get("anchor"));
				assertEquals(oldEnd.get("prevControl"), end.get("prevControl"));
				assertEquals(source.get("globalConstraints"), split.get("globalConstraints"));
				assertEquals(List.of(), split.get("eventMarkers"));
				var initial = (JSONObject) split.get("idealStartingState");
				var goal = (JSONObject) split.get("goalEndState");
				if (previousRotation != null)
					assertEquals(previousRotation, ((Number) initial.get("rotation")).doubleValue(), 1e-9);
				previousRotation = ((Number) goal.get("rotation")).doubleValue();
			}
		}
	}

	@Test
	void mosesShootingStepsFollowTheTwoAuthoredShootingDestinations() throws Exception {
		var definition = HybridAutoDefinition.load("HYBRID Moses 3 Cycle");
		var order = new ArrayList<String>();
		definition.build(name -> {
			order.add(name);
			return Commands.none();
		}, path -> {
			order.add(path);
			return Commands.none();
		});
		assertEquals(2, order.stream().filter("CoordinatedHubShot"::equals).count());
		for (var leg : List.of("04", "08")) {
			int approach = order.indexOf("HYBRID Moses 3 cycle " + leg);
			assertEquals("AutoSpinUp", order.get(approach + 1));
			assertEquals("CoordinatedHubShot", order.get(approach + 2));
		}
		assertEquals(3, order.stream().filter("SpinIntake"::equals).count());
		assertEquals("DeployIntake", order.get(0));
		assertEquals(11, definition.paths().size());
		assertEquals(java.util.stream.IntStream.rangeClosed(1, 11)
				.mapToObj(i -> "HYBRID Moses 3 cycle %02d".formatted(i)).toList(),
				order.stream().filter(name -> name.startsWith("HYBRID ")).toList());
	}

	@Test
	void deployedFootprintAccountsForFrontOnlyExtension() {
		assertEquals(.6381623, RobotNavigationFootprint.FRONT_EXTENT_METERS, 1e-7);
		assertEquals(.816980328, RobotNavigationFootprint.DEPLOYED_RADIUS_METERS, 1e-7);
		assertTrue(RobotNavigationFootprint.FRONT_EXTENT_METERS > RobotNavigationFootprint.DEPLOYED_LENGTH_METERS / 2);
	}

	@Test
	void rejectsGroupsThatCanSkipArrivalOrRunTwoDriveCommands() {
		for (var tree : List.of("""
				{"type":"race","data":{"commands":[{"type":"path","data":{"pathName":"Hybrid Survey Out"}},
				{"type":"wait","data":{"waitTime":1}}]}}
				""", """
				{"type":"deadline","data":{"commands":[{"type":"wait","data":{"waitTime":1}},
				{"type":"path","data":{"pathName":"Hybrid Survey Out"}}]}}
				""", """
				{"type":"parallel","data":{"commands":[{"type":"path","data":{"pathName":"Hybrid Survey Out"}},
				{"type":"path","data":{"pathName":"Hybrid Survey Return"}}]}}
				"""))
			assertThrows(IllegalArgumentException.class,
					() -> HybridAutoDefinition.parse(auto(tree), PathPlannerPath::fromPathFile));
	}

	@Test
	void unknownCommandsAndInvalidWaitsFailClosed() throws Exception {
		var tree = """
				{"type":"sequential","data":{"commands":[
				{"type":"path","data":{"pathName":"Hybrid Survey Out"}},
				{"type":"named","data":{"name":"Typo"}}]}}
				""";
		var definition = HybridAutoDefinition.parse(auto(tree), PathPlannerPath::fromPathFile);
		assertThrows(IllegalArgumentException.class, () -> definition.build(name -> null, path -> Commands.none()));
		assertThrows(IllegalArgumentException.class,
				() -> HybridAutoDefinition.parse(auto(
						tree.replace("\"named\",\"data\":{\"name\":\"Typo\"}", "\"wait\",\"data\":{\"waitTime\":-1}")),
						PathPlannerPath::fromPathFile));
	}
}
