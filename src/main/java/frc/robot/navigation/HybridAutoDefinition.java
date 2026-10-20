package frc.robot.navigation;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

/**
 * A PathPlanner command tree with explicit, independently validated drive
 * objectives.
 */
final class HybridAutoDefinition {
	private final Node root;
	private final Map<String, PathPlannerPath> paths;

	private record Node(String type, String name, double seconds, List<Node> children) {
		boolean drives() {
			return type.equals("path") || children.stream().anyMatch(Node::drives);
		}

		Command build(Function<String, Command> named, Function<String, Command> path) {
			var commands = children.stream().map(child -> child.build(named, path)).toArray(Command[]::new);
			return switch (type) {
				case "path" -> path.apply(name);
				case "named" -> {
					var command = named.apply(name);
					if (command == null)
						throw new IllegalArgumentException("Unknown named command: " + name);
					yield command;
				}
				case "wait" -> Commands.waitSeconds(seconds);
				case "sequential" -> Commands.sequence(commands);
				case "parallel" -> Commands.parallel(commands);
				case "race" -> Commands.race(commands);
				case "deadline" -> Commands.deadline(commands[0],
						java.util.Arrays.copyOfRange(commands, 1, commands.length));
				default -> throw new IllegalArgumentException("Unsupported command: " + type);
			};
		}
	}

	private HybridAutoDefinition(Node root, Map<String, PathPlannerPath> paths) {
		this.root = root;
		this.paths = paths;
	}

	static HybridAutoDefinition load(String name) throws Exception {
		var file = Filesystem.getDeployDirectory().toPath().resolve("pathplanner/autos/" + name + ".auto");
		return parse(Files.readString(file), PathPlannerPath::fromPathFile);
	}

	@FunctionalInterface
	interface PathLoader {
		PathPlannerPath load(String name) throws Exception;
	}

	static HybridAutoDefinition parse(String json, PathLoader loader) throws Exception {
		var auto = (JSONObject) new JSONParser().parse(json);
		if (!"2025.0".equals(auto.get("version")) || !Boolean.FALSE.equals(auto.get("resetOdom"))
				|| !Boolean.FALSE.equals(auto.get("choreoAuto")))
			throw new IllegalArgumentException("Hybrid auto requires version 2025.0 with resetOdom and choreoAuto off");
		var paths = new LinkedHashMap<String, PathPlannerPath>();
		var root = parseNode((JSONObject) auto.get("command"), loader, paths);
		if (paths.isEmpty())
			throw new IllegalArgumentException("No navigation objectives");
		return new HybridAutoDefinition(root, paths);
	}

	private static Node parseNode(JSONObject json, PathLoader loader, Map<String, PathPlannerPath> paths)
			throws Exception {
		var type = (String) json.get("type");
		var data = (JSONObject) json.get("data");
		var children = new ArrayList<Node>();
		String name = "";
		double seconds = 0;
		switch (type) {
			case "path", "named" -> {
				name = (String) data.get(type.equals("path") ? "pathName" : "name");
				if (name == null || name.isBlank())
					throw new IllegalArgumentException("Missing " + type + " name");
				if (type.equals("path") && !paths.containsKey(name))
					paths.put(name, loader.load(name));
			}
			case "wait" -> {
				seconds = ((Number) data.get("waitTime")).doubleValue();
				if (!Double.isFinite(seconds) || seconds < 0)
					throw new IllegalArgumentException("Invalid wait duration");
			}
			case "sequential", "parallel", "race", "deadline" -> {
				for (var child : (JSONArray) data.get("commands"))
					children.add(parseNode((JSONObject) child, loader, paths));
				long driving = children.stream().filter(Node::drives).count();
				if (!type.equals("sequential") && driving > 1)
					throw new IllegalArgumentException("Concurrent navigation objectives");
				if (type.equals("race") && driving > 0)
					throw new IllegalArgumentException("A race may skip physical arrival");
				if (type.equals("deadline") && (children.isEmpty() || (driving > 0 && !children.get(0).drives())))
					throw new IllegalArgumentException("Navigation must own its deadline");
			}
			default -> throw new IllegalArgumentException("Unsupported auto command: " + type);
		}
		return new Node(type, name, seconds, List.copyOf(children));
	}

	Map<String, PathPlannerPath> paths() {
		return java.util.Collections.unmodifiableMap(paths);
	}

	Command build(Function<String, Command> named, Function<String, Command> path) {
		return root.build(named, path);
	}

	/**
	 * Report every blocking path issue, rather than hiding later failures behind
	 * the first one.
	 */
	List<String> structuralIssues() {
		var issues = new ArrayList<String>();
		paths.forEach((name, path) -> {
			try {
				new AutoMissionPlan.Segment(name, path, Commands.none());
			} catch (IllegalArgumentException ex) {
				issues.add(name + ": " + ex.getMessage());
			}
		});
		return List.copyOf(issues);
	}

	List<String> clearanceIssues(NavigationSafety safety) {
		var issues = new ArrayList<String>();
		paths.forEach((name, path) -> {
			for (boolean red : new boolean[]{false, true}) {
				var variant = red && !path.preventFlipping ? path.flipPath() : path;
				String alliance = red ? "red" : "blue";
				if (!safety.safePoint(variant.getPoint(0).position))
					issues.add(name + ": " + alliance + " start lacks footprint clearance");
				if (!safety.safePoint(variant.getPoint(variant.numPoints() - 1).position))
					issues.add(name + ": " + alliance + " destination lacks footprint clearance");
				var points = variant.getAllPathPoints();
				for (int i = 1; i < points.size(); i++) {
					if (!safety.safeSegment(points.get(i - 1).position, points.get(i).position)) {
						issues.add(name + ": " + alliance + " authored corridor lacks footprint clearance");
						break;
					}
				}
			}
		});
		return List.copyOf(issues);
	}

	List<String> issues(NavigationSafety safety) {
		var issues = new ArrayList<>(structuralIssues());
		issues.addAll(clearanceIssues(safety));
		return List.copyOf(issues);
	}
}
