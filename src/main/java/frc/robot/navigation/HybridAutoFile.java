package frc.robot.navigation;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.Filesystem;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.List;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

/**
 * Loads explicit stopping paths without silently discarding unsupported auto
 * commands.
 */
final class HybridAutoFile {
	private HybridAutoFile() {
	}

	static List<PathPlannerPath> load(String name) throws Exception {
		var file = Filesystem.getDeployDirectory().toPath().resolve("pathplanner/autos/" + name + ".auto");
		var names = pathNames(Files.readString(file));
		var paths = new ArrayList<PathPlannerPath>();
		for (var pathName : names)
			paths.add(PathPlannerPath.fromPathFile(pathName));
		return List.copyOf(paths);
	}

	static List<String> pathNames(String json) throws Exception {
		var auto = (JSONObject) new JSONParser().parse(json);
		if (!"2025.0".equals(auto.get("version")) || !Boolean.FALSE.equals(auto.get("resetOdom"))
				|| !Boolean.FALSE.equals(auto.get("choreoAuto")))
			throw new IllegalArgumentException(
					"Hybrid auto requires version 2025.0, resetOdom=false, choreoAuto=false");
		var names = new ArrayList<String>();
		collect((JSONObject) auto.get("command"), names);
		if (names.isEmpty())
			throw new IllegalArgumentException("Hybrid auto requires at least one stopping path");
		return List.copyOf(names);
	}

	private static void collect(JSONObject command, List<String> names) {
		var data = (JSONObject) command.get("data");
		switch ((String) command.get("type")) {
			case "sequential" -> {
				for (var child : (JSONArray) data.get("commands"))
					collect((JSONObject) child, names);
			}
			case "path" -> {
				var name = (String) data.get("pathName");
				if (name == null || name.isBlank())
					throw new IllegalArgumentException("Hybrid path is missing its name");
				names.add(name);
			}
			default -> throw new IllegalArgumentException(
					"Hybrid survey supports only sequential path commands; unsupported: " + command.get("type"));
		}
	}
}
