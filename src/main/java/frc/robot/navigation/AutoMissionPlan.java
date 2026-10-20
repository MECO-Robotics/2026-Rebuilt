package frc.robot.navigation;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;

/**
 * Explicit stopping objectives. Marker-bearing autos must be migrated
 * deliberately.
 */
public record AutoMissionPlan(List<Segment> segments) {
	public AutoMissionPlan {
		segments = List.copyOf(segments);
		if (segments.isEmpty())
			throw new IllegalArgumentException("Empty mission");
	}

	/**
	 * Action is run once, only after measured arrival; cancellation never counts as
	 * success.
	 */
	public record Segment(String name, PathPlannerPath path, Command arrivalAction) {
		public Segment {
			if (name == null || path == null || arrivalAction == null || !path.getEventMarkers().isEmpty()
					|| path.isChoreoPath() || path.getGoalEndState().velocityMPS() != 0) {
				throw new IllegalArgumentException("Hybrid segments require marker-free paths ending at rest");
			}
			if (selfCrossing(path))
				throw new IllegalArgumentException("Split self-crossing paths into unambiguous stopping segments");
		}
	}
	private static boolean selfCrossing(PathPlannerPath path) {
		var points = path.getAllPathPoints();
		for (int i = 0; i < points.size() - 1; i++)
			for (int j = i + 2; j < points.size() - 1; j++) {
				var a = points.get(i).position;
				var b = points.get(i + 1).position;
				var c = points.get(j).position;
				var d = points.get(j + 1).position;
				if (a.getDistance(b) < 1e-9 || c.getDistance(d) < 1e-9)
					continue;
				if (Math.max(a.getX(), b.getX()) < Math.min(c.getX(), d.getX()) - 1e-9
						|| Math.max(c.getX(), d.getX()) < Math.min(a.getX(), b.getX()) - 1e-9
						|| Math.max(a.getY(), b.getY()) < Math.min(c.getY(), d.getY()) - 1e-9
						|| Math.max(c.getY(), d.getY()) < Math.min(a.getY(), b.getY()) - 1e-9)
					continue;
				if (cross(b.minus(a), c.minus(a)) * cross(b.minus(a), d.minus(a)) <= 0
						&& cross(d.minus(c), a.minus(c)) * cross(d.minus(c), b.minus(c)) <= 0)
					return true;
			}
		return false;
	}
	private static double cross(Translation2d a, Translation2d b) {
		return a.getX() * b.getY() - a.getY() * b.getX();
	}
}
