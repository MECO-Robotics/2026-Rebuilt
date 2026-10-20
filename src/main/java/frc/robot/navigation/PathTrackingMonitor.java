package frc.robot.navigation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;

/**
 * Polyline projection separates physical cross-track error from time-reference
 * lag.
 */
public final class PathTrackingMonitor {
	public record Projection(double crossTrack, double progress, int edge, Translation2d point) {
	}
	private final List<Pose2d> poses;
	private final double[] distance;
	public PathTrackingMonitor(List<Pose2d> poses) {
		if (poses == null || poses.size() < 2)
			throw new IllegalArgumentException("Route needs two poses");
		for (var p : poses) {
			if (p == null || !finite(p.getTranslation()) || !Double.isFinite(p.getRotation().getRadians()))
				throw new IllegalArgumentException("Nonfinite route pose");
		}
		this.poses = List.copyOf(poses);
		distance = new double[poses.size()];
		for (int i = 1; i < poses.size(); i++)
			distance[i] = distance[i - 1]
					+ poses.get(i).getTranslation().getDistance(poses.get(i - 1).getTranslation());
	}
	public Projection project(Translation2d point) {
		if (!finite(point))
			throw new IllegalArgumentException("Nonfinite projection query");
		double best = Double.POSITIVE_INFINITY, progress = 0;
		int edge = 0;
		Translation2d nearest = poses.get(0).getTranslation();
		for (int i = 0; i < poses.size() - 1; i++) {
			var a = poses.get(i).getTranslation();
			var delta = poses.get(i + 1).getTranslation().minus(a);
			double length = delta.getNorm();
			double t = length < 1e-9 ? 0 : MathUtil.clamp(point.minus(a).dot(delta) / (length * length), 0, 1);
			var projected = a.plus(delta.times(t));
			double error = point.getDistance(projected);
			if (error < best) {
				best = error;
				edge = i;
				nearest = projected;
				progress = distance[i] + t * length;
			}
		}
		return new Projection(best, progress, edge, nearest);
	}
	private static boolean finite(Translation2d p) {
		return p != null && Double.isFinite(p.getX()) && Double.isFinite(p.getY());
	}
	public List<Pose2d> poses() {
		return poses;
	}
	public double length() {
		return distance[distance.length - 1];
	}
}
