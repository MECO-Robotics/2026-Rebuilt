package frc.robot.navigation;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

/**
 * Conservative swept-disc clearance checks against the deployed navgrid and
 * observed obstacles. Call on the robot main thread. A rejected snapshot makes
 * every clearance check fail closed.
 */
public final class NavigationSafety {
	public static final double MAX_OBSERVATION_AGE = 0.5;
	public static final double PREDICTION_SECONDS = 0.5;
	public static final double MIN_CONFIDENCE = 0.7;
	private static final double UNCERTAINTY_METERS = 0.1;
	private final double width;
	private final double height;
	private final double inflation;
	private final List<Pair<Translation2d, Translation2d>> staticObstacles;
	private List<Pair<Translation2d, Translation2d>> dynamicObstacles = List.of();
	private boolean healthy = true;

	public NavigationSafety(double width, double height, double inflation,
			List<Pair<Translation2d, Translation2d>> staticBounds) {
		if (!Double.isFinite(width) || !Double.isFinite(height) || !Double.isFinite(inflation) || inflation < 0
				|| width <= 2 * inflation || height <= 2 * inflation || staticBounds == null) {
			throw new IllegalArgumentException("Invalid field dimensions or footprint");
		}
		this.width = width;
		this.height = height;
		this.inflation = inflation;
		List<Pair<Translation2d, Translation2d>> inflated = new ArrayList<>();
		for (var box : staticBounds) {
			if (box == null || !validBounds(box.getFirst(), box.getSecond())) {
				throw new IllegalArgumentException("Invalid static obstacle");
			}
			inflated.add(expand(box.getFirst(), box.getSecond(), inflation));
		}
		staticObstacles = List.copyOf(inflated);
	}

	public static NavigationSafety fromDeploy(double inflation) throws IOException {
		return fromNavgrid(Filesystem.getDeployDirectory().toPath().resolve("pathplanner/navgrid.json"), inflation);
	}

	public static NavigationSafety fromNavgrid(Path path, double inflation) throws IOException {
		try {
			JsonNode root = new ObjectMapper().readTree(path.toFile());
			double width = number(root.path("field_size").path("x"));
			double height = number(root.path("field_size").path("y"));
			double cell = number(root.path("nodeSizeMeters"));
			if (cell <= 0 || width <= 0 || height <= 0 || width / cell > 10000 || height / cell > 10000) {
				throw new IllegalArgumentException("Invalid navgrid dimensions");
			}
			JsonNode grid = root.path("grid");
			int rows = (int) Math.ceil(height / cell);
			int columns = (int) Math.ceil(width / cell);
			if (!grid.isArray() || grid.size() != rows) {
				throw new IllegalArgumentException("Incomplete navgrid");
			}
			List<Pair<Translation2d, Translation2d>> bounds = new ArrayList<>();
			for (int y = 0; y < rows; y++) {
				JsonNode row = grid.get(y);
				if (!row.isArray() || row.size() != columns) {
					throw new IllegalArgumentException("Incomplete navgrid row");
				}
				int run = -1;
				for (int x = 0; x <= columns; x++) {
					if (x < columns && !row.get(x).isBoolean()) {
						throw new IllegalArgumentException("Invalid navgrid cell");
					}
					boolean blocked = x < columns && row.get(x).booleanValue();
					if (blocked && run < 0) {
						run = x;
					} else if (!blocked && run >= 0) {
						bounds.add(Pair.of(new Translation2d(run * cell, y * cell),
								new Translation2d(Math.min(width, x * cell), Math.min(height, (y + 1) * cell))));
						run = -1;
					}
				}
			}
			return new NavigationSafety(width, height, inflation, bounds);
		} catch (IllegalArgumentException | NullPointerException ex) {
			throw new IOException("Unsafe or invalid navgrid: " + path, ex);
		}
	}

	private static double number(JsonNode value) {
		if (!value.isNumber() || !Double.isFinite(value.doubleValue())) {
			throw new IllegalArgumentException("Missing or invalid navgrid number");
		}
		return value.doubleValue();
	}

	/**
	 * Replaces a complete sensor snapshot. Empty explicitly means no detections;
	 * null means failure. Stale, uncertain, impossible, or malformed observations
	 * invalidate the entire snapshot.
	 */
	public void update(List<ObstacleSource.Observation> observations, double now) {
		healthy = false;
		if (observations == null || !Double.isFinite(now) || observations.size() > 64) {
			return;
		}
		List<Pair<Translation2d, Translation2d>> next = new ArrayList<>();
		for (var observation : observations) {
			if (observation == null || !validBounds(observation.min(), observation.max())
					|| !finite(observation.velocity()) || observation.velocity().getNorm() > 15
					|| !Double.isFinite(observation.timestamp()) || !Double.isFinite(observation.confidence())
					|| observation.confidence() < MIN_CONFIDENCE || observation.confidence() > 1
					|| now < observation.timestamp() || now - observation.timestamp() > MAX_OBSERVATION_AGE
					|| observation.min().getX() < 0 || observation.min().getY() < 0 || observation.max().getX() > width
					|| observation.max().getY() > height) {
				return;
			}
			// Include the original box, latency displacement, and a near-future moving
			// envelope.
			Translation2d travel = observation.velocity().times(now - observation.timestamp() + PREDICTION_SECONDS);
			Translation2d min = new Translation2d(observation.min().getX() + Math.min(0, travel.getX()),
					observation.min().getY() + Math.min(0, travel.getY()));
			Translation2d max = new Translation2d(observation.max().getX() + Math.max(0, travel.getX()),
					observation.max().getY() + Math.max(0, travel.getY()));
			next.add(expand(min, max, inflation + UNCERTAINTY_METERS));
		}
		dynamicObstacles = List.copyOf(next);
		healthy = true;
	}

	public boolean healthy() {
		return healthy;
	}

	/**
	 * Inflated bounds for Pathfinding.setDynamicObstacles, already in the shared
	 * field frame.
	 */
	public List<Pair<Translation2d, Translation2d>> dynamicObstacles() {
		return dynamicObstacles;
	}

	/**
	 * Static footprint clearance and moving envelopes for the native point-robot
	 * planner.
	 */
	public List<Pair<Translation2d, Translation2d>> plannerObstacles() {
		List<Pair<Translation2d, Translation2d>> combined = new ArrayList<>(staticObstacles);
		combined.addAll(dynamicObstacles);
		// The navgrid need not contain boundary cells. Prevent routes closer than the
		// footprint.
		combined.add(Pair.of(new Translation2d(0, 0), new Translation2d(width, inflation)));
		combined.add(Pair.of(new Translation2d(0, height - inflation), new Translation2d(width, height)));
		combined.add(Pair.of(new Translation2d(0, 0), new Translation2d(inflation, height)));
		combined.add(Pair.of(new Translation2d(width - inflation, 0), new Translation2d(width, height)));
		return List.copyOf(combined);
	}

	public boolean safePoint(Translation2d point) {
		return safeSegment(point, point);
	}

	public boolean safeSegment(Translation2d a, Translation2d b) {
		if (!healthy || !inField(a) || !inField(b)) {
			return false;
		}
		for (var bounds : staticObstacles) {
			if (intersects(a, b, bounds)) {
				return false;
			}
		}
		for (var bounds : dynamicObstacles) {
			if (intersects(a, b, bounds)) {
				return false;
			}
		}
		return true;
	}

	private boolean inField(Translation2d p) {
		return finite(p) && p.getX() >= inflation && p.getX() <= width - inflation && p.getY() >= inflation
				&& p.getY() <= height - inflation;
	}

	private static boolean validBounds(Translation2d min, Translation2d max) {
		return finite(min) && finite(max) && min.getX() <= max.getX() && min.getY() <= max.getY();
	}

	private static boolean finite(Translation2d p) {
		return p != null && Double.isFinite(p.getX()) && Double.isFinite(p.getY());
	}

	private static Pair<Translation2d, Translation2d> expand(Translation2d min, Translation2d max, double margin) {
		Translation2d offset = new Translation2d(margin, margin);
		return Pair.of(min.minus(offset), max.plus(offset));
	}

	/**
	 * Slab intersection: includes tangency and zero-length segments, without
	 * sampling gaps.
	 */
	private static boolean intersects(Translation2d a, Translation2d b, Pair<Translation2d, Translation2d> bounds) {
		double low = 0;
		double high = 1;
		for (int axis = 0; axis < 2; axis++) {
			double start = axis == 0 ? a.getX() : a.getY();
			double delta = axis == 0 ? b.getX() - start : b.getY() - start;
			double min = axis == 0 ? bounds.getFirst().getX() : bounds.getFirst().getY();
			double max = axis == 0 ? bounds.getSecond().getX() : bounds.getSecond().getY();
			if (Math.abs(delta) < 1e-12) {
				if (start < min || start > max) {
					return false;
				}
			} else {
				double first = (min - start) / delta;
				double second = (max - start) / delta;
				low = Math.max(low, Math.min(first, second));
				high = Math.min(high, Math.max(first, second));
				if (low > high) {
					return false;
				}
			}
		}
		return true;
	}
}
