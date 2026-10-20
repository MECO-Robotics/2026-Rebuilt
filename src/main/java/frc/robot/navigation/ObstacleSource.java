package frc.robot.navigation;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;

/**
 * Field-relative obstacle snapshots. Timestamps use the same monotonic clock as
 * the supervisor.
 */
@FunctionalInterface
public interface ObstacleSource {
	List<Observation> observations(double now);

	/**
	 * Bounds are in the blue-origin field frame; velocity is metres per second in
	 * that frame.
	 */
	record Observation(Translation2d min, Translation2d max, double timestamp, double confidence,
			Translation2d velocity) {
	}

	static ObstacleSource empty() {
		return now -> List.of();
	}

	/**
	 * Deterministic constant-velocity simulation actor; does not alter the robot
	 * pose estimator.
	 */
	static ObstacleSource moving(Translation2d min, Translation2d max, Translation2d velocity, double start,
			double end) {
		return now -> {
			if (now < start || now > end) {
				return List.of();
			}
			Translation2d displacement = velocity.times(now - start);
			return List.of(new Observation(min.plus(displacement), max.plus(displacement), now, 1.0, velocity));
		};
	}
}
