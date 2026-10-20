package frc.robot.navigation;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Arrays;
import java.util.List;
import org.junit.jupiter.api.Test;

class PathTrackingMonitorTest {
	private static Pose2d p(double x, double y) {
		return new Pose2d(x, y, Rotation2d.kZero);
	}

	@Test
	void separatesGeometricErrorFromTimeReferenceLag() {
		var monitor = new PathTrackingMonitor(List.of(p(0, 0), p(10, 0)));
		var robot = monitor.project(new Translation2d(2, 0));
		var timedReference = monitor.project(new Translation2d(5, 0));
		assertEquals(0, robot.crossTrack(), 1e-9);
		assertEquals(3, timedReference.progress() - robot.progress(), 1e-9);
		var displaced = monitor.project(new Translation2d(2, 0.7));
		assertEquals(0.7, displaced.crossTrack(), 1e-9);
		assertEquals(robot.progress(), displaced.progress(), 1e-9);
	}

	@Test
	void findsCorrectEdgeAndAccumulatedDistanceAroundCorner() {
		var monitor = new PathTrackingMonitor(List.of(p(0, 0), p(3, 0), p(3, 4)));
		var projection = monitor.project(new Translation2d(3.4, 2));
		assertEquals(1, projection.edge());
		assertEquals(5, projection.progress(), 1e-9);
		assertEquals(0.4, projection.crossTrack(), 1e-9);
		assertEquals(new Translation2d(3, 2), projection.point());
		assertEquals(7, monitor.length(), 1e-9);
	}

	@Test
	void clampsBeforeStartAndAfterEndAndHandlesRepeatedVertices() {
		var monitor = new PathTrackingMonitor(List.of(p(0, 0), p(0, 0), p(2, 0)));
		var before = monitor.project(new Translation2d(-1, 0));
		var after = monitor.project(new Translation2d(3, 0));
		assertEquals(0, before.progress(), 1e-9);
		assertEquals(1, before.crossTrack(), 1e-9);
		assertEquals(2, after.progress(), 1e-9);
		assertEquals(1, after.crossTrack(), 1e-9);
		assertEquals(1, monitor.project(new Translation2d(1, 0)).progress(), 1e-9);
		assertThrows(UnsupportedOperationException.class, () -> monitor.poses().add(p(5, 0)));
	}

	@Test
	void rejectsInvalidGeometryInsteadOfInventingProgress() {
		assertThrows(IllegalArgumentException.class, () -> new PathTrackingMonitor(List.of(p(0, 0))));
		assertThrows(IllegalArgumentException.class, () -> new PathTrackingMonitor(null));
		assertThrows(IllegalArgumentException.class, () -> new PathTrackingMonitor(Arrays.asList(p(0, 0), null)));
		assertThrows(IllegalArgumentException.class, () -> new PathTrackingMonitor(List.of(p(0, 0), p(Double.NaN, 1))));
		var monitor = new PathTrackingMonitor(List.of(p(0, 0), p(2, 0)));
		assertThrows(IllegalArgumentException.class, () -> monitor.project(null));
		assertThrows(IllegalArgumentException.class, () -> monitor.project(new Translation2d(0, Double.NaN)));
		assertThrows(IllegalArgumentException.class,
				() -> monitor.project(new Translation2d(Double.POSITIVE_INFINITY, 0)));
	}
}
