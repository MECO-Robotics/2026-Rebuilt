package frc.robot.navigation;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class NavigationSafetyTest {
	private static Translation2d p(double x, double y) {
		return new Translation2d(x, y);
	}

	private static NavigationSafety empty() {
		return new NavigationSafety(10, 10, 0.5, List.of());
	}

	@Test
	void preventsShortcutsThroughInflatedObstaclesIncludingTangency() {
		var safety = new NavigationSafety(10, 10, 0.5, List.of(Pair.of(p(4, 4), p(5, 5))));
		assertFalse(safety.safeSegment(p(1, 4.5), p(8, 4.5)));
		assertFalse(safety.safeSegment(p(8, 4.5), p(1, 4.5)));
		assertFalse(safety.safeSegment(p(1, 3.5), p(8, 3.5)));
		assertFalse(safety.safePoint(p(4.5, 4.5)));
		assertTrue(safety.safeSegment(p(1, 3.49), p(8, 3.49)));
		assertFalse(safety.safeSegment(p(2, 2), p(7, 7)));
	}

	@Test
	void enforcesFootprintAtFieldBoundaryAndRejectsNan() {
		var safety = empty();
		assertFalse(safety.safePoint(p(0.49, 2)));
		assertFalse(safety.safePoint(p(9.51, 2)));
		assertTrue(safety.safePoint(p(0.5, 2)));
		assertFalse(safety.safePoint(p(Double.NaN, 2)));
		assertFalse(safety.safePoint(null));
		assertThrows(IllegalArgumentException.class, () -> new NavigationSafety(10, 10, Double.NaN, List.of()));
	}

	@Test
	void inflatesMovingObservationForLatencyAndFutureTravel() {
		var safety = empty();
		safety.update(List.of(new ObstacleSource.Observation(p(2, 2), p(3, 3), 1, 1, p(2, -1))), 1.25);
		assertTrue(safety.healthy());
		assertEquals(p(1.4, 0.65), safety.dynamicObstacles().get(0).getFirst());
		assertEquals(p(5.1, 3.6), safety.dynamicObstacles().get(0).getSecond());
		assertFalse(safety.safePoint(p(4.9, 2)));
		assertTrue(safety.safePoint(p(6, 2)));
		assertEquals(5, safety.plannerObstacles().size());
	}

	@Test
	void staleFutureUncertainAndInvalidFramesFailClosedUntilNewSnapshot() {
		var safety = empty();
		for (var observation : List.of(new ObstacleSource.Observation(p(2, 2), p(3, 3), 0, 1, p(0, 0)),
				new ObstacleSource.Observation(p(2, 2), p(3, 3), 2, 1, p(0, 0)),
				new ObstacleSource.Observation(p(2, 2), p(3, 3), 1, 0.3, p(0, 0)),
				new ObstacleSource.Observation(p(-2, 2), p(3, 3), 1, 1, p(0, 0)),
				new ObstacleSource.Observation(p(4, 2), p(3, 3), 1, 1, p(0, 0)),
				new ObstacleSource.Observation(p(2, 2), p(3, 3), 1, 1, p(Double.NaN, 0)))) {
			safety.update(List.of(observation), 1);
			assertFalse(safety.healthy());
			assertFalse(safety.safePoint(p(8, 8)));
			safety.update(List.of(), 1);
			assertTrue(safety.healthy());
			assertTrue(safety.safePoint(p(8, 8)));
		}
		safety.update(null, 1);
		assertFalse(safety.healthy());
	}

	@Test
	void readsBlockedCellsAndRejectsMissingNavgrid(@TempDir Path directory) throws IOException {
		Path navgrid = directory.resolve("navgrid.json");
		Files.writeString(navgrid, """
				{"field_size":{"x":3,"y":2},"nodeSizeMeters":1,
				"grid":[[false,true,false],[false,false,false]]}
				""");
		var safety = NavigationSafety.fromNavgrid(navgrid, 0.1);
		assertFalse(safety.safeSegment(p(0.5, 0.5), p(2.5, 0.5)));
		assertTrue(safety.safeSegment(p(0.5, 1.5), p(2.5, 1.5)));
		assertThrows(IOException.class, () -> NavigationSafety.fromNavgrid(directory.resolve("absent"), 0.1));
		Files.writeString(navgrid, "{\"field_size\":{\"x\":3,\"y\":2},\"nodeSizeMeters\":1,\"grid\":[]}");
		assertThrows(IOException.class, () -> NavigationSafety.fromNavgrid(navgrid, 0.1));
	}

	@Test
	void deterministicActorMovesContinuouslyWithFreshTimestamps() {
		var source = ObstacleSource.moving(p(2, 2), p(3, 3), p(1, -0.5), 1, 5);
		assertTrue(source.observations(0).isEmpty());
		assertEquals(p(3, 1.5), source.observations(2).get(0).min());
		assertEquals(2, source.observations(2).get(0).timestamp());
		assertEquals(p(1, -0.5), source.observations(2).get(0).velocity());
		assertTrue(source.observations(6).isEmpty());
		assertTrue(ObstacleSource.empty().observations(2).isEmpty());
	}
}
