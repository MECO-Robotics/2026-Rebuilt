package frc.robot.navigation;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.LogTable;

class ObstacleReplayTest {
	@Test
	void replaysGeometryVelocityTimestampAndConfidenceWithoutInventingFreshData() {
		var original = new LoggedObstacleSource.Inputs();
		original.values = List.of(new ObstacleSource.Observation(new Translation2d(2, 2), new Translation2d(3, 3), 10,
				.9, new Translation2d(.5, -.2)));
		var table = new LogTable(10_000_000);
		original.toLog(table);
		var replay = new LoggedObstacleSource.Inputs();
		replay.fromLog(table);
		assertEquals(original.values, replay.values);
		var safety = new NavigationSafety(16.54, 8.07, .65, List.of());
		safety.update(replay.values, 10.2);
		assertTrue(safety.healthy());
		safety.update(replay.values, 10.6);
		assertFalse(safety.healthy(), "Replay must preserve stale observation age");
	}
	@Test
	void missingOrCorruptRecordedSnapshotFailsClosedAndEmptyRemainsEmpty() {
		var replay = new LoggedObstacleSource.Inputs();
		replay.fromLog(new LogTable(0));
		assertNull(replay.values);
		var malformed = new LogTable(0);
		malformed.put("SnapshotAvailable", true);
		malformed.put("Observations", new double[]{1, 2, 3});
		replay.fromLog(malformed);
		assertNull(replay.values);
		var empty = new LoggedObstacleSource.Inputs();
		var table = new LogTable(0);
		empty.toLog(table);
		replay.fromLog(table);
		assertEquals(List.of(), replay.values);
	}
}
