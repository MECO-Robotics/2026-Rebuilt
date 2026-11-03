package frc.robot.simulation;

import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.Test;

class PracticeScoreCounterTest {
	@Test
	void countsHitsOnceAndKeepsAirborneAutoShotsInAutoAfterTransition() {
		var scores = new PracticeScoreCounter();
		Runnable airborneAuto = scores.onLaunch(true, false);
		Runnable teleop = scores.onLaunch(false, true);
		teleop.run();
		airborneAuto.run();
		airborneAuto.run();
		scores.onLaunch(false, false).run(); // Test/disabled does not count as teleop.
		scores.onLaunch(false, true); // A miss never invokes its callback.
		assertEquals(1, scores.autoCount());
		assertEquals(1, scores.teleopCount());
		assertEquals(2, scores.totalCount());
	}

	@Test
	void resetClearsBothModesAndRejectsPreviousSessionHits() {
		var scores = new PracticeScoreCounter();
		Runnable oldShot = scores.onLaunch(true, false);
		scores.onLaunch(false, true).run();
		scores.reset();
		oldShot.run();
		assertEquals(0, scores.autoCount());
		assertEquals(0, scores.teleopCount());
		assertEquals(0, scores.totalCount());
		scores.onLaunch(true, false).run();
		assertEquals(1, scores.totalCount());
	}
}
