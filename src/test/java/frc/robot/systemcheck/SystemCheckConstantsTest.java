package frc.robot.systemcheck;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class SystemCheckConstantsTest {
	@Test
	void everyActiveOrObservationStageIsThreeToFiveSeconds() {
		double[] durations = {SystemCheckConstants.PREFLIGHT_SECONDS, SystemCheckConstants.SWERVE_STAGE_SECONDS,
				SystemCheckConstants.JOINT_STAGE_SECONDS, SystemCheckConstants.ROLLER_STAGE_SECONDS,
				SystemCheckConstants.HOOD_STAGE_SECONDS, SystemCheckConstants.SHOOTER_STAGE_SECONDS,
				SystemCheckConstants.VISION_STAGE_SECONDS, SystemCheckConstants.NEUTRAL_STAGE_SECONDS,
				SystemCheckConstants.PRACTICE_READY_SECONDS, SystemCheckConstants.PRACTICE_FEED_SECONDS};

		for (double duration : durations) {
			assertTrue(duration >= 3.0 && duration <= 5.0, () -> duration + " is outside the required range");
		}
	}

	@Test
	void normalSequenceTakesEightyFiveSecondsIncludingCountdown() {
		double total = SystemCheckConstants.COUNTDOWN_SECONDS + SystemCheckConstants.PREFLIGHT_SECONDS
				+ 5 * SystemCheckConstants.SWERVE_STAGE_SECONDS + 4 * SystemCheckConstants.JOINT_STAGE_SECONDS
				+ 4 * SystemCheckConstants.ROLLER_STAGE_SECONDS + 2 * SystemCheckConstants.HOOD_STAGE_SECONDS
				+ SystemCheckConstants.SHOOTER_STAGE_SECONDS + 2 * SystemCheckConstants.ROLLER_STAGE_SECONDS
				+ SystemCheckConstants.VISION_STAGE_SECONDS + SystemCheckConstants.NEUTRAL_STAGE_SECONDS;

		assertEquals(85.0, total, 1e-9);
		assertEquals(8.0, SystemCheckConstants.PRACTICE_TOTAL_SECONDS, 1e-9);
	}
}
