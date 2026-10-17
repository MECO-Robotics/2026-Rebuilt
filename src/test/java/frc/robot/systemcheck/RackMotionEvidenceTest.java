package frc.robot.systemcheck;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;

class RackMotionEvidenceTest {
	@Test
	void startupZeroCannotProveDeployMotionAndReturnCanAlreadyBeStowed() {
		assertTrue(SystemCheckManager.rackMovementProven("intake-rack-stow", 0, 0, 0, .02));
		assertFalse(SystemCheckManager.rackMovementProven("intake-rack-safe", .13, 0, .13, .02));
		assertFalse(SystemCheckManager.rackMovementProven("intake-rack-deploy", .30, 0, .30, .02));
		assertFalse(SystemCheckManager.rackMovementProven("intake-rack-deploy", 0, .005, .30, .02));
		assertTrue(SystemCheckManager.rackMovementProven("intake-rack-safe", 0, .12, .13, .02));
		assertTrue(SystemCheckManager.rackMovementProven("intake-rack-deploy", .13, .16, .30, .02));
		assertTrue(SystemCheckManager.rackMovementProven("intake-rack-return", 0, 0, 0, .02));
		assertFalse(SystemCheckManager.rackMovementProven("intake-rack-return", .3, 0, 0, .02));
		assertFalse(SystemCheckManager.rackMovementProven("intake-rack-safe", Double.NaN, .13, .13, .02));
	}
}
