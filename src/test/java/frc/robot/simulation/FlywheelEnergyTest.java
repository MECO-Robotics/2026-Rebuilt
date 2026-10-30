package frc.robot.simulation;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;

class FlywheelEnergyTest {
	@Test
	void conservesEnergyAndCannotReverseOrCreateWheelMotion() {
		double after = FlywheelEnergy.afterShot(200, 0.007, 5);
		assertEquals(5, 0.5 * 0.007 * (200 * 200 - after * after), 1e-9);
		assertTrue(after < 200 && after > 0);
		assertEquals(-after, FlywheelEnergy.afterShot(-200, 0.007, 5), 1e-9);
		assertEquals(0, FlywheelEnergy.afterShot(10, 0.007, 100), 1e-9);
		assertEquals(0, FlywheelEnergy.afterShot(0, 0.007, 5), 1e-9);
	}
}
