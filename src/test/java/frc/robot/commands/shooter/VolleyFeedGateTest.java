package frc.robot.commands.shooter;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import org.junit.jupiter.api.Test;

class VolleyFeedGateTest {
	@Test
	void waitsForStableSpeedThenLatchesUntilReleaseAndRearms() {
		assertTrue(HAL.initialize(500, 0));
		SimHooks.pauseTiming();
		try {
			var gate = new VolleyFeedGate();
			gate.reset();
			assertFalse(gate.calculate(true, 0, 37));
			SimHooks.stepTiming(.2);
			assertFalse(gate.calculate(true, 30, 37));
			SimHooks.stepTiming(.05);
			assertFalse(gate.calculate(true, 37, 37));
			SimHooks.stepTiming(.11);
			assertTrue(gate.calculate(true, 37, 37));
			assertTrue(gate.calculate(true, 30, 37), "Speed sag must not interrupt an active volley");
			assertFalse(gate.calculate(false, 30, 37));
			assertFalse(gate.calculate(true, 30, 37), "New press must recheck speed");
			SimHooks.stepTiming(.2);
			assertTrue(gate.calculate(true, 37, 37));
			gate.reset();
			assertFalse(gate.calculate(true, 37, 37), "Command restart clears the latch and debounce");
			assertFalse(gate.calculate(true, Double.NaN, 37));
			assertFalse(gate.calculate(true, 0, 0));
		} finally {
			SimHooks.resumeTiming();
		}
	}
}
