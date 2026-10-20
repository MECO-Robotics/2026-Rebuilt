package frc.robot.navigation;

import static org.junit.jupiter.api.Assertions.*;
import frc.robot.constants.Constants;
import frc.robot.simulation.HybridSimulationEnvironment;
import org.junit.jupiter.api.Test;

class HybridSimulationEnvironmentTest {
	@Test
	void optInCannotEnableTheFixtureOnHardwareOrReplay() {
		for (var mode : Constants.Mode.values()) {
			assertFalse(HybridSimulationEnvironment.allowed(mode, true, true));
			assertFalse(HybridSimulationEnvironment.allowed(mode, false, false));
		}
		assertFalse(HybridSimulationEnvironment.allowed(Constants.Mode.REAL, false, true));
		assertFalse(HybridSimulationEnvironment.allowed(Constants.Mode.REPLAY, false, true));
		assertTrue(HybridSimulationEnvironment.allowed(Constants.Mode.SIM, false, true));
	}
}
