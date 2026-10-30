package frc.robot.simulation;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;

class FuelTransportSimTest {
	@Test
	void stoppedReversedOrInvalidTransportCannotMoveFuel() {
		assertTrue(FuelTransportSim.allowsMotion(5, 10, 5));
		assertTrue(FuelTransportSim.allowsMotion(5, -10, 5)); // Inverted controller encoder.
		assertFalse(FuelTransportSim.allowsMotion(-5, 10, 5));
		assertFalse(FuelTransportSim.allowsMotion(0, 10, 5));
		assertFalse(FuelTransportSim.allowsMotion(5, 0, 5));
		assertFalse(FuelTransportSim.allowsMotion(5, Double.NaN, 5));
		assertFalse(FuelTransportSim.allowsMotion(5, 10, 0));
	}
}
