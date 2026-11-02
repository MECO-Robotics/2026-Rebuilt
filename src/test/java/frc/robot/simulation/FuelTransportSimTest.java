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

	@Test
	void eachShootingCommandRequiresItsOwnSignedRequests() {
		for (var direction : frc.robot.commands.shooter.IndexerFeedDirection.values()) {
			assertTrue(FuelTransportSim.allowsMotion(direction.bottomVolts(), 20, direction.bottomVolts()));
			assertTrue(FuelTransportSim.allowsMotion(direction.topVolts(), -20, direction.topVolts()));
			assertFalse(FuelTransportSim.allowsMotion(-direction.bottomVolts(), 20, direction.bottomVolts()));
			assertFalse(FuelTransportSim.allowsMotion(-direction.topVolts(), 20, direction.topVolts()));
		}
		assertEquals(-frc.robot.commands.shooter.IndexerFeedDirection.MANUAL.bottomVolts(),
				frc.robot.commands.shooter.IndexerFeedDirection.COORDINATED.bottomVolts());
	}
}
