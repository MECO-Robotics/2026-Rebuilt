package frc.robot.systemcheck;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class DeviceHealthTest {
	@Test
	void internalEncoderDoesNotFailWhenNoExternalEncoderIsConfigured() {
		DeviceHealth health = health(new boolean[]{true}, new double[]{0.0}, new double[]{0.0}, new double[]{40.0},
				new double[]{25.0}, false, false);

		assertTrue(health.allConnected());
	}

	@Test
	void requiredExternalEncoderAndDisconnectedMotorFailConnectivity() {
		assertFalse(
				health(new boolean[]{true}, new double[]{}, new double[]{}, new double[]{}, new double[]{}, true, false)
						.allConnected());
		assertFalse(health(new boolean[]{false}, new double[]{}, new double[]{}, new double[]{}, new double[]{}, false,
				true).allConnected());
	}

	@Test
	void normalizedFollowersCompareMagnitudeAfterInversion() {
		DeviceHealth matching = health(new boolean[]{true, true}, new double[]{20.0, 16.0}, new double[]{},
				new double[]{}, new double[]{}, false, true);
		DeviceHealth mismatched = health(new boolean[]{true, true}, new double[]{20.0, 14.9}, new double[]{},
				new double[]{}, new double[]{}, false, true);

		assertTrue(matching.normalizedSpeedsAgree(0.25));
		assertFalse(mismatched.normalizedSpeedsAgree(0.25));
	}

	@Test
	void detectsTemperatureNonFiniteAndDebouncedStallInputs() {
		DeviceHealth stalled = health(new boolean[]{true}, new double[]{0.0}, new double[]{36.0}, new double[]{40.0},
				new double[]{Double.NaN}, false, true);

		assertTrue(stalled.hasNonFiniteFastMeasurement());
		assertTrue(stalled.hasStalledMotor(0.90, 0.01));

		DeviceHealth hot = health(new boolean[]{true}, new double[]{1.0}, new double[]{1.0}, new double[]{40.0},
				new double[]{85.0}, false, true);
		assertEquals(SystemCheckConstants.TEMPERATURE_ABORT_CELSIUS, hot.maximumTemperatureCelsius(), 1e-9);
	}

	@Test
	void deviceIdsAreRetainedAndDefensivelyCopied() {
		int[] source = {34, 35};
		DeviceHealth health = new DeviceHealth("Shooter", new boolean[]{true, true}, new double[]{}, new double[]{},
				new double[]{}, new double[]{}, false, true, new String[]{}, source);

		source[0] = 99;
		int[] returned = health.deviceIds();
		returned[1] = 88;
		assertArrayEquals(new int[]{34, 35}, health.deviceIds());
	}

	private static DeviceHealth health(boolean[] connections, double[] velocities, double[] currents,
			double[] currentLimits, double[] temperatures, boolean encoderExpected, boolean encoderConnected) {
		return new DeviceHealth("Test", connections, velocities, currents, currentLimits, temperatures, encoderExpected,
				encoderConnected, new String[]{});
	}
}
