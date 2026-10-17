package frc.robot.util.mechanical_advantage;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;

class LoggedTunableNumberTest {
	@Test
	void groupedInitialValuesApplyOnceWithoutRepeatingUnchangedCanConfigurations() {
		var one = new LoggedTunableNumber("GroupTest/One", 1);
		var two = new LoggedTunableNumber("GroupTest/Two", 2);
		var three = new LoggedTunableNumber("GroupTest/Three", 3);
		int[] count = {0};
		for (int i = 0; i < 5; i++) {
			LoggedTunableNumber.ifChanged(21, values -> {
				count[0]++;
				assertArrayEquals(new double[]{1, 2, 3}, values);
			}, one, two, three);
		}
		assertEquals(1, count[0]);
	}
}
