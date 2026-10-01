package frc.robot.commands.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.constants.subsystems.ShooterConstants;
import org.junit.jupiter.api.Test;

class ShooterCalibrationTest {
	@Test
	void clampsHoodRequestsToMechanicalLimits() {
		assertEquals(ShooterConstants.HOOD_GAINS.kMinPosition(), ShooterCommands.clampCalibrationHoodRotations(-1.0));
		assertEquals(0.031, ShooterCommands.clampCalibrationHoodRotations(0.031));
		assertEquals(ShooterConstants.HOOD_GAINS.kMaxPosition(), ShooterCommands.clampCalibrationHoodRotations(1.0));
		assertEquals(ShooterConstants.HOOD_GAINS.kMinPosition(),
				ShooterCommands.clampCalibrationHoodRotations(Double.NaN));
	}

	@Test
	void clampsFlywheelRequestsToSafeForwardRange() {
		assertEquals(0.0, ShooterCommands.clampCalibrationFlywheelRps(-100.0));
		assertEquals(37.0, ShooterCommands.clampCalibrationFlywheelRps(37.0));
		assertEquals(ShooterConstants.MAX_CALIBRATION_FLYWHEEL_RPS, ShooterCommands.clampCalibrationFlywheelRps(100.0));
		assertEquals(0.0, ShooterCommands.clampCalibrationFlywheelRps(Double.POSITIVE_INFINITY));
	}
}
