package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.subsystems.IntakeConstants;
import frc.robot.subsystems.position_joint.PositionJoint;
import frc.robot.subsystems.position_joint.PositionJointIO;
import org.junit.jupiter.api.Test;

class ShotIntakeSettingsTest {
	@Test
	void boundsRejectNonfiniteValuesAndExcessiveRetraction() {
		assertEquals(new ShotIntakeSettings(.1, .13), new ShotIntakeSettings(Double.NaN, Double.POSITIVE_INFINITY));
		assertEquals(new ShotIntakeSettings(.01, .13), new ShotIntakeSettings(-1, -.2));
		assertEquals(new ShotIntakeSettings(.5, .3), new ShotIntakeSettings(10, 10));
	}

	@Test
	void savedSettingsApplyNextVolleyAndReleaseRestoresNormalProfile() {
		assertTrue(HAL.initialize(500, 0));
		class IO implements PositionJointIO {
			double target, speed;
			public String getName() {
				return "ShotIntakeTest";
			}
			public void setPosition(double p, double v) {
				target = p;
			}
			public boolean setPositionDynamic(double p, double v, double a) {
				target = p;
				speed = v;
				return true;
			}
		}
		var io = new IO();
		var rack = new PositionJoint(io, IntakeConstants.INTAKE_RACK_GAINS);
		boolean[] feeding = {true};
		var command = IntakeCommands.holdRackRetractedWhileFeeding(rack, () -> feeding[0]);
		try {
			Preferences.setDouble(ShotIntakeSettings.SPEED_KEY, .08);
			Preferences.setDouble(ShotIntakeSettings.POSITION_KEY, .20);
			ShotIntakeSettings.initialize(); // Reinitializing must preserve saved values.
			assertEquals(new ShotIntakeSettings(.08, .20), ShotIntakeSettings.load());
			rack.periodic();
			command.initialize();
			command.execute();
			rack.periodic();
			assertEquals(.08, io.speed);
			assertEquals(.20, io.target);
			Preferences.setDouble(ShotIntakeSettings.SPEED_KEY, .06);
			Preferences.setDouble(ShotIntakeSettings.POSITION_KEY, .22);
			command.execute();
			rack.periodic();
			assertEquals(.08, io.speed);
			assertEquals(.20, io.target);
			feeding[0] = false;
			command.execute();
			rack.periodic();
			assertEquals(.5, io.speed);
			assertEquals(.30, io.target);
			feeding[0] = true;
			command.execute();
			rack.periodic();
			assertEquals(.06, io.speed);
			assertEquals(.22, io.target);
			command.end(true);
			assertEquals(0, rack.getCommandedVoltage());
		} finally {
			Preferences.remove(ShotIntakeSettings.SPEED_KEY);
			Preferences.remove(ShotIntakeSettings.POSITION_KEY);
			CommandScheduler.getInstance().unregisterSubsystem(rack);
		}
	}
}
