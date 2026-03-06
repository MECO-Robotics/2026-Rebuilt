package frc.robot.commands.flywheel;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.flywheel.Flywheel;
import java.util.function.DoubleConsumer;
import org.littletonrobotics.junction.Logger;

/** Grouped SysId command factories for flywheel-style mechanisms. */
public class FlywheelSysIdCommands {
	private FlywheelSysIdCommands() {
	}

	public static Command quasistatic(Flywheel flywheel, SysIdRoutine.Direction direction) {
		String stateLogKey = flywheel.getName() + "/SysIdState";
		return warmup(flywheel).andThen(createRoutine(flywheel, stateLogKey).quasistatic(direction));
	}

	public static Command dynamic(Flywheel flywheel, SysIdRoutine.Direction direction) {
		String stateLogKey = flywheel.getName() + "/SysIdState";
		return warmup(flywheel).andThen(createRoutine(flywheel, stateLogKey).dynamic(direction));
	}

	private static SysIdRoutine createRoutine(Flywheel flywheel, String stateLogKey) {
		DoubleConsumer characterizationConsumer = flywheel::setVoltage;
		return new SysIdRoutine(
				new SysIdRoutine.Config(null, null, null,
						(state) -> Logger.recordOutput(stateLogKey, state.toString())),
				new SysIdRoutine.Mechanism((voltage) -> characterizationConsumer.accept(voltage.in(Volts)), null,
						flywheel));
	}

	private static Command warmup(Flywheel flywheel) {
		return flywheel.run(() -> flywheel.setVoltage(0.0)).withTimeout(1.0);
	}
}
