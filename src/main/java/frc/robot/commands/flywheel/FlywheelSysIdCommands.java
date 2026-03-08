package frc.robot.commands.flywheel;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.util.SysIdResultsPublisher;
import frc.robot.util.SysIdRunStats;
import java.util.function.DoubleConsumer;

/** Grouped SysId command factories for flywheel-style mechanisms. */
public class FlywheelSysIdCommands {
	private FlywheelSysIdCommands() {
	}

	public static Command quasistatic(Flywheel flywheel, SysIdRoutine.Direction direction) {
		String testName = flywheel.getName() + " Quasistatic " + direction.name();
		final double[] appliedVolts = new double[]{0.0};
		return warmup(flywheel).andThen(withResults(createRoutine(flywheel, appliedVolts).quasistatic(direction),
				flywheel, testName, appliedVolts));
	}

	public static Command dynamic(Flywheel flywheel, SysIdRoutine.Direction direction) {
		String testName = flywheel.getName() + " Dynamic " + direction.name();
		final double[] appliedVolts = new double[]{0.0};
		return warmup(flywheel).andThen(withResults(createRoutine(flywheel, appliedVolts).dynamic(direction), flywheel,
				testName, appliedVolts));
	}

	private static SysIdRoutine createRoutine(Flywheel flywheel, double[] appliedVolts) {
		DoubleConsumer characterizationConsumer = flywheel::setVoltage;
		return new SysIdRoutine(new SysIdRoutine.Config(null, null, null, null),
				new SysIdRoutine.Mechanism((voltage) -> {
					appliedVolts[0] = voltage.in(Volts);
					characterizationConsumer.accept(appliedVolts[0]);
				}, null, flywheel));
	}

	private static Command withResults(Command sysIdCommand, Flywheel flywheel, String testName,
			double[] appliedVolts) {
		SysIdRunStats stats = new SysIdRunStats(flywheel::getPosition, flywheel::getVelocity, appliedVolts);
		return sysIdCommand.deadlineFor(Commands.run(stats::sample)).beforeStarting(stats::start)
				.finallyDo((interrupted) -> {
					stats.finish();
					SysIdResultsPublisher.publish(testName, interrupted, stats);
				});
	}

	private static Command warmup(Flywheel flywheel) {
		return flywheel.run(() -> flywheel.setVoltage(0.0)).withTimeout(1.0);
	}
}
