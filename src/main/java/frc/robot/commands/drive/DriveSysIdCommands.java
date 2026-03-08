package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.SysIdResultsPublisher;
import frc.robot.util.SysIdRunStats;

/** Grouped SysId command factories for drivetrain drive and azimuth tuning. */
public class DriveSysIdCommands {
	private DriveSysIdCommands() {
	}

	public static Command driveQuasistatic(Drive drive, SysIdRoutine.Direction direction) {
		String testName = "Drive Quasistatic " + direction.name();
		final double[] appliedVolts = new double[]{0.0};
		return warmup(drive, drive::runCharacterization).andThen(
				withResults(createRoutine(drive, drive::runCharacterization, appliedVolts).quasistatic(direction),
						testName, () -> average(drive.getWheelRadiusCharacterizationPositions()),
						drive::getFFCharacterizationVelocity, appliedVolts));
	}

	public static Command driveDynamic(Drive drive, SysIdRoutine.Direction direction) {
		String testName = "Drive Dynamic " + direction.name();
		final double[] appliedVolts = new double[]{0.0};
		return warmup(drive, drive::runCharacterization)
				.andThen(withResults(createRoutine(drive, drive::runCharacterization, appliedVolts).dynamic(direction),
						testName, () -> average(drive.getWheelRadiusCharacterizationPositions()),
						drive::getFFCharacterizationVelocity, appliedVolts));
	}

	public static Command azimuthQuasistatic(Drive drive, SysIdRoutine.Direction direction) {
		String testName = "Azimuth Quasistatic " + direction.name();
		final double[] appliedVolts = new double[]{0.0};
		return warmup(drive, drive::runAzimuthCharacterization).andThen(withResults(
				createRoutine(drive, drive::runAzimuthCharacterization, appliedVolts).quasistatic(direction), testName,
				drive::getAverageAzimuthPositionRotations, drive::getAverageAzimuthVelocityRotationsPerSecond,
				appliedVolts));
	}

	public static Command azimuthDynamic(Drive drive, SysIdRoutine.Direction direction) {
		String testName = "Azimuth Dynamic " + direction.name();
		final double[] appliedVolts = new double[]{0.0};
		return warmup(drive, drive::runAzimuthCharacterization).andThen(
				withResults(createRoutine(drive, drive::runAzimuthCharacterization, appliedVolts).dynamic(direction),
						testName, drive::getAverageAzimuthPositionRotations,
						drive::getAverageAzimuthVelocityRotationsPerSecond, appliedVolts));
	}

	private static SysIdRoutine createRoutine(Drive drive, java.util.function.DoubleConsumer characterizationConsumer,
			double[] appliedVolts) {
		return new SysIdRoutine(new SysIdRoutine.Config(null, null, null, null),
				new SysIdRoutine.Mechanism((voltage) -> {
					appliedVolts[0] = voltage.in(Volts);
					characterizationConsumer.accept(appliedVolts[0]);
				}, null, drive));
	}

	private static Command withResults(Command sysIdCommand, String testName,
			java.util.function.DoubleSupplier positionSupplier, java.util.function.DoubleSupplier velocitySupplier,
			double[] appliedVolts) {
		SysIdRunStats stats = new SysIdRunStats(positionSupplier, velocitySupplier, appliedVolts);
		return sysIdCommand.deadlineFor(Commands.run(stats::sample)).beforeStarting(stats::start)
				.finallyDo((interrupted) -> {
					stats.finish();
					SysIdResultsPublisher.publish(testName, interrupted, stats);
				});
	}

	private static double average(double[] values) {
		if (values.length == 0) {
			return 0.0;
		}
		double sum = 0.0;
		for (int i = 0; i < values.length; i++) {
			sum += values[i];
		}
		return sum / values.length;
	}

	private static Command warmup(Drive drive, java.util.function.DoubleConsumer characterizationConsumer) {
		return drive.run(() -> characterizationConsumer.accept(0.0)).withTimeout(1.0);
	}
}
