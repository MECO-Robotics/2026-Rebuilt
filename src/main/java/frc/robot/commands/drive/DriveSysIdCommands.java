package frc.robot.commands.drive;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

/** Grouped SysId command factories for drivetrain drive and azimuth tuning. */
public class DriveSysIdCommands {
	private DriveSysIdCommands() {
	}

	public static Command driveQuasistatic(Drive drive, SysIdRoutine.Direction direction) {
		return warmup(drive, drive::runCharacterization).andThen(
				createRoutine(drive, "Drive/SysId/DriveState", drive::runCharacterization).quasistatic(direction));
	}

	public static Command driveDynamic(Drive drive, SysIdRoutine.Direction direction) {
		return warmup(drive, drive::runCharacterization)
				.andThen(createRoutine(drive, "Drive/SysId/DriveState", drive::runCharacterization).dynamic(direction));
	}

	public static Command azimuthQuasistatic(Drive drive, SysIdRoutine.Direction direction) {
		return warmup(drive, drive::runAzimuthCharacterization)
				.andThen(createRoutine(drive, "Drive/SysId/AzimuthState", drive::runAzimuthCharacterization)
						.quasistatic(direction));
	}

	public static Command azimuthDynamic(Drive drive, SysIdRoutine.Direction direction) {
		return warmup(drive, drive::runAzimuthCharacterization).andThen(
				createRoutine(drive, "Drive/SysId/AzimuthState", drive::runAzimuthCharacterization).dynamic(direction));
	}

	private static SysIdRoutine createRoutine(Drive drive, String stateLogKey,
			java.util.function.DoubleConsumer characterizationConsumer) {
		return new SysIdRoutine(
				new SysIdRoutine.Config(null, null, null,
						(state) -> Logger.recordOutput(stateLogKey, state.toString())),
				new SysIdRoutine.Mechanism((voltage) -> characterizationConsumer.accept(voltage.in(Volts)), null,
						drive));
	}

	private static Command warmup(Drive drive, java.util.function.DoubleConsumer characterizationConsumer) {
		return drive.run(() -> characterizationConsumer.accept(0.0)).withTimeout(1.0);
	}
}
