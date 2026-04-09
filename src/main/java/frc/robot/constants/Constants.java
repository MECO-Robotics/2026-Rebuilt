package frc.robot.constants;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.Optional;
import java.util.function.DoubleSupplier;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always
 * "real" when running on a roboRIO. Change the value of "simMode" to switch
 * between "sim" (physics sim) and "replay" (log replay from a file).
 */
public final class Constants {
	public static final Mode simMode = Mode.SIM;
	public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

	public static final DoubleSupplier kZERO_SUPPLY = () -> 0.0;
	public static final boolean ENABLE_DRIVETRAIN_SYSID_AUTOS = false;

	public static boolean tuningMode = true;

	public static enum Mode {
		/** Running on a real robot. */
		REAL,

		/** Running a physics simulator. */
		SIM,

		/** Replaying from a log file. */
		REPLAY
	}

	public static boolean isAllianceRed() {
		Optional<Alliance> alliance = DriverStation.getAlliance();
		boolean yes = false;
		if (alliance.isPresent()) {
			if (alliance.get() == Alliance.Red) {
				yes = true;
			}
			if (alliance.get() == Alliance.Blue) {
				yes = false;
			}
		}
		return yes;
	}
}
