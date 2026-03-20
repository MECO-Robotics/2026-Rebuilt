package frc.robot.commands.shooter;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.flywheel.FlywheelVelocityCommand;
import frc.robot.commands.flywheel.FlywheelVoltageCommand;
import frc.robot.commands.position_joint.PositionJointPositionCommand;
// import frc.robot.simulation.LaunchedFuelSim;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.position_joint.PositionJoint;
import frc.robot.util.mechanical_advantage.LoggedTunableNumber;

/** Factory methods for coordinated shooter/indexer/conveyor command groups. */
public class ShooterCommands {
	// private static LaunchedFuelSim launchedFuelSimulation;

	// public static void setLaunchedFuelSimulation(LaunchedFuelSim sim) {
	// launchedFuelSimulation = sim;
	// }

	/** Conveyor roller preset voltages. */
	public final class CONVEYOR_VOLTS {
		public static final LoggedTunableNumber FEED = new LoggedTunableNumber("ConveyorVolts/IntakeSpeed", -10);
		public static final LoggedTunableNumber SLOW = new LoggedTunableNumber("ConveyorVolts/Slow", -1.5);
		public static final LoggedTunableNumber EJECT = new LoggedTunableNumber("ConveyorVolts/Eject", 12);
		public static final LoggedTunableNumber STOP = new LoggedTunableNumber("ConveyorVolts/Stop", 0);
	}

	/** Intake rotation preset positions. */
	public static final class HOOD_POSITIONS {
		public static final LoggedTunableNumber STOW = new LoggedTunableNumber("IntakePosition/Stow", 0);
		public static final LoggedTunableNumber FENDERHOOD = new LoggedTunableNumber("Fender", 0.001);
		public static final LoggedTunableNumber FERRYHOOD = new LoggedTunableNumber("Ferry", 0.049);
	}

	/** Indexer roller preset voltages. */
	public final class INDEXER_VOLTS {
		public static final LoggedTunableNumber FEED = new LoggedTunableNumber("IndexerVolts/IntakeSpeed", -7);
		public static final LoggedTunableNumber FEEDOTHER = new LoggedTunableNumber("IndexerVolts/IntakeSpeedOther", 7);
		public static final LoggedTunableNumber SLOW = new LoggedTunableNumber("IndexerVolts/Slow", -4);
		public static final LoggedTunableNumber EJECT = new LoggedTunableNumber("IndexerVolts/Eject", 12);
		public static final LoggedTunableNumber STOP = new LoggedTunableNumber("IndexerVolts/Stop", 0);
		public static final LoggedTunableNumber FERREYVELOC = new LoggedTunableNumber("Ferry Velocity", 35);

	}

	/**
	 * Shooter roller preset voltages. (NOTE: MAINLY FOR TESTING/SHUTTLE (maybe))
	 */
	public final class SHOOTER_VOLTS {
		public static final LoggedTunableNumber SHOOT = new LoggedTunableNumber("ShooterVolts/Shoot", -11);
		public static final LoggedTunableNumber SLOW = new LoggedTunableNumber("ShooterVolts/Slow", -1.5);
		public static final LoggedTunableNumber EJECT = new LoggedTunableNumber("ShooterVolts/Eject", 12);
		public static final LoggedTunableNumber STOP = new LoggedTunableNumber("ShooterVolts/Stop", 0);
		public static final LoggedTunableNumber FENDERFLYWHEEL = new LoggedTunableNumber("Fender Hood", 30);
	}

	/**
	 * Puts the Shooter and bottom indexers in a slow idle speed, and stopping the
	 * top indexer
	 */
	public static Command idleRollers(Flywheel bottomIntakingRoller, Flywheel topIntakingRoller,
			Flywheel conveyorRoller) {
		return Commands.parallel(new FlywheelVoltageCommand(bottomIntakingRoller, INDEXER_VOLTS.STOP),
				new FlywheelVoltageCommand(topIntakingRoller, INDEXER_VOLTS.STOP),
				new FlywheelVoltageCommand(conveyorRoller, CONVEYOR_VOLTS.STOP));
	}

	/** Puts both indexers feeding towards the shooter */
	public static Command feedRollers(Flywheel bottomIntakingRoller, Flywheel topIntakingRoller,
			Flywheel conveyorRoller) {
		return Commands.parallel(new FlywheelVoltageCommand(bottomIntakingRoller, INDEXER_VOLTS.FEED),
				new FlywheelVoltageCommand(topIntakingRoller, INDEXER_VOLTS.FEEDOTHER),
				new FlywheelVoltageCommand(conveyorRoller, CONVEYOR_VOLTS.FEED));
		// launchedFuelSimulation != null ? launchedFuelSimulation.launchCommand() :
		// Commands.none());
	}

	public static Command stopShooting(Flywheel shooterRoller, PositionJoint hood) {

		return Commands.parallel(new FlywheelVoltageCommand(shooterRoller, SHOOTER_VOLTS.STOP),
				new PositionJointPositionCommand(hood, HOOD_POSITIONS.STOW));
	}

	public static Command fender(Flywheel shooterRoller, PositionJoint hood) {
		return Commands.parallel(new PositionJointPositionCommand(hood, HOOD_POSITIONS.FENDERHOOD),
				new FlywheelVelocityCommand(shooterRoller, SHOOTER_VOLTS.FENDERFLYWHEEL));
	}

	public static Command off(Flywheel shooterRoller, PositionJoint hood) {
		return Commands.parallel(new PositionJointPositionCommand(hood, HOOD_POSITIONS.STOW),
				new FlywheelVelocityCommand(shooterRoller, SHOOTER_VOLTS.STOP));
	}
}
