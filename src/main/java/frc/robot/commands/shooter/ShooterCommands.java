package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.subsystems.ShooterConstants.CONVEYOR_PRESET;
import frc.robot.constants.subsystems.ShooterConstants.HOOD_PRESET;
import frc.robot.constants.subsystems.ShooterConstants.INDEXER_PRESET;
import frc.robot.constants.subsystems.ShooterConstants.SHOOTER_PRESET;
import frc.robot.simulation.LaunchedFuelSim;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.position_joint.PositionJoint;

/** Factory methods for coordinated shooter/indexer/conveyor command groups. */
public class ShooterCommands {
	private static LaunchedFuelSim launchedFuelSimulation;

	/** Registers the sim hook that spawns launched fuel during feed commands. */
	public static void setLaunchedFuelSimulation(LaunchedFuelSim sim) {
		launchedFuelSimulation = sim;
	}

	/**
	 * Idles both indexers and the conveyor without spinning the shooter flywheel.
	 */
	public static Command idleRollers(Flywheel bottomIntakingRoller, Flywheel topIntakingRoller,
			Flywheel conveyorRoller) {
		return Commands.parallel(Flywheel.setVoltage(bottomIntakingRoller, INDEXER_PRESET.IDLE_BOTTOM),
				Flywheel.setVoltage(topIntakingRoller, INDEXER_PRESET.IDLE_BOTTOM),
				Flywheel.setVoltage(conveyorRoller, CONVEYOR_PRESET.IDLE));
	}

	/** Idles both indexers without touching the conveyor. */
	public static Command idleRollers(Flywheel bottomIntakingRoller, Flywheel topIntakingRoller) {
		return Commands.parallel(Flywheel.setVoltage(bottomIntakingRoller, INDEXER_PRESET.IDLE_BOTTOM),
				Flywheel.setVoltage(topIntakingRoller, INDEXER_PRESET.IDLE_BOTTOM));
	}

	/** Feeds both indexers and the conveyor toward the shooter. */
	public static Command feedRollers(Flywheel bottomIntakingRoller, Flywheel topIntakingRoller,
			Flywheel conveyorRoller) {
		return Commands.deadline(Flywheel.setVoltage(bottomIntakingRoller, INDEXER_PRESET.FEED_BOTTOM),
				Flywheel.setVoltage(topIntakingRoller, INDEXER_PRESET.FEED_TOP),
				Flywheel.setVoltage(conveyorRoller, CONVEYOR_PRESET.FEED),
				launchedFuelSimulation != null ? launchedFuelSimulation.launchCommand() : Commands.none());
	}

	/** Pulses the two indexers without running the conveyor. */
	public static Command agitateIntake(Flywheel bottomIntakingRoller, Flywheel topIntakingRoller) {
		return Commands.deadline(Flywheel.setVoltage(bottomIntakingRoller, INDEXER_PRESET.FEED_BOTTOM),
				Flywheel.setVoltage(topIntakingRoller, INDEXER_PRESET.FEED_TOP),
				launchedFuelSimulation != null ? launchedFuelSimulation.launchCommand() : Commands.none());
	}

	/** Stows the hood and stops the shooter flywheel. */
	public static Command shooterIdle(Flywheel shooterRoller, PositionJoint hood) {
		return Commands.deadline(PositionJoint.setPosition(hood, HOOD_PRESET.STOW),
				Flywheel.setVoltage(shooterRoller, SHOOTER_PRESET.IDLE));
	}

	/** Applies the close hub shot preset. */
	public static Command hubPreset(Flywheel shooterRoller, PositionJoint hood) {
		return Commands.deadline(PositionJoint.setPosition(hood, HOOD_PRESET.HUB),
				Flywheel.setVelocity(shooterRoller, SHOOTER_PRESET.HUB));
	}

	/** Applies the ferry shot preset. */
	public static Command ferryPreset(Flywheel shooter, PositionJoint hood) {
		return Commands.deadline(PositionJoint.setPosition(hood, HOOD_PRESET.FERRY),
				Flywheel.setVelocity(shooter, SHOOTER_PRESET.FERRY));
	}

	/** Applies the trench shot preset. */
	public static Command trenchPreset(Flywheel shooter, PositionJoint hood) {
		return Commands.deadline(PositionJoint.setPosition(hood, HOOD_PRESET.TRENCH),
				Flywheel.setVelocity(shooter, SHOOTER_PRESET.TRENCH));
	}
}
