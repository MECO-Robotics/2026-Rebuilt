package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.subsystems.ShooterConstants.CONVEYOR_PRESET;
import frc.robot.constants.subsystems.ShooterConstants.HOOD_PRESET;
import frc.robot.constants.subsystems.ShooterConstants.INDEXER_PRESET;
import frc.robot.constants.subsystems.ShooterConstants.SHOOTER_PRESET;
// import frc.robot.simulation.LaunchedFuelSim;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.position_joint.PositionJoint;

/** Factory methods for coordinated shooter/indexer/conveyor command groups. */
public class ShooterCommands {
	// private static LaunchedFuelSim launchedFuelSimulation;

	// public static void setLaunchedFuelSimulation(LaunchedFuelSim sim) {
	// launchedFuelSimulation = sim;
	// }

	/**
	 * Puts the Shooter and bottom indexers in a slow idle speed, and stopping the
	 * top indexer
	 */
	public static Command idleRollers(Flywheel bottomIntakingRoller, Flywheel topIntakingRoller,
			Flywheel conveyorRoller) {
		return Commands.parallel(
				Flywheel.setVoltage(bottomIntakingRoller, INDEXER_PRESET.IDLE_BOTTOM),
				Flywheel.setVoltage(topIntakingRoller, INDEXER_PRESET.IDLE_BOTTOM),
				Flywheel.setVoltage(conveyorRoller, CONVEYOR_PRESET.IDLE));
	}

	/** Puts both indexers feeding towards the shooter */
	public static Command feedRollers(Flywheel bottomIntakingRoller, Flywheel topIntakingRoller,
			Flywheel conveyorRoller) {
		return Commands.parallel(
				Flywheel.setVoltage(bottomIntakingRoller, INDEXER_PRESET.FEED_BOTTOM),
				Flywheel.setVoltage(topIntakingRoller, INDEXER_PRESET.FEED_TOP),
				Flywheel.setVoltage(conveyorRoller, CONVEYOR_PRESET.FEED));
		// launchedFuelSimulation != null ? launchedFuelSimulation.launchCommand() :
		// Commands.none());
	}

	/* PRESETS */

	public static Command shooterIdle(Flywheel shooterRoller, PositionJoint hood) {
		return Commands.deadline(
				PositionJoint.setPosition(hood, HOOD_PRESET.STOW),
				Flywheel.setVoltage(shooterRoller, SHOOTER_PRESET.IDLE));
	}

	public static Command hubPreset(Flywheel shooterRoller, PositionJoint hood) {
		return Commands.deadline(
				PositionJoint.setPosition(hood, HOOD_PRESET.HUB),
				Flywheel.setVoltage(shooterRoller, SHOOTER_PRESET.HUB));
	}	

	public static Command ferryPreset(Flywheel shooter, PositionJoint hood) {
		return Commands.deadline(
				PositionJoint.setPosition(hood, HOOD_PRESET.FERRY),
				Flywheel.setVelocity(shooter, SHOOTER_PRESET.FERRY));
	}
}
