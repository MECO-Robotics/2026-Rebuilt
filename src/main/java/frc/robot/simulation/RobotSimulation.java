package frc.robot.simulation;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.shooter.ShooterCommands;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.position_joint.PositionJoint;
import frc.robot.util.visualization.RobotRemyVisualizer;
import java.util.Arrays;
import java.util.Set;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.littletonrobotics.junction.Logger;

/**
 * Runtime simulation integration hooks with a no-op implementation for non-sim
 * modes.
 */
public interface RobotSimulation {
	void bindCommandHooks();

	void visualizationPeriodic();

	void autonomousInit(Command autonomousCommand);

	void simulationPeriodic();

	static void configureArenaOverride(Constants.Mode mode) {
		if (mode == Constants.Mode.SIM || mode == Constants.Mode.REPLAY) {
			SimulatedArena
					.overrideInstance(new org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt(false));
		}
	}

	static RobotSimulation create(CommandSwerveDrivetrain drive, PositionJoint intakeRack, PositionJoint hood,
			Flywheel shooterFlywheel) {
		if (Constants.currentMode == Constants.Mode.SIM && drive.getSimulation() != null) {
			return new MapleRobotSimulation(drive, intakeRack, hood, shooterFlywheel);
		}
		return new NoopRobotSimulation(drive, intakeRack, hood, shooterFlywheel);
	}

	class NoopRobotSimulation implements RobotSimulation {
		private final RobotRemyVisualizer robotRemyVisualizer;

		NoopRobotSimulation(CommandSwerveDrivetrain drive, PositionJoint intakeRack, PositionJoint hood,
				Flywheel shooterFlywheel) {
			robotRemyVisualizer = new RobotRemyVisualizer(drive::getPhysicsPose, hood::getPosition,
					shooterFlywheel::getPosition, intakeRack::getPosition, () -> 0);
		}

		@Override
		public void bindCommandHooks() {
			IntakeCommands.setIntakeSimulation(null);
			ShooterCommands.setLaunchedFuelSimulation(null);
		}

		@Override
		public void visualizationPeriodic() {
			robotRemyVisualizer.periodic();
		}

		@Override
		public void autonomousInit(Command autonomousCommand) {
		}

		@Override
		public void simulationPeriodic() {
		}
	}

	class MapleRobotSimulation implements RobotSimulation {
		private final CommandSwerveDrivetrain drive;
		private final LaunchedFuelSim launchedFuelSim;
		private final Hopper hopper;
		private final RobotRemyVisualizer robotRemyVisualizer;

		MapleRobotSimulation(CommandSwerveDrivetrain drive, PositionJoint intakeRack, PositionJoint hood,
				Flywheel shooterFlywheel) {
			this.drive = drive;
			IntakeSim intakeSim = new IntakeSim(drive.getSimulation());
			launchedFuelSim = new LaunchedFuelSim(drive, intakeSim, hood, shooterFlywheel);
			hopper = new Hopper(intakeSim::getStoredFuelCount, intakeRack::getPosition, drive::getPhysicsPose);
			robotRemyVisualizer = new RobotRemyVisualizer(drive::getPhysicsPose, hood::getPosition,
					shooterFlywheel::getPosition, intakeRack::getPosition, () -> 0);
			IntakeCommands.setIntakeSimulation(intakeSim);
			ShooterCommands.setLaunchedFuelSimulation(launchedFuelSim);
		}

		@Override
		public void bindCommandHooks() {
		}

		@Override
		public void visualizationPeriodic() {
			hopper.periodic();
			robotRemyVisualizer.periodic();
		}

		@Override
		public void autonomousInit(Command autonomousCommand) {
			SimulatedArena.getInstance().resetFieldForAuto();
			launchedFuelSim.resetSuccessfulScoreCount();
			resetAutonomousPose(autonomousCommand);
		}

		@Override
		public void simulationPeriodic() {
			Pose3d[] arenaFuelPoses = SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel");
			Pose3d[] hopperFuelPoses = hopper.getGamePiecePoses();
			Pose3d[] fuelPoses = concat(arenaFuelPoses, hopperFuelPoses);

			Set<GamePieceProjectile> projectiles = SimulatedArena.getInstance().gamePieceLaunched();
			Pose3d[] fuelProjectilePoses = projectiles.stream()
					.filter(projectile -> "Fuel".equals(projectile.getType())).map(GamePieceProjectile::getPose3d)
					.toArray(Pose3d[]::new);

			Logger.recordOutput("FieldSimulation/FuelPositions", fuelPoses);
			Logger.recordOutput("FieldSimulation/HopperFuelPositions", hopperFuelPoses);
			Logger.recordOutput("FieldSimulation/HopperFuelCount", hopperFuelPoses.length);
			Logger.recordOutput("FieldSimulation/SuccessfulScoreCount", launchedFuelSim.getSuccessfulScoreCount());
			Logger.recordOutput("FieldSimulation/FuelProjectilePositions", fuelProjectilePoses);
			Logger.recordOutput("FieldSimulation/FuelProjectileCount", fuelProjectilePoses.length);
		}

		private void resetAutonomousPose(Command autonomousCommand) {
			if (!(autonomousCommand instanceof PathPlannerAuto pathPlannerAuto)) {
				return;
			}

			Pose2d startingPose = pathPlannerAuto.getStartingPose();
			if (startingPose == null) {
				return;
			}

			Pose2d allianceAdjustedPose = Constants.isAllianceRed()
					? FlippingUtil.flipFieldPose(startingPose)
					: startingPose;
			drive.getSimulation().setSimulationWorldPose(allianceAdjustedPose);
			drive.resetPose(allianceAdjustedPose);
		}

		private Pose3d[] concat(Pose3d[] first, Pose3d[] second) {
			if (second.length == 0) {
				return first;
			}
			Pose3d[] result = Arrays.copyOf(first, first.length + second.length);
			System.arraycopy(second, 0, result, first.length, second.length);
			return result;
		}
	}
}
