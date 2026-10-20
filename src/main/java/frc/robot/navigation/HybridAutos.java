package frc.robot.navigation;

import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.pathplanner.LocalADStarAK;
import java.util.List;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

/**
 * Experimental neutral-zone survey, deliberately separate from competition
 * autos.
 */
public final class HybridAutos {
	private HybridAutos() {
	}
	private static final class PlannerHolder {
		static final LocalADStarAK INSTANCE = new LocalADStarAK();
		static {
			Pathfinding.setPathfinder(INSTANCE);
			if (DriverStation.isDisabled())
				CommandScheduler.getInstance()
						.schedule(PathfindingCommand.warmupCommand().until(DriverStation::isEnabled));
		}
	}
	public static Command survey(CommandSwerveDrivetrain drive, Vision vision) {
		try {
			// File IO and robot configuration are resolved before autonomous scheduling.
			var outbound = PathPlannerPath.fromPathFile("Hybrid Survey Out");
			var returning = PathPlannerPath.fromPathFile("Hybrid Survey Return");
			var robot = RobotConfig.fromGUISettings();
			var safety = NavigationSafety.fromDeploy(.9);
			var planner = PlannerHolder.INSTANCE;
			return Commands.defer(() -> {
				try {
					var mission = new AutoMissionPlan(List.of(
							new AutoMissionPlan.Segment("Survey outbound", outbound,
									Commands.runOnce(() -> Logger.recordOutput("Hybrid/Survey/OutboundVisited", true))),
							new AutoMissionPlan.Segment("Survey return", returning,
									Commands.runOnce(() -> Logger.recordOutput("Hybrid/Survey/ReturnVisited", true)))));
					return new HybridAutoSupervisor(mission, drive, () -> drive.getState().Pose,
							() -> drive.getState().Speeds, drive::applyHybridSpeeds,
							() -> DriverStation.getAlliance().isPresent() && vision.isPoseReady()
									&& vision.isConnected() && vision.getLastAbsoluteCorrectionAgeSeconds() < 2,
							() -> DriverStation.getAlliance()
									.orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red,
							robot, planner, safety, new LoggedObstacleSource(ObstacleSource.empty()),
							AutoSafetyConfig.tunedDefaults());
				} catch (RuntimeException ex) {
					DriverStation.reportError("Hybrid configuration rejected: " + ex.getMessage(), false);
					return Commands.runOnce(drive::stop, drive);
				}
			}, Set.of(drive)).withName("EXPERIMENTAL Hybrid Survey");
		} catch (Exception ex) {
			DriverStation.reportError("Hybrid survey unavailable: " + ex.getMessage(), false);
			return Commands.runOnce(drive::stop, drive).withName("Hybrid unavailable - stop");
		}
	}
}
