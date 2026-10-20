package frc.robot.navigation;

import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.pathplanner.LocalADStarAK;
import java.util.Set;
import java.util.List;
import java.util.function.Function;
import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.simulation.HybridSimulationEnvironment;
import org.littletonrobotics.junction.Logger;

/**
 * Hybrid survey and validated, opt-in competition auto candidates.
 */
public final class HybridAutos {
	public static final String SURVEY_AUTO_NAME = "EXPERIMENTAL Hybrid Survey (neutral zone)";
	public static final String MIGRATION_PREFIX = "HYBRID ";

	public static boolean isHybridAuto(String name) {
		return SURVEY_AUTO_NAME.equals(name) || name.startsWith(MIGRATION_PREFIX);
	}

	/**
	 * Load a staged PathPlanner auto without ever falling back to timed following.
	 */
	public static Command fromAutoFile(String name, CommandSwerveDrivetrain drive, Vision vision,
			Function<String, Command> named) {
		try {
			var definition = HybridAutoDefinition.load(name);
			boolean simulated = HybridSimulationEnvironment.enabled();
			var safety = simulated
					? HybridSimulationEnvironment.safety()
					: NavigationSafety.fromDeploy(RobotNavigationFootprint.DEPLOYED_RADIUS_METERS);
			var issues = definition.structuralIssues();
			if (!issues.isEmpty())
				throw new IllegalArgumentException(String.join("; ", issues));
			SmartDashboard.putString("Hybrid/Migration/" + name, "Path migration complete");
			var clearance = definition.clearanceIssues(safety);
			if (!clearance.isEmpty())
				throw new IllegalArgumentException("Clearance pending: " + String.join("; ", clearance));
			var robot = RobotConfig.fromGUISettings();
			var config = simulated ? HybridSimulationEnvironment.config() : AutoSafetyConfig.tunedDefaults();
			com.pathplanner.lib.pathfinding.Pathfinder planner = simulated
					? HybridSimulationEnvironment.pathfinder()
					: PlannerHolder.INSTANCE;
			BooleanSupplier healthy = simulated
					? () -> HybridSimulationEnvironment.enabled() && DriverStation.getAlliance().isPresent()
					: () -> DriverStation.getAlliance().isPresent() && vision.isPoseReady() && vision.isConnected()
							&& vision.getLastAbsoluteCorrectionAgeSeconds() < 2;
			var command = new HybridCommandAuto(definition, named, pathName -> {
				var mission = new AutoMissionPlan(List
						.of(new AutoMissionPlan.Segment(pathName, definition.paths().get(pathName), Commands.none())));
				return new HybridAutoSupervisor(mission, drive,
						() -> simulated ? drive.getPhysicsPose() : drive.getState().Pose,
						() -> simulated ? drive.getPhysicsSpeeds() : drive.getState().Speeds, drive::applyHybridSpeeds,
						healthy,
						() -> DriverStation.getAlliance()
								.orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red,
						robot, planner, safety, new LoggedObstacleSource(ObstacleSource.empty()), config);
			}, drive, healthy, drive::stop, config.missionTimeout());
			if (simulated)
				return new HybridSimulationAuto(name, command, drive, definition.paths().values().iterator().next());
			SmartDashboard.putString("Hybrid/Readiness/" + name,
					"Geometry and command validation passed; hardware unverified");
			return command.withName(name);
		} catch (Exception ex) {
			String reason = "Hybrid auto unavailable: " + name + ": " + ex.getMessage();
			SmartDashboard.putString("Hybrid/Readiness/" + name, reason);
			Logger.recordOutput("Hybrid/Readiness/" + name, reason);
			return Commands.runOnce(() -> {
				drive.stop();
				DriverStation.reportError(reason, false);
			}, drive).withName(name + " (BLOCKED)");
		}
	}

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
			var paths = HybridAutoFile.load(SURVEY_AUTO_NAME);
			var robot = RobotConfig.fromGUISettings();
			var safety = NavigationSafety.fromDeploy(RobotNavigationFootprint.DEPLOYED_RADIUS_METERS);
			var planner = PlannerHolder.INSTANCE;
			return Commands.defer(() -> {
				try {
					var mission = new AutoMissionPlan(
							paths.stream()
									.map(path -> new AutoMissionPlan.Segment(path.name, path, Commands.runOnce(
											() -> Logger.recordOutput("Hybrid/Survey/LastVisitedPath", path.name))))
									.toList());
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
