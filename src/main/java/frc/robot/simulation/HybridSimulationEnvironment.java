package frc.robot.simulation;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.pathfinding.Pathfinder;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.constants.Constants;
import frc.robot.navigation.AutoSafetyConfig;
import frc.robot.navigation.NavigationSafety;
import java.util.List;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;

/**
 * Explicit desktop-only fixture. Never substitutes for real-field clearance
 * validation.
 */
public final class HybridSimulationEnvironment {
	private HybridSimulationEnvironment() {
	}

	public static boolean enabled() {
		return allowed(Constants.currentMode, RobotBase.isReal(), Boolean.getBoolean("meco.hybridSimulation")
				|| "true".equalsIgnoreCase(System.getenv("MECO_HYBRID_SIM")));
	}

	public static boolean allowed(Constants.Mode mode, boolean realHardware, boolean requested) {
		return requested && !realHardware && mode == Constants.Mode.SIM;
	}

	public static void requireEnabled() {
		if (!enabled())
			throw new IllegalStateException("Hybrid test environment requires explicitly enabled desktop SIM");
	}

	public static NavigationSafety safety() {
		requireEnabled();
		// Test the command sequence within the authored field coordinate rectangle.
		// There are no static obstacles or footprint boundary margins in this fixture.
		return new NavigationSafety(16.54, 8.07, 0, List.of());
	}

	public static AutoSafetyConfig config() {
		requireEnabled();
		var d = AutoSafetyConfig.defaults();
		// Keep controller/speed bounds; permit slow, segmented routes to finish for
		// diagnosis.
		return new AutoSafetyConfig(d.maxSpeed(), d.maxAcceleration(), d.maxOmega(), d.maxAlpha(),
				d.crossTrackTrigger(), d.severeError(), d.lagTrigger(), d.debounce(), d.stallSeconds(),
				d.dwellSeconds(), 30, d.plannerTimeout(), 180, d.maxAttempts(), d.positionTolerance(),
				d.headingTolerance(), d.speedTolerance());
	}

	public static SimulatedArena arena() {
		requireEnabled();
		return new SimulatedArena(new SimulatedArena.FieldMap() {
		}) {
			@Override
			public void placeGamePiecesOnField() {
				// Sparse, deterministic fuel positions for intake/shot integration.
				for (double x = 1; x < 16; x += .75)
					for (double y = 1; y < 7.5; y += .75)
						addGamePiece(new RebuiltFuelOnField(new Translation2d(x, y)));
			}
		};
	}

	/**
	 * Direct routes are sufficient in this empty fixture; never load the production
	 * obstacle grid.
	 */
	public static Pathfinder pathfinder() {
		requireEnabled();
		return new Pathfinder() {
			private Translation2d start = new Translation2d(), goal = new Translation2d();
			private boolean pending;
			private NavigationSafety bounds = safety();

			public boolean isNewPathAvailable() {
				return pending;
			}
			public PathPlannerPath getCurrentPath(PathConstraints constraints, GoalEndState end) {
				pending = false;
				if (!bounds.safeSegment(start, goal) || start.getDistance(goal) < .001)
					return null;
				var heading = goal.minus(start).getAngle();
				return new PathPlannerPath(
						PathPlannerPath.waypointsFromPoses(new Pose2d(start, heading), new Pose2d(goal, heading)),
						constraints, new IdealStartingState(0, heading), end);
			}
			public void setStartPosition(Translation2d value) {
				start = value;
				pending = true;
			}
			public void setGoalPosition(Translation2d value) {
				goal = value;
				pending = true;
			}
			public void setDynamicObstacles(List<Pair<Translation2d, Translation2d>> obstacles, Translation2d current) {
				bounds = new NavigationSafety(16.54, 8.07, 0, obstacles);
				start = current;
				pending = true;
			}
		};
	}
}
