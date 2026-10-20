package frc.robot.navigation;

import static org.junit.jupiter.api.Assertions.*;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.util.pathplanner.LocalADStarAK;
import java.util.List;
import java.util.Random;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Real asynchronous AD* integration; the plant here is explicitly kinematic,
 * not physics.
 */
class HybridPathfindingTest {
	private static final Pose2d START = new Pose2d(6.6, 4, Rotation2d.kZero);
	private static final Pose2d GOAL = new Pose2d(9.6, 4, Rotation2d.kZero);
	private static final PathConstraints LIMITS = new PathConstraints(2, 2, 3, 6);
	private HybridAutoSupervisor supervisor;

	@BeforeEach
	void setup() {
		HAL.initialize(500, 0);
		SimHooks.pauseTiming();
	}

	@AfterEach
	void cleanup() {
		if (supervisor != null) {
			supervisor.end(true);
		}
		CommandScheduler.getInstance().clearComposedCommands();
		SimHooks.resumeTiming();
	}

	@Test
	void actualAdStarRoutesAroundInflatedObstacleWithSafeContinuousCorridor() throws Exception {
		var safety = NavigationSafety.fromDeploy(.25);
		safety.update(obstacle(false).observations(Timer.getFPGATimestamp()), Timer.getFPGATimestamp());
		assertFalse(safety.safeSegment(START.getTranslation(), GOAL.getTranslation()));
		var planner = new LocalADStarAK();
		planner.setDynamicObstacles(safety.plannerObstacles(), START.getTranslation());
		planner.setStartPosition(START.getTranslation());
		planner.setGoalPosition(GOAL.getTranslation());
		PathPlannerPath accepted = null;
		long deadline = System.nanoTime() + 3_000_000_000L;
		while (System.nanoTime() < deadline && accepted == null) {
			if (planner.isNewPathAvailable()) {
				var candidate = planner.getCurrentPath(LIMITS, new GoalEndState(0, GOAL.getRotation()));
				if (candidate != null && safe(safety, candidate)) {
					accepted = candidate;
				}
			}
			Thread.sleep(5);
		}
		assertNotNull(accepted, "AD* must produce a corridor accepted by the supervisor clearance policy");
		assertTrue(accepted.getPoint(accepted.numPoints() - 1).position.getDistance(GOAL.getTranslation()) < .15);
		assertTrue(accepted.getPathPoses().stream().anyMatch(p -> Math.abs(p.getY() - 4) > .7),
				"An actual detour must leave the blocked straight route");
	}

	@Test
	void supervisorExecutesRealAdStarDetourAndCompletesActionExactlyOnce() throws Exception {
		var plant = new Plant();
		int[] actions = {0};
		var safety = NavigationSafety.fromDeploy(.25);
		supervisor = create(plant, safety, obstacle(false), actions);
		supervisor.initialize();
		assertEquals(HybridAutoSupervisor.NavigationState.OBSTACLE_REROUTE, supervisor.state());
		int cycles = 0;
		while (!supervisor.isFinished() && cycles++ < 1000) {
			SimHooks.stepTiming(.02);
			supervisor.execute();
			plant.step();
			assertTrue(safety.safePoint(plant.pose.getTranslation()), "Kinematic robot entered an inflated obstacle");
			Thread.sleep(5);
		}
		assertEquals(HybridAutoSupervisor.NavigationState.COMPLETE, supervisor.state(), supervisor.reason());
		assertEquals(1, actions[0]);
		assertEquals(1, supervisor.completedObjectives());
		assertTrue(plant.pose.getTranslation().getDistance(GOAL.getTranslation()) < .10);
		assertEquals(0, Math.hypot(plant.speeds.vxMetersPerSecond, plant.speeds.vyMetersPerSecond), 1e-9);
	}

	@Test
	void impenetrableWallFailsWithinPlannerDeadlineWithoutMotionOrAction() throws Exception {
		var plant = new Plant();
		int[] actions = {0};
		supervisor = create(plant, NavigationSafety.fromDeploy(.25), obstacle(true), actions);
		supervisor.initialize();
		assertEquals(HybridAutoSupervisor.NavigationState.OBSTACLE_REROUTE, supervisor.state());
		double start = Timer.getFPGATimestamp();
		for (int cycle = 0; cycle < 120 && !supervisor.isFinished(); cycle++) {
			SimHooks.stepTiming(.02);
			supervisor.execute();
			plant.step();
			Thread.sleep(5);
		}
		assertEquals(HybridAutoSupervisor.NavigationState.SAFE_ABORT, supervisor.state(), supervisor.reason());
		assertTrue(supervisor.reason().contains("AD* route deadline"), supervisor.reason());
		assertTrue(Timer.getFPGATimestamp() - start <= 2.04);
		assertEquals(START, plant.pose);
		assertEquals(0, actions[0]);
		assertEquals(0, supervisor.completedObjectives());
		assertEquals(0, plant.speeds.vxMetersPerSecond);
		assertEquals(0, plant.speeds.vyMetersPerSecond);
	}

	@Test
	void fullRobotInflationStillPermitsAnActualAlternateRoute() throws Exception {
		var plant = new Plant();
		// The larger footprint cannot occupy START beside the inflated static hub.
		plant.pose = new Pose2d(7, 4, Rotation2d.kZero);
		int[] actions = {0};
		var safety = NavigationSafety.fromDeploy(.65);
		ObstacleSource source = now -> List.of(new ObstacleSource.Observation(new Translation2d(8.05, 3.95),
				new Translation2d(8.15, 4.05), now, 1, new Translation2d()));
		supervisor = create(plant, safety, source, actions);
		supervisor.initialize();
		runUntilTerminal(plant, safety, 800);
		assertEquals(HybridAutoSupervisor.NavigationState.COMPLETE, supervisor.state(), supervisor.reason());
		assertEquals(1, actions[0]);
	}

	@Test
	void obstacleAppearingDuringGeometricRecoveryPreemptsBline() throws Exception {
		var plant = new Plant();
		plant.pose = new Pose2d(6.6, 4.4, Rotation2d.kZero);
		var safety = NavigationSafety.fromDeploy(.25);
		double start = Timer.getFPGATimestamp();
		ObstacleSource source = now -> now - start < .12 ? List.of() : obstacle(false).observations(now);
		int[] actions = {0};
		supervisor = create(plant, safety, source, actions);
		supervisor.initialize();
		assertEquals(HybridAutoSupervisor.NavigationState.GEOMETRIC_RECOVERY, supervisor.state());
		boolean rerouted = false;
		for (int cycle = 0; cycle < 800 && !supervisor.isFinished(); cycle++) {
			step(plant, safety);
			rerouted |= supervisor.state() == HybridAutoSupervisor.NavigationState.OBSTACLE_REROUTE;
		}
		assertTrue(rerouted, "New perception must preempt the geometric recovery follower");
		assertTerminalAndStopped(plant, actions);
	}

	@Test
	void disappearingObstacleDoesNotDuplicateTheArrivalAction() throws Exception {
		var plant = new Plant();
		var safety = NavigationSafety.fromDeploy(.25);
		double start = Timer.getFPGATimestamp();
		ObstacleSource source = now -> now - start > .25 ? List.of() : obstacle(false).observations(now);
		int[] actions = {0};
		supervisor = create(plant, safety, source, actions);
		supervisor.initialize();
		assertEquals(HybridAutoSupervisor.NavigationState.OBSTACLE_REROUTE, supervisor.state());
		runUntilTerminal(plant, safety, 800);
		assertEquals(HybridAutoSupervisor.NavigationState.COMPLETE, supervisor.state(), supervisor.reason());
		assertEquals(1, actions[0]);
	}

	@Test
	void multipleObstaclesHaveBoundedSafeOutcome() throws Exception {
		var plant = new Plant();
		var safety = NavigationSafety.fromDeploy(.25);
		ObstacleSource source = now -> List.of(
				new ObstacleSource.Observation(new Translation2d(7.5, 3.3), new Translation2d(7.9, 4.2), now, 1,
						new Translation2d()),
				new ObstacleSource.Observation(new Translation2d(8.6, 3.8), new Translation2d(9, 4.7), now, 1,
						new Translation2d()));
		int[] actions = {0};
		supervisor = create(plant, safety, source, actions);
		supervisor.initialize();
		runUntilTerminal(plant, safety, 800);
		assertTerminalAndStopped(plant, actions);
	}

	@Test
	void thirtySeededAccelerationBoundedOpponentsYieldSafeBoundedOutcomes() throws Exception {
		int successes = 0;
		int aborts = 0;
		var planner = new LocalADStarAK();
		for (int seed = 0; seed < 30; seed++) {
			var plant = new Plant();
			var safety = NavigationSafety.fromDeploy(.25);
			var source = new AcceleratingOpponent(seed, Timer.getFPGATimestamp());
			int[] actions = {0};
			supervisor = create(plant, safety, source, actions, planner);
			supervisor.initialize();
			runUntilTerminal(plant, safety, 800);
			assertTerminalAndStopped(plant, actions);
			if (supervisor.state() == HybridAutoSupervisor.NavigationState.COMPLETE) {
				successes++;
			} else {
				aborts++;
			}
			supervisor.end(false);
			CommandScheduler.getInstance().clearComposedCommands();
		}
		System.out.printf("Hybrid seeded moving opponents: complete=%d safeAbort=%d total=30%n", successes, aborts);
		assertEquals(30, successes + aborts);
		assertTrue(successes > 0, "The varied moving-opponent suite must include successful passage");
	}

	@Test
	void seededOpponentTrajectoriesHaveContinuousPositionAndBoundedAcceleration() {
		for (int seed = 0; seed < 30; seed++) {
			var actor = new AcceleratingOpponent(seed, 0);
			ObstacleSource.Observation previous = null;
			for (int sample = 0; sample < 100; sample++) {
				double time = actor.appears + sample * .02;
				var current = actor.observations(time).get(0);
				if (previous != null) {
					assertTrue(current.velocity().minus(previous.velocity()).getNorm() / .02 <= .8 + 1e-8);
					assertTrue(current.min().getDistance(previous.min()) <= .014 + 1e-8);
				}
				previous = current;
			}
		}
	}

	private void runUntilTerminal(Plant plant, NavigationSafety safety, int limit) throws Exception {
		for (int cycle = 0; cycle < limit && !supervisor.isFinished(); cycle++) {
			step(plant, safety);
		}
		assertTrue(supervisor.isFinished(), "Supervisor exceeded scenario bound: " + supervisor.reason());
	}

	private void step(Plant plant, NavigationSafety safety) throws Exception {
		SimHooks.stepTiming(.02);
		var before = plant.pose;
		supervisor.execute();
		plant.step();
		// A newly observed opponent can occupy the robot's existing footprint. The
		// correct
		// response is a stop; this assertion forbids actively driving through an unsafe
		// corridor.
		if (plant.pose.getTranslation().getDistance(before.getTranslation()) > 1e-10) {
			assertTrue(safety.safeSegment(before.getTranslation(), plant.pose.getTranslation()),
					"Output moved through unsafe geometry: " + supervisor.reason());
		}
		Thread.sleep(5);
	}

	private void assertTerminalAndStopped(Plant plant, int[] actions) {
		assertTrue(supervisor.isFinished(), supervisor.reason());
		assertEquals(supervisor.state() == HybridAutoSupervisor.NavigationState.COMPLETE ? 1 : 0, actions[0]);
		assertEquals(actions[0], supervisor.completedObjectives());
		assertEquals(0, Math.hypot(plant.speeds.vxMetersPerSecond, plant.speeds.vyMetersPerSecond), 1e-9);
		assertEquals(0, plant.speeds.omegaRadiansPerSecond, 1e-9);
	}

	/**
	 * Seeded straight crossings with a one-second acceleration ramp, then constant
	 * velocity.
	 */
	private static final class AcceleratingOpponent implements ObstacleSource {
		final double appears;
		private final Translation2d origin;
		private final Translation2d initialVelocity;
		private final Translation2d finalVelocity;

		AcceleratingOpponent(int seed, double start) {
			var random = new Random(8324L + seed);
			appears = start + .05 + random.nextDouble() * .75;
			double direction = seed % 2 == 0 ? 1 : -1;
			origin = new Translation2d(7.6 + random.nextDouble() * .6, direction > 0 ? 2.4 : 5.0);
			double speed = .2 + random.nextDouble() * .45;
			initialVelocity = new Translation2d(0, direction * speed * .2);
			finalVelocity = new Translation2d((random.nextDouble() - .5) * .2, direction * speed);
		}

		@Override
		public List<Observation> observations(double now) {
			double t = now - appears;
			if (t < 0 || t > 4) {
				return List.of();
			}
			double ramp = Math.min(t, 1);
			var acceleration = finalVelocity.minus(initialVelocity);
			var displacement = initialVelocity.times(ramp).plus(acceleration.times(.5 * ramp * ramp))
					.plus(finalVelocity.times(Math.max(0, t - 1)));
			var min = origin.plus(displacement);
			return List.of(new Observation(min, min.plus(new Translation2d(.45, .45)), now, 1,
					initialVelocity.plus(acceleration.times(ramp))));
		}
	}

	private static HybridAutoSupervisor create(Plant plant, NavigationSafety safety, ObstacleSource source,
			int[] actions) throws Exception {
		return create(plant, safety, source, actions, new LocalADStarAK());
	}

	private static HybridAutoSupervisor create(Plant plant, NavigationSafety safety, ObstacleSource source,
			int[] actions, LocalADStarAK planner) throws Exception {
		var path = new PathPlannerPath(PathPlannerPath.waypointsFromPoses(START, GOAL), LIMITS,
				new IdealStartingState(0, Rotation2d.kZero), new GoalEndState(0, Rotation2d.kZero));
		var mission = new AutoMissionPlan(
				List.of(new AutoMissionPlan.Segment("Detour", path, Commands.runOnce(() -> actions[0]++))));
		return new HybridAutoSupervisor(mission, new Subsystem() {
		}, () -> plant.pose, () -> plant.speeds, s -> plant.speeds = s, () -> true, () -> false,
				RobotConfig.fromGUISettings(), planner, safety, source, AutoSafetyConfig.defaults());
	}

	private static ObstacleSource obstacle(boolean wall) {
		return now -> List.of(new ObstacleSource.Observation(new Translation2d(7.7, wall ? 0 : 3.6),
				new Translation2d(8.3, wall ? 8.07 : 4.4), now, 1, new Translation2d()));
	}

	private static boolean safe(NavigationSafety safety, PathPlannerPath path) {
		var previous = START.getTranslation();
		for (var point : path.getPathPoses()) {
			if (!safety.safeSegment(previous, point.getTranslation())) {
				return false;
			}
			previous = point.getTranslation();
		}
		return true;
	}

	private static final class Plant {
		Pose2d pose = START;
		ChassisSpeeds speeds = new ChassisSpeeds();

		void step() {
			pose = pose.exp(new Twist2d(speeds.vxMetersPerSecond * .02, speeds.vyMetersPerSecond * .02,
					speeds.omegaRadiansPerSecond * .02));
		}
	}
}
