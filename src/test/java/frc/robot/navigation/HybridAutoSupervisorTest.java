package frc.robot.navigation;

import static org.junit.jupiter.api.Assertions.*;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.*;
import com.pathplanner.lib.pathfinding.Pathfinder;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.*;
import java.nio.file.Files;
import java.util.*;
import org.junit.jupiter.api.*;

/**
 * Real follower/controller integration with a deterministic
 * acceleration-limited chassis model. This is deliberately not a calibrated
 * tire/contact model or a substitute for MapleSim/hardware.
 */
class HybridAutoSupervisorTest {
	private static final double DT = .02;
	private static final AutoSafetyConfig LIMITS = AutoSafetyConfig.defaults();
	private final CommandScheduler scheduler = CommandScheduler.getInstance();
	private static RobotConfig robot;
	enum Disturbance {
		NONE, LATERAL_PUSH, FORWARD_PUSH, BACKWARD_PUSH, SLIP, STALL_HALF, STALL_ONE, PERMANENT_STALL, NEAR_END_PUSH, MULTIPLE_PUSH
	}

	@BeforeAll
	static void initialize() throws Exception {
		assertTrue(HAL.initialize(500, 0));
		robot = RobotConfig.fromGUISettings();
	}
	@BeforeEach
	void setup() {
		scheduler.cancelAll();
		SimHooks.pauseTiming();
		DriverStationSim.setDsAttached(true);
		DriverStationSim.setEnabled(true);
		DriverStationSim.notifyNewData();
		DriverStation.refreshData();
	}
	@AfterEach
	void cleanup() {
		scheduler.cancelAll();
		scheduler.clearComposedCommands();
		DriverStationSim.setEnabled(false);
		DriverStationSim.notifyNewData();
		DriverStation.refreshData();
		SimHooks.resumeTiming();
	}

	private static PathPlannerPath route(int shape) {
		List<Pose2d> points = shape == 1
				? List.of(new Pose2d(2, 2, Rotation2d.fromDegrees(0)), new Pose2d(4, 3, Rotation2d.fromDegrees(30)),
						new Pose2d(6, 3, Rotation2d.fromDegrees(-20)))
				: List.of(new Pose2d(2, 2, new Rotation2d()), new Pose2d(6, 2, new Rotation2d()));
		return new PathPlannerPath(PathPlannerPath.waypointsFromPoses(points), new PathConstraints(2, 2, 3, 6),
				new IdealStartingState(0, new Rotation2d()),
				new GoalEndState(0, Rotation2d.fromDegrees(shape == 2 ? 160 : 0)));
	}

	/**
	 * Field-frame inertial state and estimator are separate. Impulses change
	 * velocity only.
	 */
	private static final class Plant extends SubsystemBase {
		Pose2d truth, estimate;
		ChassisSpeeds demand = new ChassisSpeeds();
		double vx, vy, omega;
		boolean healthy = true;
		Plant(Pose2d start) {
			truth = start;
			estimate = start;
		}
		ChassisSpeeds measured() {
			return ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, estimate.getRotation());
		}
		void output(ChassisSpeeds value) {
			assertTrue(NavigationOutput.finite(value), "nonfinite output");
			demand = value;
		}
		void step(double traction, boolean blocked) {
			var field = ChassisSpeeds.fromRobotRelativeSpeeds(demand, truth.getRotation());
			double dx = field.vxMetersPerSecond - vx, dy = field.vyMetersPerSecond - vy;
			double fraction = Math.min(1, 2.0 * traction * DT / Math.max(1e-12, Math.hypot(dx, dy)));
			vx += dx * fraction;
			vy += dy * fraction;
			omega += Math.max(-6 * DT, Math.min(6 * DT, field.omegaRadiansPerSecond - omega));
			if (blocked) {
				vx = 0;
				vy = 0;
				omega = 0;
			}
			truth = new Pose2d(truth.getX() + vx * DT, truth.getY() + vy * DT,
					truth.getRotation().plus(new Rotation2d(omega * DT)));
			estimate = new Pose2d(truth.getTranslation(), truth.getRotation());
		}
	}

	private HybridAutoSupervisor hybrid(Plant p, PathPlannerPath path, boolean flip, int[] actions) {
		return new HybridAutoSupervisor(
				new AutoMissionPlan(
						List.of(new AutoMissionPlan.Segment("test", path, Commands.runOnce(() -> actions[0]++)))),
				p, () -> p.estimate, p::measured, p::output, () -> p.healthy, () -> flip, robot, new NoRoute(),
				new NavigationSafety(17.55, 8.05, .45, List.of()), ObstacleSource.empty(), LIMITS);
	}
	private static final class NoRoute implements Pathfinder {
		public boolean isNewPathAvailable() {
			return false;
		}
		public PathPlannerPath getCurrentPath(PathConstraints c, GoalEndState e) {
			return null;
		}
		public void setStartPosition(Translation2d p) {
		}
		public void setGoalPosition(Translation2d p) {
		}
		public void setDynamicObstacles(List<Pair<Translation2d, Translation2d>> o, Translation2d p) {
		}
	}
	private record Result(boolean success, double endpoint, double heading, double speed, double seconds, double rms,
			double p95, double maximum, int attempts, int transitions, int violations, double maxLoopMs,
			double recoverySeconds, String reason) {
		String csv(String controller, Disturbance disturbance, int seed, int shape, boolean flip) {
			return String.format(Locale.ROOT,
					"%s,%s,%d,%d,%s,%s,%.6f,%.6f,%.6f,%.3f,%.6f,%.6f,%.6f,%d,%d,%d,%.6f,%.6f,%s%n", controller,
					disturbance, seed, shape, flip, success, endpoint, heading, speed, seconds, rms, p95, maximum,
					attempts, transitions, violations, maxLoopMs, recoverySeconds, reason);
		}
	}

	private Result run(boolean useHybrid, Disturbance disturbance, int seed, int shape, boolean flip) {
		var authored = route(shape);
		var selected = flip ? authored.flipPath() : authored;
		Pose2d start = new Pose2d(selected.getPoint(0).position, selected.getIdealStartingState().rotation());
		Pose2d goal = new Pose2d(selected.getPoint(selected.numPoints() - 1).position,
				selected.getGoalEndState().rotation());
		var plant = new Plant(start);
		int[] actions = {0};
		HybridAutoSupervisor supervisor = useHybrid ? hybrid(plant, authored, flip, actions) : null;
		Command command = useHybrid
				? supervisor
				: new FollowPathCommand(selected, () -> plant.estimate, plant::measured, (s, ff) -> plant.output(s),
						HybridAutoSupervisor.pathController(), robot, () -> false, plant);
		var monitor = new PathTrackingMonitor(selected.getPathPoses());
		var rng = new Random(seed);
		double injection = .65 + rng.nextDouble() * 1.25;
		double impulse = 1.4 + rng.nextDouble() * .6;
		boolean injected = false, secondInjected = false;
		List<Double> errors = new ArrayList<>();
		int violations = 0, transitions = 0;
		String oldState = "";
		double maxLoop = 0;
		int step = 0;
		scheduler.schedule(command);
		try {
			for (; step < 1600 && command.isScheduled(); step++) {
				double t = step * DT;
				boolean injectionReady = disturbance == Disturbance.NEAR_END_PUSH
						? plant.truth.getTranslation().getDistance(goal.getTranslation()) < .8
						: t >= injection;
				if (!injected && injectionReady) {
					injected = true;
					// Physical momentum impulse, never a pose reset or estimator-only jump.
					if (disturbance == Disturbance.LATERAL_PUSH || disturbance == Disturbance.NEAR_END_PUSH
							|| disturbance == Disturbance.MULTIPLE_PUSH)
						plant.vy += (seed % 2 == 0 ? 1 : -1) * impulse;
					if (disturbance == Disturbance.FORWARD_PUSH)
						plant.vx += (flip ? -1 : 1) * impulse;
					if (disturbance == Disturbance.BACKWARD_PUSH)
						plant.vx -= (flip ? -1 : 1) * impulse;
				}
				if (disturbance == Disturbance.MULTIPLE_PUSH && injected && !secondInjected && t >= injection + 1) {
					plant.vy -= (seed % 2 == 0 ? 1 : -1) * impulse;
					secondInjected = true;
				}
				double duration = disturbance == Disturbance.STALL_HALF
						? .5
						: disturbance == Disturbance.PERMANENT_STALL ? 100 : 1;
				boolean blocked = (disturbance == Disturbance.STALL_HALF || disturbance == Disturbance.STALL_ONE
						|| disturbance == Disturbance.PERMANENT_STALL) && t >= injection && t < injection + duration;
				double traction = disturbance == Disturbance.SLIP && t >= injection && t < injection + 1.2 ? .15 : 1;
				plant.step(traction, blocked);
				SimHooks.stepTiming(DT);
				long before = System.nanoTime();
				scheduler.run();
				maxLoop = Math.max(maxLoop, (System.nanoTime() - before) / 1e6);
				errors.add(monitor.project(plant.truth.getTranslation()).crossTrack());
				if (plant.truth.getX() < .45 || plant.truth.getX() > 17.1 || plant.truth.getY() < .45
						|| plant.truth.getY() > 7.6)
					violations++;
				if (supervisor != null) {
					String state = supervisor.state().name();
					if (!oldState.equals(state)) {
						transitions++;
						oldState = state;
					}
					assertTrue(Math.hypot(plant.demand.vxMetersPerSecond,
							plant.demand.vyMetersPerSecond) <= LIMITS.maxSpeed() + 1e-6);
					assertTrue(Math.abs(plant.demand.omegaRadiansPerSecond) <= LIMITS.maxOmega() + 1e-6);
				}
			}
			if (disturbance == Disturbance.NEAR_END_PUSH)
				assertTrue(injected, "Endpoint scenario must actually inject its physical disturbance");
			if (disturbance == Disturbance.MULTIPLE_PUSH)
				assertTrue(secondInjected, "Multiple-push scenario must inject both physical disturbances");
			// A timed follower may terminate while the physical chassis is still braking.
			// Compare both implementations after the same bounded passive stop, without
			// allowing an extra position controller to repair the baseline endpoint.
			if (!command.isScheduled()) {
				plant.output(new ChassisSpeeds());
				for (int brakingStep = 0; brakingStep < 50
						&& (Math.hypot(plant.vx, plant.vy) >= .15 || Math.abs(plant.omega) >= .15); brakingStep++) {
					double t = step * DT;
					double duration = disturbance == Disturbance.STALL_HALF
							? .5
							: disturbance == Disturbance.PERMANENT_STALL ? 100 : 1;
					boolean blocked = (disturbance == Disturbance.STALL_HALF || disturbance == Disturbance.STALL_ONE
							|| disturbance == Disturbance.PERMANENT_STALL) && t >= injection
							&& t < injection + duration;
					double traction = disturbance == Disturbance.SLIP && t >= injection && t < injection + 1.2
							? .15
							: 1;
					plant.step(traction, blocked);
					SimHooks.stepTiming(DT);
					step++;
					errors.add(monitor.project(plant.truth.getTranslation()).crossTrack());
					if (plant.truth.getX() < .45 || plant.truth.getX() > 17.1 || plant.truth.getY() < .45
							|| plant.truth.getY() > 7.6)
						violations++;
				}
			}
			double endpoint = plant.truth.getTranslation().getDistance(goal.getTranslation());
			double heading = Math.abs(plant.truth.getRotation().minus(goal.getRotation()).getDegrees());
			double speed = Math.hypot(plant.vx, plant.vy);
			boolean success = !command.isScheduled() && endpoint < .15 && heading < 5 && speed < .15
					&& (!useHybrid || supervisor.state() == HybridAutoSupervisor.NavigationState.COMPLETE);
			if (useHybrid)
				assertEquals(success ? 1 : 0, actions[0], "arrival action must reflect physical arrival");
			double rms = Math.sqrt(errors.stream().mapToDouble(e -> e * e).average().orElse(0));
			errors.sort(Double::compare);
			return new Result(success, endpoint, heading, speed, step * DT, rms,
					errors.get(Math.min(errors.size() - 1, (int) (errors.size() * .95))), errors.get(errors.size() - 1),
					useHybrid ? supervisor.recoveryAttempts() : 0, transitions, violations, maxLoop,
					useHybrid ? supervisor.recoveryDurationSeconds() : 0,
					useHybrid ? supervisor.reason() : "Timed follower ended");
		} finally {
			scheduler.cancel(command);
			scheduler.unregisterSubsystem(plant);
		}
	}

	@Test
	void deterministicPhysicalDisturbanceBenchmark() throws Exception {
		var csv = new StringBuilder(
				"controller,disturbance,seed,shape,flipped,success,endpoint_m,heading_deg,speed_mps,seconds,rms_cross_track_m,p95_cross_track_m,max_cross_track_m,recovery_attempts,observed_transitions,clearance_violations,max_loop_ms,recovery_seconds,reason\n");
		List<String> failures = new ArrayList<>();
		int recovered = 0;
		for (var disturbance : Disturbance.values()) {
			int count = disturbance == Disturbance.NONE ? 6 : 30;
			for (int seed = 0; seed < count; seed++) {
				int shape = seed % 3;
				boolean flip = seed % 2 == 1;
				for (boolean hybrid : new boolean[]{false, true}) {
					Result result = run(hybrid, disturbance, seed, shape, flip);
					csv.append(result.csv(hybrid ? "hybrid" : "baseline", disturbance, seed, shape, flip));
					if (hybrid && result.attempts > 0 && result.success)
						recovered++;
					if (hybrid && result.violations > 0)
						failures.add(disturbance + " seed=" + seed + " clearance violation");
					if (hybrid && disturbance != Disturbance.PERMANENT_STALL && !result.success)
						failures.add(
								disturbance + " seed=" + seed + ": " + result.reason + " error=" + result.endpoint);
					if (hybrid && disturbance == Disturbance.PERMANENT_STALL && result.success)
						failures.add("Impossible stalled mission completed");
				}
			}
		}
		var report = java.nio.file.Path.of("build/reports/hybrid/physical-benchmark.csv");
		Files.createDirectories(report.getParent());
		Files.writeString(report, csv);
		assertTrue(failures.isEmpty(), () -> String.join("\n", failures));
		assertTrue(recovered > 0, "Actual BLine recovery must complete representative physical disturbances");
	}

	@Test
	void invalidLocalizationAndDiscontinuousCorrectionsStopWithoutActions() {
		for (int fault = 0; fault < 3; fault++) {
			Plant p = new Plant(new Pose2d(2, 2, new Rotation2d()));
			int[] actions = {0};
			var command = hybrid(p, route(0), false, actions);
			scheduler.schedule(command);
			for (int i = 0; i < 30; i++) {
				p.step(1, false);
				SimHooks.stepTiming(DT);
				scheduler.run();
			}
			if (fault == 0)
				p.healthy = false;
			if (fault == 1)
				p.estimate = new Pose2d(Double.NaN, 2, new Rotation2d());
			if (fault == 2)
				p.estimate = new Pose2d(4, 4, new Rotation2d());
			SimHooks.stepTiming(DT);
			scheduler.run();
			assertEquals(HybridAutoSupervisor.NavigationState.SAFE_ABORT, command.state());
			assertEquals(0, actions[0]);
			assertEquals(0, p.demand.vxMetersPerSecond, 1e-12);
			assertEquals(0, p.demand.vyMetersPerSecond, 1e-12);
			scheduler.unregisterSubsystem(p);
		}
	}

	@Test
	void schedulerConflictAndDisableInterruptExclusiveNavigation() {
		for (boolean conflict : new boolean[]{false, true}) {
			Plant p = new Plant(new Pose2d(2, 2, new Rotation2d()));
			int[] actions = {0};
			var command = hybrid(p, route(0), false, actions);
			scheduler.schedule(command);
			for (int i = 0; i < 30; i++) {
				p.step(1, false);
				SimHooks.stepTiming(DT);
				scheduler.run();
			}
			Command other = Commands.run(() -> {
			}, p);
			if (conflict)
				scheduler.schedule(other);
			else {
				DriverStationSim.setEnabled(false);
				DriverStationSim.notifyNewData();
				DriverStation.refreshData();
			}
			SimHooks.stepTiming(DT);
			scheduler.run();
			assertFalse(command.isScheduled());
			assertEquals(HybridAutoSupervisor.NavigationState.SAFE_ABORT, command.state());
			assertEquals(0, actions[0]);
			assertEquals(0, p.demand.vxMetersPerSecond, 1e-12);
			scheduler.cancel(other);
			scheduler.unregisterSubsystem(p);
			DriverStationSim.setEnabled(true);
			DriverStationSim.notifyNewData();
			DriverStation.refreshData();
		}
	}
}
