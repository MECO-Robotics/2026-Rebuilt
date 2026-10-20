package frc.robot.navigation;

import static org.junit.jupiter.api.Assertions.*;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinder;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Lifecycle integration using an ideal velocity plant, not a physical-accuracy
 * benchmark.
 */
class HybridMissionLifecycleTest {
	private static RobotConfig robot;
	private static final double DT = 0.02;
	private final CommandScheduler scheduler = CommandScheduler.getInstance();
	private Drive drive;

	private static final class Drive extends SubsystemBase {
		Pose2d pose = new Pose2d(2, 2, Rotation2d.kZero);
		ChassisSpeeds velocity = new ChassisSpeeds();

		void step() {
			var field = ChassisSpeeds.fromRobotRelativeSpeeds(velocity, pose.getRotation());
			pose = new Pose2d(pose.getX() + field.vxMetersPerSecond * DT, pose.getY() + field.vyMetersPerSecond * DT,
					pose.getRotation().plus(new Rotation2d(field.omegaRadiansPerSecond * DT)));
		}
	}

	private static final class NoRoute implements Pathfinder {
		public boolean isNewPathAvailable() {
			return false;
		}
		public PathPlannerPath getCurrentPath(PathConstraints constraints, GoalEndState goal) {
			return null;
		}
		public void setStartPosition(Translation2d position) {
		}
		public void setGoalPosition(Translation2d position) {
		}
		public void setDynamicObstacles(List<Pair<Translation2d, Translation2d>> bounds, Translation2d current) {
		}
	}

	@BeforeAll
	static void initialize() throws Exception {
		assertTrue(HAL.initialize(500, 0));
		robot = RobotConfig.fromGUISettings();
	}

	@BeforeEach
	void setup() {
		scheduler.cancelAll();
		scheduler.clearComposedCommands();
		SimHooks.pauseTiming();
		DriverStationSim.setDsAttached(true);
		DriverStationSim.setEnabled(true);
		DriverStationSim.notifyNewData();
		DriverStation.refreshData();
		drive = new Drive();
	}

	@AfterEach
	void cleanup() {
		scheduler.cancelAll();
		scheduler.clearComposedCommands();
		scheduler.unregisterSubsystem(drive);
		DriverStationSim.setEnabled(false);
		DriverStationSim.notifyNewData();
		DriverStation.refreshData();
		SimHooks.resumeTiming();
	}

	private static AutoMissionPlan.Segment segment(double start, double end, Command action) {
		var path = new PathPlannerPath(
				PathPlannerPath.waypointsFromPoses(new Pose2d(start, 2, Rotation2d.kZero),
						new Pose2d(end, 2, Rotation2d.kZero)),
				new PathConstraints(2, 2, 3, 6), new IdealStartingState(0, Rotation2d.kZero),
				new GoalEndState(0, Rotation2d.kZero));
		return new AutoMissionPlan.Segment("stop " + end, path, action);
	}

	private HybridAutoSupervisor supervisor(List<AutoMissionPlan.Segment> segments) {
		return supervisor(segments, () -> drive.pose);
	}

	private HybridAutoSupervisor supervisor(List<AutoMissionPlan.Segment> segments, Supplier<Pose2d> pose) {
		return new HybridAutoSupervisor(new AutoMissionPlan(segments), drive, pose, () -> drive.velocity,
				speed -> drive.velocity = speed, () -> true, () -> false, robot, new NoRoute(),
				new NavigationSafety(10, 8, 0.45, List.of()), ObstacleSource.empty(), AutoSafetyConfig.defaults());
	}

	private void tick() {
		drive.step();
		SimHooks.stepTiming(DT);
		scheduler.run();
	}

	@Test
	void rejectsAnArrivalActionThatCompetesForDrivetrainOwnership() {
		assertThrows(IllegalArgumentException.class, () -> supervisor(List.of(segment(2, 3, Commands.run(() -> {
		}, drive)))));
	}

	@Test
	void rejectedMissionDoesNotLeaveEarlierActionsRegisteredAsComposed() {
		int[] calls = {0};
		var first = Commands.runOnce(() -> calls[0]++);
		assertThrows(IllegalArgumentException.class,
				() -> supervisor(List.of(segment(2, 3, first), segment(3, 4, Commands.run(() -> {
				}, drive)))));
		assertDoesNotThrow(() -> scheduler.schedule(first));
		tick();
		assertEquals(1, calls[0], "A failed constructor must not retain ownership of earlier actions");
	}

	@Test
	void nullOrThrowingPoseAtInitializationStopsWithoutEscapingScheduler() {
		for (boolean throwing : new boolean[]{false, true}) {
			int[] actions = {0};
			Supplier<Pose2d> faulty = () -> {
				if (throwing)
					throw new IllegalStateException("pose source disconnected");
				return null;
			};
			drive.velocity = new ChassisSpeeds(1, 0, 0);
			var command = supervisor(List.of(segment(2, 3, Commands.runOnce(() -> actions[0]++))), faulty);
			assertDoesNotThrow(() -> scheduler.schedule(command));
			SimHooks.stepTiming(DT);
			assertDoesNotThrow(scheduler::run);
			assertEquals(HybridAutoSupervisor.NavigationState.SAFE_ABORT, command.state());
			assertFalse(command.isScheduled());
			assertEquals(0, command.completedObjectives());
			assertEquals(0, actions[0]);
			assertEquals(0, Math.hypot(drive.velocity.vxMetersPerSecond, drive.velocity.vyMetersPerSecond), 1e-9);
		}
	}

	@Test
	void nullOrThrowingPoseDuringExecutionStopsWithoutTelemetryRethrowing() {
		for (boolean throwing : new boolean[]{false, true}) {
			boolean[] failed = {false};
			int[] actions = {0};
			Supplier<Pose2d> faulty = () -> {
				if (!failed[0])
					return drive.pose;
				if (throwing)
					throw new IllegalStateException("pose source disconnected");
				return null;
			};
			drive.pose = new Pose2d(2, 2, Rotation2d.kZero);
			var command = supervisor(List.of(segment(2, 3, Commands.runOnce(() -> actions[0]++))), faulty);
			scheduler.schedule(command);
			for (int i = 0; i < 10; i++)
				tick();
			assertTrue(command.isScheduled(), command.reason());
			failed[0] = true;
			SimHooks.stepTiming(DT);
			assertDoesNotThrow(scheduler::run, "Telemetry and cleanup must tolerate the same failed pose source");
			assertEquals(HybridAutoSupervisor.NavigationState.SAFE_ABORT, command.state());
			assertFalse(command.isScheduled());
			assertEquals(0, command.completedObjectives());
			assertEquals(0, actions[0]);
			assertEquals(0, Math.hypot(drive.velocity.vxMetersPerSecond, drive.velocity.vyMetersPerSecond), 1e-9);
		}
	}

	@Test
	void completesTwoPhysicalDestinationsAndExecutesEachActionOnceInOrder() {
		List<Integer> events = new ArrayList<>();
		List<Double> actionPositions = new ArrayList<>();
		var first = Commands.runOnce(() -> {
			events.add(1);
			actionPositions.add(drive.pose.getX());
		});
		var second = Commands.runOnce(() -> {
			events.add(2);
			actionPositions.add(drive.pose.getX());
		});
		var command = supervisor(List.of(segment(2, 3, first), segment(3, 4, second)));
		scheduler.schedule(command);
		for (int i = 0; i < 1000 && command.isScheduled(); i++)
			tick();
		assertEquals(HybridAutoSupervisor.NavigationState.COMPLETE, command.state(), command.reason());
		assertEquals(2, command.completedObjectives());
		assertEquals(List.of(1, 2), events);
		assertEquals(3, actionPositions.get(0), 0.10);
		assertEquals(4, actionPositions.get(1), 0.10);
		for (int i = 0; i < 20; i++)
			tick();
		assertEquals(List.of(1, 2), events, "Completed actions must not repeat after command termination");
		assertEquals(0, Math.hypot(drive.velocity.vxMetersPerSecond, drive.velocity.vyMetersPerSecond), 1e-9);
	}

	@Test
	void cancellationDuringArrivalActionInterruptsItAndDoesNotCompleteOrAdvanceObjective() {
		int[] started = {0};
		int[] ended = {0};
		int[] next = {0};
		var action = Commands.startEnd(() -> started[0]++, () -> ended[0]++);
		var command = supervisor(List.of(segment(2, 3, action), segment(3, 4, Commands.runOnce(() -> next[0]++))));
		scheduler.schedule(command);
		for (int i = 0; i < 600 && command.isScheduled()
				&& command.state() != HybridAutoSupervisor.NavigationState.ACTION; i++)
			tick();
		assertEquals(HybridAutoSupervisor.NavigationState.ACTION, command.state(), command.reason());
		assertEquals(1, started[0]);
		scheduler.cancel(command);
		assertEquals(1, ended[0]);
		assertEquals(0, command.completedObjectives());
		assertEquals(0, next[0]);
		assertEquals(HybridAutoSupervisor.NavigationState.SAFE_ABORT, command.state());
		assertFalse(command.isScheduled());
		assertEquals(0, Math.hypot(drive.velocity.vxMetersPerSecond, drive.velocity.vyMetersPerSecond), 1e-9);
	}
	@Test
	void displacementDuringArrivalActionInterruptsWithoutMarkingItComplete() {
		int[] ended = {0};
		var action = Commands.startEnd(() -> {
		}, () -> ended[0]++);
		var command = supervisor(List.of(segment(2, 3, action)));
		scheduler.schedule(command);
		for (int i = 0; i < 600 && command.isScheduled()
				&& command.state() != HybridAutoSupervisor.NavigationState.ACTION; i++)
			tick();
		assertEquals(HybridAutoSupervisor.NavigationState.ACTION, command.state(), command.reason());
		// Below the discontinuity gate, but outside the independently verified arrival
		// envelope.
		drive.pose = new Pose2d(drive.pose.getX(), drive.pose.getY() + .16, drive.pose.getRotation());
		tick();
		assertEquals(HybridAutoSupervisor.NavigationState.SAFE_ABORT, command.state());
		assertEquals("Lost arrival pose while action active", command.reason());
		assertEquals(0, command.completedObjectives());
		assertEquals(1, ended[0]);
		assertEquals(0, drive.velocity.vxMetersPerSecond, 1e-9);
	}

	private HybridAutoDefinition commandTree(String tail) throws Exception {
		return HybridAutoDefinition.parse("""
				{"version":"2025.0","resetOdom":false,"choreoAuto":false,"command":{
				  "type":"sequential","data":{"commands":[
				    {"type":"deadline","data":{"commands":[
				      {"type":"path","data":{"pathName":"Test"}},
				      {"type":"named","data":{"name":"Intake"}}
				    ]}},
				    %s
				  ]}}}
				""".formatted(tail), name -> segment(2, 3, Commands.none()).path());
	}

	@Test
	void splitLoopVisitsBothEndsBeforeScoringWithoutRestartingIntake() throws Exception {
		var definition = HybridAutoDefinition.parse("""
				{"version":"2025.0","resetOdom":false,"choreoAuto":false,"command":{
				  "type":"sequential","data":{"commands":[
				    {"type":"deadline","data":{"commands":[
				      {"type":"sequential","data":{"commands":[
				        {"type":"path","data":{"pathName":"Out"}},
				        {"type":"path","data":{"pathName":"Return"}}]}},
				      {"type":"named","data":{"name":"Intake"}}]}},
				    {"type":"named","data":{"name":"Score"}}
				  ]}}}
				""",
				name -> name.equals("Out")
						? segment(2, 3, Commands.none()).path()
						: segment(3, 2, Commands.none()).path());
		var visits = new ArrayList<String>();
		int[] starts = {0};
		var command = new HybridCommandAuto(definition, name -> name.equals("Intake") ? Commands.run(() -> {
		}).beforeStarting(() -> starts[0]++) : Commands.runOnce(() -> {
			assertEquals(2, drive.pose.getX(), .1);
			visits.add("Score");
		}), name -> {
			var item = new AutoMissionPlan.Segment(name, definition.paths().get(name),
					Commands.runOnce(() -> visits.add(name)));
			return supervisor(List.of(item));
		}, drive, () -> true, () -> drive.velocity = new ChassisSpeeds(), 30);
		scheduler.schedule(command);
		for (int i = 0; i < 1000 && command.isScheduled(); i++)
			tick();
		assertFalse(command.isScheduled(), command.failure());
		assertEquals("", command.failure());
		assertEquals(List.of("Out", "Return", "Score"), visits);
		assertEquals(1, starts[0]);
	}

	@Test
	void commandTreeKeepsIntakeActiveAcrossRecoveryAndFeedsOnlyAfterArrival() throws Exception {
		int[] events = {0, 0, 0};
		var definition = commandTree("""
				{"type":"named","data":{"name":"Feed"}}
				""");
		var navigation = supervisor(List.of(segment(2, 3, Commands.none())));
		var command = new HybridCommandAuto(definition,
				name -> name.equals("Intake")
						? Commands.startEnd(() -> events[0]++, () -> events[1]++)
						: Commands.runOnce(() -> {
							assertEquals(3, drive.pose.getX(), .10);
							assertTrue(Math.hypot(drive.velocity.vxMetersPerSecond,
									drive.velocity.vyMetersPerSecond) < .15);
							events[2]++;
						}),
				name -> navigation, drive, () -> true, () -> drive.velocity = new ChassisSpeeds(), 30);
		for (int run = 0; run < 2; run++) {
			drive.pose = new Pose2d(2, 2.30, Rotation2d.kZero);
			scheduler.schedule(command);
			for (int i = 0; i < 700 && command.isScheduled(); i++) {
				tick();
			}
			assertFalse(command.isScheduled());
			assertEquals("", command.failure());
			assertTrue(navigation.recoveryAttempts() > 0);
			assertArrayEquals(new int[]{run + 1, run + 1, run + 1}, events);
		}
	}

	@Test
	void commandTreeAbortsWithoutFeedingAndInterruptsIntake() throws Exception {
		int[] events = {0, 0};
		var definition = commandTree("""
				{"type":"named","data":{"name":"Feed"}}
				""");
		var command = new HybridCommandAuto(definition, name -> name.equals("Intake") ? Commands.startEnd(() -> {
		}, () -> events[0]++) : Commands.runOnce(() -> events[1]++),
				name -> supervisor(List.of(segment(2, 3, Commands.none()))), drive, () -> true,
				() -> drive.velocity = new ChassisSpeeds(), 30);
		scheduler.schedule(command);
		tick();
		drive.pose = new Pose2d(6, 6, Rotation2d.kZero);
		tick();
		assertFalse(command.isScheduled());
		assertFalse(command.failure().isEmpty());
		assertArrayEquals(new int[]{1, 0}, events);
		assertEquals(0, drive.velocity.vxMetersPerSecond);
	}

	@Test
	void commandTreeChecksHealthBeforeStartingMechanismsAndCancelsOnGlobalDeadline() throws Exception {
		int[] starts = {0};
		var definition = commandTree("""
				{"type":"wait","data":{"waitTime":100}}
				""");
		var command = new HybridCommandAuto(definition, name -> Commands.runOnce(() -> starts[0]++),
				name -> supervisor(List.of(segment(2, 3, Commands.none()))), drive, () -> false,
				() -> drive.velocity = new ChassisSpeeds(), 30);
		scheduler.schedule(command);
		tick();
		assertFalse(command.isScheduled());
		assertEquals(0, starts[0]);

		var bounded = new HybridCommandAuto(definition, name -> Commands.none(),
				name -> supervisor(List.of(segment(2, 3, Commands.none()))), drive, () -> true,
				() -> drive.velocity = new ChassisSpeeds(), .10);
		scheduler.schedule(bounded);
		for (int i = 0; i < 20 && bounded.isScheduled(); i++)
			tick();
		assertFalse(bounded.isScheduled());
		assertEquals("Whole-auto deadline", bounded.failure());
		assertEquals(0, drive.velocity.vxMetersPerSecond);
	}
}
