package frc.robot.navigation;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinder;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.util.pathplanner.AdvancedPPHolonomicDriveController;
import frc.robot.util.mechanical_advantage.LoggedTunableNumber;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Exclusive command composition. No child is independently scheduled and no
 * recovery resets pose.
 */
public final class HybridAutoSupervisor extends Command {
	// Reuse tunable subscriptions across runs; never allocate NT subscribers per
	// recovered path.
	private static final LoggedTunableNumber NORMAL_P = new LoggedTunableNumber("Hybrid/Controller/TranslationP", 15);
	private static final LoggedTunableNumber NORMAL_I = new LoggedTunableNumber("Hybrid/Controller/TranslationI", 0);
	private static final LoggedTunableNumber NORMAL_D = new LoggedTunableNumber("Hybrid/Controller/TranslationD", 0);
	private static final LoggedTunableNumber ROTATION_P = new LoggedTunableNumber("Hybrid/Controller/RotationP", 10);
	private static final LoggedTunableNumber ROTATION_I = new LoggedTunableNumber("Hybrid/Controller/RotationI", 0);
	private static final LoggedTunableNumber ROTATION_D = new LoggedTunableNumber("Hybrid/Controller/RotationD", 0);
	static AdvancedPPHolonomicDriveController pathController() {
		return new AdvancedPPHolonomicDriveController(new PIDConstants(15, 0, 0), new PIDConstants(10, 0, 0), .02,
				NORMAL_P, NORMAL_I, NORMAL_D, ROTATION_P, ROTATION_I, ROTATION_D);
	}

	public enum NavigationState {
		INACTIVE, NORMAL_PATH, GEOMETRIC_RECOVERY, OBSTACLE_REROUTE, FINAL_ALIGNMENT, ACTION, COMPLETE, SAFE_ABORT
	}
	private final AutoMissionPlan mission;
	private final Subsystem drive;
	private final Supplier<Pose2d> pose;
	private final Supplier<ChassisSpeeds> speeds;
	private final BooleanSupplier localizationHealthy;
	private final BooleanSupplier flip;
	private final RobotConfig robotConfig;
	private final Pathfinder planner;
	private final NavigationSafety safety;
	private final ObstacleSource obstacles;
	private final AutoSafetyConfig config;
	private final NavigationOutput output;
	private final List<Command> owned = new ArrayList<>();
	private NavigationState state = NavigationState.INACTIVE;
	private String reason = "Not started";
	private Command child;
	private PathPlannerPath path;
	private PathTrackingMonitor route;
	private Pose2d goal, previousPose, target;
	private ChassisSpeeds commanded = new ChassisSpeeds();
	private ChassisSpeeds measuredSnapshot = new ChassisSpeeds();
	private ChassisSpeeds desiredVelocity = new ChassisSpeeds();
	private double started, stateStarted, lastTime, errorSince, stallSince, settledSince;
	private double crossTrack, lag, recoveryStarted, recoveryDuration;
	private int segment, completed, attempts, replans, transitions;
	private boolean flipped, waitingForPlanner, detour;
	private double recoveryHoldUntil, plannerUpdated, plannerLatency;
	private List<edu.wpi.first.math.Pair<Translation2d, Translation2d>> plannerSnapshot = List.of();

	public HybridAutoSupervisor(AutoMissionPlan mission, Subsystem drive, Supplier<Pose2d> pose,
			Supplier<ChassisSpeeds> speeds, Consumer<ChassisSpeeds> output, BooleanSupplier localizationHealthy,
			BooleanSupplier flip, RobotConfig robotConfig, Pathfinder planner, NavigationSafety safety,
			ObstacleSource obstacles, AutoSafetyConfig config) {
		this.mission = mission;
		this.drive = drive;
		this.pose = pose;
		this.speeds = speeds;
		this.localizationHealthy = localizationHealthy;
		this.flip = flip;
		this.robotConfig = robotConfig;
		this.planner = planner;
		this.safety = safety;
		this.obstacles = obstacles;
		this.config = config;
		this.output = new NavigationOutput(config, output);
		addRequirements(drive);
		for (var item : mission.segments()) {
			if (item.arrivalAction().getRequirements().contains(drive))
				throw new IllegalArgumentException("Arrival actions must not own drivetrain");
			addRequirements(item.arrivalAction().getRequirements());

		}
		CommandScheduler.getInstance().registerComposedCommands(
				mission.segments().stream().map(AutoMissionPlan.Segment::arrivalAction).toArray(Command[]::new));
	}

	@Override
	public void initialize() {
		segment = completed = attempts = replans = transitions = 0;
		started = lastTime = Timer.getFPGATimestamp();
		errorSince = stallSince = settledSince = Double.NaN;
		previousPose = null;
		recoveryHoldUntil = 0;
		child = null;
		waitingForPlanner = false;
		recoveryDuration = 0;
		plannerLatency = 0;
		recoveryStarted = 0;
		state = NavigationState.INACTIVE;
		try {
			previousPose = pose.get();
			if (!validPose(previousPose) || !NavigationOutput.finite(speeds.get())
					|| !localizationHealthy.getAsBoolean()) {
				abort("Localization unavailable at start");
				return;
			}
			flipped = flip.getAsBoolean();
			output.reset(speeds.get(), previousPose.getRotation());
			safety.update(obstacles.observations(started), started);
			if (!safety.safePoint(previousPose.getTranslation())) {
				abort("Unsafe initial geometry");
				return;
			}
			beginSegment();
		} catch (RuntimeException ex) {
			abort("Initialization: " + ex.getMessage());
		}
	}

	private void beginSegment() {
		detour = false;
		recoveryStarted = 0;
		path = mission.segments().get(segment).path();
		output.setTranslationLimit(path.getGlobalConstraints().maxVelocityMPS());
		if (flipped && !path.preventFlipping)
			path = path.flipPath();
		goal = new Pose2d(path.getPoint(path.numPoints() - 1).position, path.getGoalEndState().rotation());
		route = new PathTrackingMonitor(path.getPathPoses());
		target = pose.get();
		if (!safety.safePoint(goal.getTranslation())) {
			abort("Unsafe mission destination");
			return;
		}
		if (!remainingSafe()) {
			reroute("Obstructed mission corridor");
			return;
		}
		if (route.project(pose.get().getTranslation()).crossTrack() > config.crossTrackTrigger()
				|| pose.get().getTranslation().getDistance(path.getPoint(0).position) > .35) {
			recover("Measured start differs from authored start");
			return;
		}
		transition(NavigationState.NORMAL_PATH, "Begin authored segment");
		follow(path);
	}

	private void follow(PathPlannerPath selected) {
		var controller = pathController();
		PathFollowingController observed = new PathFollowingController() {
			public void reset(Pose2d p, ChassisSpeeds s) {
				controller.reset(p, s);
			}
			public boolean isHolonomic() {
				return true;
			}
			public ChassisSpeeds calculateRobotRelativeSpeeds(Pose2d p, PathPlannerTrajectoryState desired) {
				target = desired.pose;
				desiredVelocity = desired.fieldSpeeds;
				return controller.calculateRobotRelativeSpeeds(p, desired);
			}
		};
		startChild(new FollowPathCommand(selected, pose, speeds, (s, ff) -> output.request(s), observed, robotConfig,
				() -> false, drive), true);
		Logger.recordOutput("Hybrid/Route", selected.getPathPoses().toArray(Pose2d[]::new));
	}

	@Override
	public void execute() {
		long cpuStart = System.nanoTime();
		try {
			if (isFinished()) {
				stopOutput();
				return;
			}
			double now = Timer.getFPGATimestamp();
			double dt = now - lastTime;
			Pose2d current = pose.get();
			var measured = speeds.get();
			measuredSnapshot = measured;
			if (!validPose(current) || !NavigationOutput.finite(measured) || !localizationHealthy.getAsBoolean()
					|| dt <= 0 || dt > .1) {
				abort("Invalid localization or control clock");
				return;
			}
			if (current.getTranslation().getDistance(previousPose.getTranslation()) > config.maxSpeed() * dt + .3
					|| Math.abs(current.getRotation().minus(previousPose.getRotation())
							.getRadians()) > config.maxOmega() * dt + .3) {
				abort("Discontinuous localization; reacquire before retry");
				return;
			}
			lastTime = now;
			previousPose = current;
			safety.update(obstacles.observations(now), now);
			if (!safety.healthy() || !safety.safePoint(current.getTranslation())) {
				abort("Invalid obstacles or occupied robot footprint");
				return;
			}
			if (now - started > config.missionTimeout() || now - stateStarted > config.modeTimeout()) {
				abort("Mission or state deadline");
				return;
			}
			var projection = route.project(current.getTranslation());
			crossTrack = projection.crossTrack();
			lag = target == null ? 0 : route.project(target.getTranslation()).progress() - projection.progress();
			if (state == NavigationState.ACTION) {
				if (!atGoal(current, measured)) {
					abort("Lost arrival pose while action active");
					return;
				}
				output.request(new ChassisSpeeds());
				child.execute();
				if (child.isFinished()) {
					endChild(false);
					completed++;
					if (++segment == mission.segments().size())
						transition(NavigationState.COMPLETE, "All objectives completed");
					else
						beginSegment();
				}
			} else if (waitingForPlanner) {
				output.request(new ChassisSpeeds());
				if (now - stateStarted > config.plannerTimeout()) {
					abort("AD* route deadline");
					return;
				}
				if (now - plannerUpdated >= config.dwellSeconds()
						&& !plannerSnapshot.equals(safety.dynamicObstacles())) {
					planner.setDynamicObstacles(safety.plannerObstacles(), current.getTranslation());
					planner.setStartPosition(current.getTranslation());
					planner.setGoalPosition(goal.getTranslation());
					plannerSnapshot = safety.dynamicObstacles();
					plannerUpdated = now;
					replans++;
				}
				if (planner.isNewPathAvailable()) {
					var candidate = planner.getCurrentPath(constraints(), new GoalEndState(0, goal.getRotation()));
					if (candidate != null && candidate.numPoints() >= 2 && safeRoute(candidate.getPathPoses())
							&& candidate.getPoint(candidate.numPoints() - 1).position
									.getDistance(goal.getTranslation()) < .15) {
						waitingForPlanner = false;
						plannerLatency = now - stateStarted;
						route = new PathTrackingMonitor(candidate.getPathPoses());
						follow(candidate);
					}
				}
			} else {
				if (!remainingSafe()) {
					reroute("Remaining corridor obstructed");
				} else if (recoveryHoldUntil == 0) {
					boolean stalled = Math.hypot(commanded.vxMetersPerSecond, commanded.vyMetersPerSecond) > .8
							&& Math.hypot(measured.vxMetersPerSecond, measured.vyMetersPerSecond) < .15;
					stallSince = stalled ? (Double.isNaN(stallSince) ? now : stallSince) : Double.NaN;
					boolean disturbed = crossTrack > config.crossTrackTrigger() || lag > config.lagTrigger();
					// Release below a smaller threshold, so noisy measurements do not erase
					// sustained errors.
					if (disturbed && Double.isNaN(errorSince))
						errorSince = now;
					if (crossTrack < config.crossTrackTrigger() * .6 && lag < config.lagTrigger() * .6)
						errorSince = Double.NaN;
					if (!Double.isNaN(stallSince) && now - stallSince >= config.stallSeconds()) {
						// No reliable obstacle explanation: one bounded retry, then stop pushing the
						// same element.
						if (state != NavigationState.NORMAL_PATH) {
							abort("Persistent stall during recovery");
							return;
						}
						recover("Sustained no-progress condition; bounded clearance wait", true);
					} else if ((state == NavigationState.NORMAL_PATH || state == NavigationState.OBSTACLE_REROUTE)
							&& now - stateStarted >= config.dwellSeconds() && (crossTrack > config.severeError()
									|| (!Double.isNaN(errorSince) && now - errorSince >= config.debounce()))) {
						recover("Geometric error or along-track lag");
					}
				}
				if (!isFinished() && !waitingForPlanner) {
					if (recoveryHoldUntil > 0) {
						output.request(new ChassisSpeeds());
						if (now >= recoveryHoldUntil) {
							recoveryHoldUntil = 0;
							startGeometricPath();
						}
					} else if (state == NavigationState.FINAL_ALIGNMENT)
						align(now);
					else if (child != null) {
						child.execute();
						if (child.isFinished()) {
							endChild(false);
							if (pose.get().getTranslation().getDistance(goal.getTranslation()) > .25)
								recover("Timed follower ended before physical arrival");
							else
								transition(NavigationState.FINAL_ALIGNMENT,
										"Follower ended; validate measured arrival");
						}
					}
				}
			}
			if (isFinished())
				stopOutput();
			else {
				commanded = output.calculate(current.getRotation(), dt);
				// Check short stopping corridor, not only the authored route.
				var field = ChassisSpeeds.fromRobotRelativeSpeeds(commanded, current.getRotation());
				double speed = Math.hypot(field.vxMetersPerSecond, field.vyMetersPerSecond);
				double horizon = .1 + speed / (2 * config.maxAcceleration());
				if (!safety.safeSegment(current.getTranslation(), current.getTranslation()
						.plus(new Translation2d(field.vxMetersPerSecond, field.vyMetersPerSecond).times(horizon)))) {
					abort("Unsafe commanded stopping corridor");
				} else {
					var physical = ChassisSpeeds.fromRobotRelativeSpeeds(measured, current.getRotation());
					double actualSpeed = Math.hypot(physical.vxMetersPerSecond, physical.vyMetersPerSecond);
					var stop = current.getTranslation()
							.plus(new Translation2d(physical.vxMetersPerSecond, physical.vyMetersPerSecond)
									.times(.1 + actualSpeed / (2 * config.maxAcceleration())));
					if (!safety.safeSegment(current.getTranslation(), stop))
						abort("Unsafe measured stopping corridor");
					else
						output.publish(commanded);
				}
			}
		} catch (RuntimeException ex) {
			abort("Controller fault: " + ex.getClass().getSimpleName() + ": " + ex.getMessage());
		} finally {
			log((System.nanoTime() - cpuStart) / 1e6);
		}
	}

	private void align(double now) {
		var current = pose.get();
		var delta = goal.getTranslation().minus(current.getTranslation());
		double angle = goal.getRotation().minus(current.getRotation()).getRadians();
		output.request(ChassisSpeeds.fromFieldRelativeSpeeds(delta.getX() * 3, delta.getY() * 3, angle * 4,
				current.getRotation()));
		var measured = speeds.get();
		boolean arrived = atGoal(current, measured);
		settledSince = arrived ? (Double.isNaN(settledSince) ? now : settledSince) : Double.NaN;
		if (arrived && now - settledSince >= .2) {
			recoveryDuration = recoveryStarted > 0 ? now - recoveryStarted : 0;
			transition(NavigationState.ACTION, "Measured objective reached");
			startChild(mission.segments().get(segment).arrivalAction(), false);
			output.request(new ChassisSpeeds());
		}
	}

	private boolean atGoal(Pose2d current, ChassisSpeeds measured) {
		return goal.getTranslation().getDistance(current.getTranslation()) < config.positionTolerance()
				&& Math.abs(goal.getRotation().minus(current.getRotation()).getRadians()) < config.headingTolerance()
				&& Math.hypot(measured.vxMetersPerSecond, measured.vyMetersPerSecond) < config.speedTolerance()
				&& Math.abs(measured.omegaRadiansPerSecond) < .15;
	}
	private boolean remainingSafe() {
		var points = route.poses();
		int edge = route.project(pose.get().getTranslation()).edge();
		Translation2d previous = pose.get().getTranslation();
		for (int i = edge + 1; i < points.size(); i++) {
			var next = points.get(i).getTranslation();
			if (!safety.safeSegment(previous, next))
				return false;
			previous = next;
		}
		return true;
	}
	private boolean safeRoute(List<Pose2d> points) {
		Translation2d previous = pose.get().getTranslation();
		for (var point : points) {
			if (!validPose(point) || !safety.safeSegment(previous, point.getTranslation()))
				return false;
			previous = point.getTranslation();
		}
		return true;
	}
	private void recover(String why) {
		recover(why, false);
	}
	private void recover(String why, boolean waitForClearance) {
		if (++attempts > config.maxAttempts()) {
			abort("Recovery retry limit");
			return;
		}
		if (!remainingSafe()) {
			reroute("Recovery corridor unsafe");
			return;
		}
		endChild(true);
		transition(NavigationState.GEOMETRIC_RECOVERY, why);
		target = goal;
		desiredVelocity = new ChassisSpeeds();
		recoveryStarted = Timer.getFPGATimestamp();
		if (waitForClearance) {
			recoveryHoldUntil = recoveryStarted + AutoSafetyConfig.STALL_CLEARANCE_WAIT_SECONDS;
			output.request(new ChassisSpeeds());
		} else
			startGeometricPath();
	}
	private void startGeometricPath() {
		var elements = new ArrayList<Path.PathElement>();
		elements.add(new Path.Waypoint(pose.get(), .05));
		int edge = route.project(pose.get().getTranslation()).edge();
		var points = route.poses();
		for (int i = edge + 1; i < points.size() - 1; i++) {
			var p = points.get(i);
			elements.add(new Path.TranslationTarget(p.getX(), p.getY(), .06));
		}
		elements.add(new Path.Waypoint(goal, .05));
		var bounded = constraints();
		var limits = new Path.PathConstraints().setMaxVelocityMetersPerSec(bounded.maxVelocityMPS())
				.setMaxAccelerationMetersPerSec2(bounded.maxAccelerationMPSSq())
				.setMaxVelocityDegPerSec(Math.toDegrees(bounded.maxAngularVelocityRadPerSec()))
				.setMaxAccelerationDegPerSec2(Math.toDegrees(bounded.maxAngularAccelerationRadPerSecSq()))
				.setEndTranslationToleranceMeters(.06).setEndRotationToleranceDeg(3);
		var recovery = new Path(elements, limits,
				new Path.DefaultGlobalConstraints(config.maxSpeed(), config.maxAcceleration(),
						Math.toDegrees(config.maxOmega()), Math.toDegrees(config.maxAlpha()), .06, 3, .06));
		var builder = new FollowPath.Builder(drive, pose, speeds, output::request, new PIDController(2, 0, 0),
				new PIDController(4, 0, 0), new PIDController(3, 0, 0)).withTRatioBasedTranslationHandoffs(true);
		startChild(builder.build(recovery), true);
		Logger.recordOutput("Hybrid/RecoveryRoute", recovery.getTranslations().toArray(Translation2d[]::new));
	}
	private void reroute(String why) {
		double now = Timer.getFPGATimestamp();
		if (state == NavigationState.OBSTACLE_REROUTE && now - stateStarted < config.dwellSeconds()) {
			abort("Route obstructed inside replan cooldown");
			return;
		}
		if (++attempts > config.maxAttempts()) {
			abort("Replan retry limit");
			return;
		}
		if (!safety.safePoint(goal.getTranslation())) {
			abort("Destination occupied");
			return;
		}
		endChild(true);
		transition(NavigationState.OBSTACLE_REROUTE, why);
		detour = true;
		output.setTranslationLimit(
				Math.min(AutoSafetyConfig.DETOUR_MAX_SPEED, path.getGlobalConstraints().maxVelocityMPS()));
		replans++;
		waitingForPlanner = true;
		recoveryHoldUntil = 0;
		plannerUpdated = now;
		plannerSnapshot = safety.dynamicObstacles();
		output.request(new ChassisSpeeds());
		// Drain an older result before issuing a new request. Every result is still
		// validated locally.
		if (planner.isNewPathAvailable())
			planner.getCurrentPath(constraints(), new GoalEndState(0, goal.getRotation()));
		planner.setDynamicObstacles(safety.plannerObstacles(), pose.get().getTranslation());
		planner.setStartPosition(pose.get().getTranslation());
		planner.setGoalPosition(goal.getTranslation());
	}
	private PathConstraints constraints() {
		double v = Math.min(config.maxSpeed(), detour ? AutoSafetyConfig.DETOUR_MAX_SPEED : config.maxSpeed()),
				a = config.maxAcceleration(), w = config.maxOmega(), alpha = config.maxAlpha();
		var all = new ArrayList<PathConstraints>();
		all.add(path.getGlobalConstraints());
		for (var zone : path.getConstraintZones())
			all.add(zone.constraints());
		for (var c : all) {
			v = Math.min(v, c.maxVelocityMPS());
			a = Math.min(a, c.maxAccelerationMPSSq());
			w = Math.min(w, c.maxAngularVelocityRadPerSec());
			alpha = Math.min(alpha, c.maxAngularAccelerationRadPerSecSq());
		}
		return new PathConstraints(v, a, w, alpha);
	}
	private void startChild(Command next, boolean register) {
		endChild(true);
		if (!getRequirements().containsAll(next.getRequirements()))
			throw new IllegalArgumentException("Unexpected child requirements");
		if (register) {
			CommandScheduler.getInstance().registerComposedCommands(next);
			owned.add(next);
		}
		child = next;
		child.initialize();
	}
	private void endChild(boolean interrupted) {
		if (child != null) {
			var old = child;
			child = null;
			old.end(interrupted);
		}
	}
	private void transition(NavigationState next, String why) {
		state = next;
		reason = why;
		stateStarted = Timer.getFPGATimestamp();
		transitions++;
		errorSince = stallSince = settledSince = Double.NaN;
	}
	private void stopOutput() {
		commanded = new ChassisSpeeds();
		output.stop();
	}
	private void abort(String why) {
		try {
			endChild(true);
		} finally {
			waitingForPlanner = false;
			transition(NavigationState.SAFE_ABORT, why);
			stopOutput();
		}
	}
	@Override
	public boolean isFinished() {
		return state == NavigationState.COMPLETE || state == NavigationState.SAFE_ABORT;
	}
	@Override
	public void end(boolean interrupted) {
		try {
			endChild(true);
		} finally {
			stopOutput();
			waitingForPlanner = false;
			if (interrupted)
				transition(NavigationState.SAFE_ABORT, "Supervisor interrupted/disabled");
			for (var command : owned)
				CommandScheduler.getInstance().removeComposedCommand(command);
			owned.clear();
			log(0);
		}
	}
	public NavigationState state() {
		return state;
	}
	/** Reset an inactive leaf's status before its enclosing auto is reused. */
	void prepareForAuto() {
		state = NavigationState.INACTIVE;
		completed = attempts = 0;
	}
	public String reason() {
		return reason;
	}
	public int completedObjectives() {
		return completed;
	}
	public double recoveryDurationSeconds() {
		return recoveryDuration;
	}
	public double plannerLatencySeconds() {
		return plannerLatency;
	}
	public int recoveryAttempts() {
		return attempts;
	}
	private static boolean validPose(Pose2d p) {
		return p != null && Double.isFinite(p.getX()) && Double.isFinite(p.getY())
				&& Double.isFinite(p.getRotation().getRadians());
	}
	private void log(double elapsedMs) {
		Logger.recordOutput("Hybrid/State", state.toString());
		Logger.recordOutput("Hybrid/Reason", reason);
		Logger.recordOutput("Hybrid/Segment", segment);
		Logger.recordOutput("Hybrid/Objective",
				mission.segments().get(Math.min(segment, mission.segments().size() - 1)).name());
		Logger.recordOutput("Hybrid/PendingActions", mission.segments().size() - completed);
		Logger.recordOutput("Hybrid/CompletedObjectives", completed);
		var loggedPose = previousPose;
		if (validPose(loggedPose))
			Logger.recordOutput("Hybrid/Pose", loggedPose);
		if (target != null) {
			Logger.recordOutput("Hybrid/Target", target);
			if (validPose(loggedPose))
				Logger.recordOutput("Hybrid/TrackingHeadingError",
						target.getRotation().minus(loggedPose.getRotation()).getRadians());
		}
		if (goal != null && validPose(loggedPose)) {
			Logger.recordOutput("Hybrid/FinalTranslationError",
					loggedPose.getTranslation().getDistance(goal.getTranslation()));
			Logger.recordOutput("Hybrid/HeadingError", goal.getRotation().minus(loggedPose.getRotation()).getRadians());
		}
		Logger.recordOutput("Hybrid/DesiredFieldSpeeds", desiredVelocity);
		Logger.recordOutput("Hybrid/CrossTrack", crossTrack);
		Logger.recordOutput("Hybrid/AlongTrackLag", lag);
		Logger.recordOutput("Hybrid/CommandedSpeeds", commanded);
		var loggedSpeeds = measuredSnapshot;
		if (NavigationOutput.finite(loggedSpeeds))
			Logger.recordOutput("Hybrid/MeasuredSpeeds", loggedSpeeds);
		Logger.recordOutput("Hybrid/Attempts", attempts);
		Logger.recordOutput("Hybrid/Replans", replans);
		Logger.recordOutput("Hybrid/Transitions", transitions);
		Logger.recordOutput("Hybrid/RecoveryDuration", recoveryDuration);
		Logger.recordOutput("Hybrid/PlannerLatency", plannerLatency);
		Logger.recordOutput("Hybrid/Elapsed", Timer.getFPGATimestamp() - started);
		Logger.recordOutput("Hybrid/LoopMs", elapsedMs);
		Logger.recordOutput("Hybrid/LoopOverrun", elapsedMs > 20);
	}
}
