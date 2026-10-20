package frc.robot.navigation;

import static org.junit.jupiter.api.Assertions.*;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.*;
import com.pathplanner.lib.pathfinding.LocalADStar;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.constants.drive.DrivetrainConstants;
import java.util.List;
import org.dyn4j.geometry.Vector2;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.*;
import org.junit.jupiter.api.Test;

/**
 * Native MapleSim tire/motor/inertia smoke test in a deliberately empty test
 * arena.
 */
class HybridMapleSimulationTest {
	@Test
	void physicalLateralImpulseWithIndependentOdometry() throws Exception {
		runPhysics(false, false, false);
	}

	@Test
	void sustainedPhysicalPushRequiresGeometricRecovery() throws Exception {
		runPhysics(true, false, false);
	}

	@Test
	void highSpeedCurvedSplineComparison() throws Exception {
		String baseline = runPhysics(false, true, true);
		String hybrid = runPhysics(false, true, false);
		var report = java.nio.file.Path.of("build/reports/hybrid/maple-high-speed.csv");
		java.nio.file.Files.createDirectories(report.getParent());
		java.nio.file.Files.writeString(report,
				"controller,endpoint_m,heading_deg,speed_mps,seconds,recovery_attempts,peak_speed_mps\n" + baseline
						+ hybrid);
	}

	private String runPhysics(boolean sustainedPush, boolean highSpeed, boolean baseline) throws Exception {
		HAL.initialize(500, 0);
		SimHooks.pauseTiming();
		var scheduler = CommandScheduler.getInstance();
		var previousArena = SimulatedArena.getInstance();
		var arena = new SimulatedArena(new SimulatedArena.FieldMap() {
		}) {
			@Override
			public void placeGamePiecesOnField() {
			}
		};
		SimulatedArena.overrideInstance(arena);
		var requirement = new SubsystemBase() {
		};
		try {
			DriverStationSim.setDsAttached(true);
			DriverStationSim.setEnabled(true);
			DriverStationSim.notifyNewData();
			DriverStation.refreshData();
			RoboRioSim.setVInVoltage(12.5);
			Pose2d start = new Pose2d(5, 4, new Rotation2d());
			var physics = new SwerveDriveSimulation(DrivetrainConstants.SIMULATION_CONFIG, start);
			arena.addDriveTrainSimulation(physics);
			var drive = new SelfControlledSwerveDriveSimulation(physics);
			var waypoints = highSpeed
					? PathPlannerPath.waypointsFromPoses(start, new Pose2d(8, 5, Rotation2d.fromDegrees(25)),
							new Pose2d(11, 4, Rotation2d.fromDegrees(-25)))
					: PathPlannerPath.waypointsFromPoses(start, new Pose2d(9, 4, new Rotation2d()));
			var destination = highSpeed ? new Translation2d(11, 4) : new Translation2d(9, 4);
			var path = new PathPlannerPath(waypoints, new PathConstraints(highSpeed ? 4 : 2, highSpeed ? 4 : 2, 3, 6),
					new IdealStartingState(0, new Rotation2d()), new GoalEndState(0, new Rotation2d()));
			int[] actions = {0};
			var supervisor = new HybridAutoSupervisor(
					new AutoMissionPlan(
							List.of(new AutoMissionPlan.Segment("maple", path, Commands.runOnce(() -> actions[0]++)))),
					requirement, drive::getOdometryEstimatedPose, () -> drive.getMeasuredSpeedsRobotRelative(true),
					speeds -> drive.runChassisSpeeds(speeds, new Translation2d(), false, true), () -> true, () -> false,
					RobotConfig.fromGUISettings(), new LocalADStar(), new NavigationSafety(17.55, 8.05, .45, List.of()),
					ObstacleSource.empty(),
					highSpeed
							? new AutoSafetyConfig(4, 4, 3, 6, .25, .5, .65, .2, .4, .5, 10, 2, 30, 4, .1,
									Math.toRadians(4), .15)
							: AutoSafetyConfig.defaults());
			Command command = baseline
					? new com.pathplanner.lib.commands.FollowPathCommand(path, drive::getOdometryEstimatedPose,
							() -> drive.getMeasuredSpeedsRobotRelative(true),
							(speeds, ff) -> drive.runChassisSpeeds(speeds, new Translation2d(), false, true),
							HybridAutoSupervisor.pathController(), RobotConfig.fromGUISettings(), () -> false,
							requirement)
					: supervisor;
			int[] cycle = {0};
			arena.addCustomSimulation(subTick -> {
				if (sustainedPush && cycle[0] >= 55 && cycle[0] < 115) {
					// Contact-like pusher applies bounded force at every physics substep.
					// Tire friction, wheel dynamics and estimator still evolve naturally.
					double dt = SimulatedArena.getSimulationDt().in(edu.wpi.first.units.Units.Seconds);
					double acceleration = Math.max(0, Math.min(40, (1.2 - physics.getLinearVelocity().y) / dt));
					physics.applyForce(new Vector2(0, physics.getMass().getMass() * acceleration));
				}
			});
			scheduler.schedule(command);
			double largestTruthEstimateSeparation = 0;
			int ticks = 0;
			double peakSpeed = 0;
			for (int i = 0; i < 1500 && command.isScheduled(); i++) {
				cycle[0] = i;
				ticks++;
				if (!highSpeed && !sustainedPush && i == 55)
					physics.applyImpulse(new Vector2(0, physics.getMass().getMass() * 1.8));
				arena.simulationPeriodic();
				SimHooks.stepTiming(.02);
				drive.periodic();
				// A synthetic absolute sensor at 50 Hz; wheel/gyro estimator remains
				// independent.
				drive.addVisionEstimation(physics.getSimulatedDriveTrainPose(),
						edu.wpi.first.wpilibj.Timer.getFPGATimestamp());
				scheduler.run();
				var actual = drive.getActualSpeedsFieldRelative();
				peakSpeed = Math.max(peakSpeed, Math.hypot(actual.vxMetersPerSecond, actual.vyMetersPerSecond));
				largestTruthEstimateSeparation = Math.max(largestTruthEstimateSeparation,
						drive.getActualPoseInSimulationWorld().getTranslation()
								.getDistance(drive.getOdometryEstimatedPose().getTranslation()));
			}
			// Identical passive physical braking interval for both followers.
			drive.runChassisSpeeds(new edu.wpi.first.math.kinematics.ChassisSpeeds(), new Translation2d(), false, true);
			for (int braking = 0; braking < 50 && Math.hypot(drive.getActualSpeedsFieldRelative().vxMetersPerSecond,
					drive.getActualSpeedsFieldRelative().vyMetersPerSecond) >= .15; braking++) {
				arena.simulationPeriodic();
				SimHooks.stepTiming(.02);
				drive.periodic();
				ticks++;
			}
			double endpoint = drive.getActualPoseInSimulationWorld().getTranslation().getDistance(destination);
			double heading = Math.abs(drive.getActualPoseInSimulationWorld().getRotation().getDegrees());
			var velocity = drive.getActualSpeedsFieldRelative();
			double speed = Math.hypot(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond);
			if (!baseline) {
				assertEquals(HybridAutoSupervisor.NavigationState.COMPLETE, supervisor.state(), supervisor.reason());
				assertEquals(1, actions[0]);
				assertTrue(endpoint < .15, "Measured arrival must correspond to physical arrival: " + endpoint);
				assertTrue(heading < 5, "Physical heading: " + heading);
				assertTrue(speed < .15, "Physical speed: " + speed);
				if (sustainedPush)
					assertTrue(supervisor.recoveryAttempts() > 0,
							"Sustained physical displacement must execute BLine recovery");
			}
			if (highSpeed)
				assertTrue(peakSpeed > 2.5,
						"High-speed trial must physically exceed ordinary 2 m/s envelope: " + peakSpeed);
			assertTrue(largestTruthEstimateSeparation > 0, "Estimator and ground truth must be distinct states");
			return String.format(java.util.Locale.ROOT, "%s,%.6f,%.6f,%.6f,%.3f,%d,%.6f%n",
					baseline ? "baseline" : "hybrid", endpoint, heading, speed, ticks * .02,
					baseline ? 0 : supervisor.recoveryAttempts(), peakSpeed);
		} finally {
			scheduler.cancelAll();
			scheduler.unregisterSubsystem(requirement);
			scheduler.clearComposedCommands();
			DriverStationSim.setEnabled(false);
			DriverStationSim.notifyNewData();
			DriverStation.refreshData();
			arena.shutDown();
			SimulatedArena.overrideInstance(previousArena);
			SimHooks.resumeTiming();
		}
	}
}
