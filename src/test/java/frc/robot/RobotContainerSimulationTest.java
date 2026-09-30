package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.subsystems.IntakeConstants;
import frc.robot.constants.subsystems.ShooterConstants;
import frc.robot.commands.shooter.ShooterCalculator;
import frc.robot.commands.shooter.ShotSolution;
import frc.robot.simulation.RobotSimulation;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.position_joint.PositionJoint;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * End-to-end desktop simulation checks for the real driver controller bindings.
 */
class RobotContainerSimulationTest {
	private static final double VOLTAGE_TOLERANCE = 1e-6;
	private static final int SETTLE_CYCLES = 10;

	private final CommandScheduler scheduler = CommandScheduler.getInstance();
	private RobotContainer container;
	private XboxControllerSim controller;
	private Flywheel shooter;
	private Flywheel intake;
	private Flywheel conveyor;
	private Flywheel bottomIndexer;
	private Flywheel topIndexer;
	private PositionJoint intakeRack;
	private PositionJoint hood;

	@BeforeEach
	void setUpFullRobotSimulation() throws ReflectiveOperationException {
		HAL.initialize(500, 0);
		RobotSimulation.configureArenaOverride(frc.robot.constants.Constants.Mode.SIM);

		DriverStationSim.resetData();
		DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
		DriverStationSim.setDsAttached(true);
		DriverStationSim.setAutonomous(false);
		DriverStationSim.setEnabled(true);

		controller = new XboxControllerSim(0);
		controller.setAxisCount(6);
		controller.setButtonCount(10);
		controller.setPOVCount(1);
		controller.notifyNewData();
		DriverStation.refreshData();

		container = new RobotContainer();
		shooter = getField(container, "shooterFlywheel", Flywheel.class);
		intake = getField(container, "intakeRoller", Flywheel.class);
		conveyor = getField(container, "conveyor", Flywheel.class);
		bottomIndexer = getField(container, "bottomIndexer", Flywheel.class);
		topIndexer = getField(container, "topIndexer", Flywheel.class);
		intakeRack = getField(container, "intakeRack", PositionJoint.class);
		hood = getField(container, "hood", PositionJoint.class);
		runCycles(SETTLE_CYCLES);
	}

	@AfterEach
	void cleanUpSimulation() {
		scheduler.cancelAll();
		scheduler.unregisterAllSubsystems();
		DriverStationSim.setEnabled(false);
		DriverStationSim.notifyNewData();
		DriverStation.refreshData();
	}

	@Test
	void driverBindingsOperateAndReturnEveryRollerToNeutral() {
		verifyDriveBindingMovesRobot();
		verifyShooterPresetStopsOnRelease();
		verifyAcquireDeploysAndStopsOnRelease();
		verifyForceFeedRequiresCoordinatedShot();
		verifyEjectStopsOnRelease();
		verifyAutonomousShotTimesOutSafely();
	}

	private void verifyDriveBindingMovesRobot() {
		Pose2d startingPose = container.drivetrain.getPhysicsPose();
		controller.setLeftY(-0.65);
		runCycles(60);
		controller.setLeftY(0.0);
		runCycles(SETTLE_CYCLES);

		Pose2d endingPose = container.drivetrain.getPhysicsPose();
		assertTrue(endingPose.getTranslation().getDistance(startingPose.getTranslation()) > 0.05,
				() -> "Drive binding did not move the simulated robot: start=" + startingPose + ", end=" + endingPose);
	}

	private void verifyShooterPresetStopsOnRelease() {
		controller.setXButton(true);
		runCycles(SETTLE_CYCLES);
		assertEquals(ShooterConstants.SHOOTER_PRESET.HUB.get(), shooter.getVelocitySetpoint(), 1e-6);
		assertTrue(Math.abs(shooter.getVelocity()) > 1.0, "The simulated shooter did not physically spin up");

		controller.setXButton(false);
		runCycles(SETTLE_CYCLES);
		assertNeutral(shooter, "Shooter after X release");
		assertEquals(ShooterConstants.HOOD_PRESET.STOW.get(), hood.getDesiredPosition(), 1e-6,
				() -> "Hood should return to its default safe position after X is released; current command="
						+ (hood.getCurrentCommand() == null ? "none" : hood.getCurrentCommand().getName())
						+ ", default active=" + (hood.getCurrentCommand() == hood.getDefaultCommand()) + ", goal="
						+ positionGoal(hood));
	}

	private void verifyAcquireDeploysAndStopsOnRelease() {
		controller.setLeftBumperButton(true);
		runCycles(SETTLE_CYCLES);

		assertEquals(IntakeConstants.RACK_PRESETS.DEPLOY.get(), intakeRack.getDesiredPosition(), 1e-6);
		assertOutputActive(intake, "Intake roller while left bumper is held");
		assertOutputActive(conveyor, "Conveyor while left bumper is held");
		assertOutputActive(bottomIndexer, "Bottom indexer while left bumper is held");
		assertOutputActive(topIndexer, "Top indexer while left bumper is held");

		controller.setLeftBumperButton(false);
		runCycles(SETTLE_CYCLES);
		assertNeutral(intake, "Intake roller after left bumper release");
		assertNeutral(conveyor, "Conveyor after left bumper release");
		assertNeutral(bottomIndexer, "Bottom indexer after left bumper release");
		assertNeutral(topIndexer, "Top indexer after left bumper release");
		assertEquals(IntakeConstants.RACK_PRESETS.DEPLOY.get(), intakeRack.getDesiredPosition(), 1e-6,
				"The intake rack should remain deployed after acquisition is released");
	}

	private void verifyForceFeedRequiresCoordinatedShot() {
		controller.setRightBumperButton(true);
		runCycles(SETTLE_CYCLES);
		assertNeutral(bottomIndexer, "Bottom indexer with right bumper alone");
		assertNeutral(topIndexer, "Top indexer with right bumper alone");
		assertNeutral(conveyor, "Conveyor with right bumper alone");

		controller.setAButton(true);
		runCycles(SETTLE_CYCLES);
		assertTrue(shooter.getVelocitySetpoint() > 0.0, "A should prepare the shooter");
		assertOutputActive(bottomIndexer, "Bottom indexer during A + right-bumper force feed");
		assertOutputActive(topIndexer, "Top indexer during A + right-bumper force feed");
		assertOutputActive(conveyor, "Conveyor during A + right-bumper force feed");
		assertTrue(controller.getRumble(GenericHID.RumbleType.kBothRumble) > 0.0,
				"Force-feed readiness should provide driver rumble");

		controller.setRightBumperButton(false);
		runCycles(SETTLE_CYCLES);
		assertTrue(shooter.getVelocitySetpoint() > 0.0, "A should keep the shooter prepared after feed stops");
		assertNeutral(bottomIndexer, "Bottom indexer after force-feed release");
		assertNeutral(topIndexer, "Top indexer after force-feed release");
		assertNeutral(conveyor, "Conveyor after force-feed release");

		controller.setAButton(false);
		runCycles(SETTLE_CYCLES);
		assertNeutral(shooter, "Shooter after A release");
		assertNeutral(bottomIndexer, "Bottom indexer after A release");
		assertNeutral(topIndexer, "Top indexer after A release");
		assertNeutral(conveyor, "Conveyor after A release");
		assertEquals(0.0, controller.getRumble(GenericHID.RumbleType.kBothRumble), VOLTAGE_TOLERANCE);
		assertEquals(ShooterConstants.HOOD_PRESET.STOW.get(), hood.getDesiredPosition(), 1e-6);
	}

	private void verifyEjectStopsOnRelease() {
		controller.setLeftTriggerAxis(1.0);
		runCycles(SETTLE_CYCLES);
		assertOutputActive(intake, "Intake roller while eject is held");
		assertOutputActive(conveyor, "Conveyor while eject is held");
		assertOutputActive(bottomIndexer, "Bottom indexer while eject is held");
		assertOutputActive(topIndexer, "Top indexer while eject is held");

		controller.setLeftTriggerAxis(0.0);
		runCycles(SETTLE_CYCLES);
		assertNeutral(intake, "Intake roller after eject release");
		assertNeutral(conveyor, "Conveyor after eject release");
		assertNeutral(bottomIndexer, "Bottom indexer after eject release");
		assertNeutral(topIndexer, "Top indexer after eject release");
	}

	private void verifyAutonomousShotTimesOutSafely() {
		Translation2d hub = FieldConstants.Hub.hubPosition();
		container.drivetrain.resetPose(new Pose2d(hub.plus(new Translation2d(1.3, 0.0)), Rotation2d.kPi));
		container.drivetrain.stop();
		runCycles(50);

		DriverStationSim.setAutonomous(true);
		DriverStationSim.notifyNewData();
		DriverStation.refreshData();
		Command auto = container.getAutonomousCommand();
		assertEquals("AutonomousHubShot", auto.getName(),
				"The configured default auto should use coordinated shooting");
		scheduler.schedule(auto);
		boolean feedObserved = false;
		boolean debouncedReadyObserved = false;
		boolean feedingTelemetryObserved = false;
		int lastScheduledCycle = -1;
		Set<String> readinessStates = new LinkedHashSet<>();
		List<String> readinessTransitions = new ArrayList<>();
		String previousReadiness = "";
		double bestMotionRatio = Double.POSITIVE_INFINITY;
		double bestHeadingErrorDegrees = Double.POSITIVE_INFINITY;
		double bestFlywheelError = Double.POSITIVE_INFINITY;
		double bestHoodError = Double.POSITIVE_INFINITY;
		double bestScheduledFlywheelError = Double.POSITIVE_INFINITY;
		double bestScheduledHoodError = Double.POSITIVE_INFINITY;
		double maxScheduledFlywheelVelocity = 0.0;
		for (int i = 0; i < 175; i++) {
			runCycles(1);
			debouncedReadyObserved |= SmartDashboard.getBoolean("Shooter/Ready", false);
			feedingTelemetryObserved |= SmartDashboard.getBoolean("Shooter/Feeding", false);
			if (auto.isScheduled()) {
				lastScheduledCycle = i;
			}
			String readiness = SmartDashboard.getString("Shooter/Readiness", "missing");
			readinessStates.add(readiness);
			var state = container.drivetrain.getState();
			ShotSolution solution = ShooterCalculator.calculate(state.Pose);
			if (!readiness.equals(previousReadiness)) {
				readinessTransitions.add(String.format("%.2fs %s (flywheel %.2f/%.2f, hood %.4f/%.4f)", i * 0.02,
						readiness, shooter.getVelocity(), solution.flywheelRotationsPerSecond(), hood.getPosition(),
						solution.hoodRotations()));
				previousReadiness = readiness;
			}
			double translationSpeed = Math.hypot(state.Speeds.vxMetersPerSecond, state.Speeds.vyMetersPerSecond);
			bestMotionRatio = Math.min(bestMotionRatio,
					Math.max(translationSpeed / ShooterConstants.MAX_SHOOTING_TRANSLATION_METERS_PER_SECOND,
							Math.abs(state.Speeds.omegaRadiansPerSecond)
									/ ShooterConstants.MAX_SHOOTING_ROTATION_RADIANS_PER_SECOND));
			bestHeadingErrorDegrees = Math.min(bestHeadingErrorDegrees,
					Math.abs(state.Pose.getRotation().minus(solution.targetHeading()).getDegrees()));
			bestFlywheelError = Math.min(bestFlywheelError,
					Math.abs(shooter.getVelocity() - solution.flywheelRotationsPerSecond()));
			bestHoodError = Math.min(bestHoodError, Math.abs(hood.getPosition() - solution.hoodRotations()));
			if (auto.isScheduled()) {
				bestScheduledFlywheelError = Math.min(bestScheduledFlywheelError,
						Math.abs(shooter.getVelocity() - solution.flywheelRotationsPerSecond()));
				bestScheduledHoodError = Math.min(bestScheduledHoodError,
						Math.abs(hood.getPosition() - solution.hoodRotations()));
				maxScheduledFlywheelVelocity = Math.max(maxScheduledFlywheelVelocity, Math.abs(shooter.getVelocity()));
			}
			feedObserved |= Math.abs(bottomIndexer.getCommandedVoltage()) > VOLTAGE_TOLERANCE
					&& Math.abs(topIndexer.getCommandedVoltage()) > VOLTAGE_TOLERANCE
					&& Math.abs(conveyor.getCommandedVoltage()) > VOLTAGE_TOLERANCE;
		}

		assertTrue(feedObserved, "Autonomous shooting never reached readiness and fed in a calibrated stationary pose; "
				+ "readiness states=" + readinessStates + ", best motion ratio=" + bestMotionRatio
				+ ", best heading error=" + bestHeadingErrorDegrees + " deg, best flywheel error=" + bestFlywheelError
				+ " RPS, best hood error=" + bestHoodError + " rotations, scheduled flywheel error="
				+ bestScheduledFlywheelError + " RPS, max scheduled flywheel velocity=" + maxScheduledFlywheelVelocity
				+ " RPS, scheduled hood error=" + bestScheduledHoodError + " rotations, transitions="
				+ readinessTransitions + ", debounced ready=" + debouncedReadyObserved + ", feeding telemetry="
				+ feedingTelemetryObserved + ", last scheduled cycle=" + lastScheduledCycle);
		assertFalse(auto.isScheduled(), "Autonomous shooting must finish or abort within three seconds");
		assertNeutral(shooter, "Shooter after autonomous shot completes");
		assertNeutral(bottomIndexer, "Bottom indexer after autonomous shot completes");
		assertNeutral(topIndexer, "Top indexer after autonomous shot completes");
		assertNeutral(conveyor, "Conveyor after autonomous shot completes");
	}

	private void runCycles(int cycles) {
		controller.notifyNewData();
		for (int i = 0; i < cycles; i++) {
			long cycleStartNanos = System.nanoTime();
			DriverStationSim.notifyNewData();
			DriverStation.refreshData();
			scheduler.run();
			container.simulationPeriodic();
			double elapsedSeconds = (System.nanoTime() - cycleStartNanos) / 1_000_000_000.0;
			Timer.delay(Math.max(0.0, 0.02 - elapsedSeconds));
		}
	}

	private static void assertNeutral(Flywheel flywheel, String mechanism) {
		assertEquals(0.0, flywheel.getVelocitySetpoint(), VOLTAGE_TOLERANCE,
				mechanism + " retained a nonzero velocity request");
		assertFalse(flywheel.isVelocityControlActive(), mechanism + " remained in closed-loop velocity control");
		assertEquals(0.0, flywheel.getCommandedVoltage(), VOLTAGE_TOLERANCE, mechanism + " was not commanded to 0 V");
	}

	private static void assertOutputActive(Flywheel flywheel, String mechanism) {
		assertFalse(flywheel.isVelocityControlActive(), mechanism + " unexpectedly used velocity control");
		assertNotEquals(0.0, flywheel.getCommandedVoltage(), VOLTAGE_TOLERANCE, mechanism + " did not receive voltage");
	}

	private static double positionGoal(PositionJoint joint) {
		try {
			return getField(joint, "goalPosition", Double.class);
		} catch (ReflectiveOperationException exception) {
			throw new AssertionError("Unable to inspect simulated position goal", exception);
		}
	}

	private static <T> T getField(Object target, String fieldName, Class<T> fieldType)
			throws ReflectiveOperationException {
		Field field = target.getClass().getDeclaredField(fieldName);
		field.setAccessible(true);
		return fieldType.cast(field.get(target));
	}
}
