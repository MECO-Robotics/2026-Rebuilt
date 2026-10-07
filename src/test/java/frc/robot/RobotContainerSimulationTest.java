package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.subsystems.IntakeConstants;
import frc.robot.constants.subsystems.ShooterConstants;
import frc.robot.commands.shooter.ShooterCalculator;
import frc.robot.commands.shooter.ShotSolution;
import frc.robot.simulation.RobotSimulation;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.position_joint.PositionJoint;
import frc.robot.subsystems.vision.Vision;
import frc.robot.systemcheck.SystemCheckManager;
import frc.robot.systemcheck.SystemCheckRunState;
import frc.robot.systemcheck.CheckStatus;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Set;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.ironmaple.simulation.SimulatedArena;

/**
 * End-to-end desktop simulation checks for the real driver controller bindings.
 */
class RobotContainerSimulationTest {
	private static final double VOLTAGE_TOLERANCE = 1e-6;
	private static final int SETTLE_CYCLES = 25;

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
		SimHooks.pauseTiming();
		RobotSimulation.configureArenaOverride(frc.robot.constants.Constants.Mode.SIM);

		DriverStationSim.resetData();
		Alliance configuredAlliance = configuredTestAlliance();
		DriverStationSim.setAllianceStationId(
				configuredAlliance == Alliance.Red ? AllianceStationID.Red1 : AllianceStationID.Blue1);
		DriverStationSim.setDsAttached(true);
		DriverStationSim.setAutonomous(false);
		DriverStationSim.setEnabled(true);

		controller = new XboxControllerSim(0);
		controller.setAxisCount(6);
		controller.setButtonCount(10);
		controller.setPOVCount(1);
		controller.notifyNewData();
		var copilot = new edu.wpi.first.wpilibj.simulation.GenericHIDSim(1);
		copilot.setAxisCount(6);
		copilot.setButtonCount(10);
		copilot.setPOVCount(1);
		copilot.notifyNewData();
		DriverStation.refreshData();
		assertEquals(configuredAlliance, DriverStation.getAlliance().orElseThrow(),
				"The simulation must use the requested FRC_TEST_ALLIANCE");
		setCalibrationInputs(false, 0.000, 30.0);

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

	private static Alliance configuredTestAlliance() {
		String requestedAlliance = System.getenv().getOrDefault("FRC_TEST_ALLIANCE", "blue").trim()
				.toLowerCase(Locale.ROOT);
		return switch (requestedAlliance) {
			case "blue" -> Alliance.Blue;
			case "red" -> Alliance.Red;
			default -> throw new IllegalArgumentException(
					"FRC_TEST_ALLIANCE must be either 'blue' or 'red', but was '" + requestedAlliance + "'");
		};
	}

	@AfterEach
	void cleanUpSimulation() {
		scheduler.cancelAll();
		scheduler.unregisterAllSubsystems();
		DriverStationSim.setEnabled(false);
		DriverStationSim.notifyNewData();
		DriverStation.refreshData();
		SimHooks.resumeTiming();
	}

	@Test
	void driverBindingsOperateAndReturnEveryRollerToNeutral() {
		assertTrue(SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel").length > 0,
				"Simulation should populate field fuel before autonomous is started");
		verifyRackSettlesWithoutJitter();
		verifyDriveBindingMovesRobot();
		verifyAllianceCanChangeWithoutRestartingSimulation();
		verifyShooterPresetStopsOnRelease();
		verifyAcquireDeploysAndStopsOnRelease();
		verifyEjectStopsOnRelease();
		verifyDefaultAutonomousRefusesUninitializedPose();
		verifyAutonomousShotTimesOutSafely();
		verifyForceFeedRequiresCoordinatedShot();
		verifyCalibrationModeUsesLiveSetpointsAndRequiresForceFeed();
		verifySystemCheckInterlocksAndAbort();
		verifyFullSystemCheckCompletesWithoutSimulationOnlyFailures();
	}

	private void verifyRackSettlesWithoutJitter() {
		for (double target : new double[]{0.13, 0.30, 0.13, 0.0}) {
			Command hold = PositionJoint.holdPosition(intakeRack, () -> target);
			scheduler.schedule(hold);
			runCycles(150);
			double minimum = Double.POSITIVE_INFINITY;
			double maximum = Double.NEGATIVE_INFINITY;
			for (int cycle = 0; cycle < 100; cycle++) {
				runCycles(1);
				minimum = Math.min(minimum, intakeRack.getPosition());
				maximum = Math.max(maximum, intakeRack.getPosition());
				assertEquals(target, intakeRack.getPosition(), .005, "Rack must settle within 5 mm at " + target);
			}
			assertTrue(maximum - minimum < .002,
					"Rack holding oscillation exceeds 2 mm at " + target + ": " + (maximum - minimum));
			scheduler.cancel(hold);
		}
	}

	private void verifyFullSystemCheckCompletesWithoutSimulationOnlyFailures() {
		// Run motor checks in clear neutral-zone space, independent of the prior shot
		// pose.
		container.drivetrain.resetPose(new Pose2d(7.0, 1.2, Rotation2d.kZero));
		container.drivetrain.stop();
		SystemCheckManager manager;
		try {
			manager = getField(container, "systemCheckManager", SystemCheckManager.class);
		} catch (ReflectiveOperationException exception) {
			throw new AssertionError("Unable to inspect full simulated system check", exception);
		}
		DriverStationSim.setAutonomous(false);
		DriverStationSim.setTest(true);
		DriverStationSim.setEnabled(true);
		RoboRioSim.setVInVoltage(12.5);
		DriverStationSim.notifyNewData();
		DriverStation.refreshData();
		clearSystemCheckControls();
		container.systemCheckTestInit();
		setSystemCheckBoolean("Controls/SetupConfirmed", true);
		setSystemCheckBoolean("Controls/Armed", true);
		container.updateDashboardOutputs();
		setSystemCheckBoolean("Controls/Start", true);
		container.updateDashboardOutputs();

		for (int cycle = 0; cycle < 6_000 && manager.getRunState() != SystemCheckRunState.COMPLETE; cycle++) {
			runSystemCheckCycle();
		}

		assertEquals(SystemCheckRunState.COMPLETE, manager.getRunState(),
				() -> "Full simulated system check did not complete: " + manager.getResults());
		var unacceptableResults = manager.getResults().values().stream()
				.filter(result -> result.status() == CheckStatus.FAIL || result.status() == CheckStatus.SKIPPED
						|| result.status() == CheckStatus.ABORTED)
				.toList();
		assertTrue(unacceptableResults.isEmpty(), () -> "Unexpected simulated system-check results: "
				+ unacceptableResults + "; all results=" + manager.getResults().values());
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

	private void verifyAllianceCanChangeWithoutRestartingSimulation() {
		Alliance startingAlliance = DriverStation.getAlliance().orElseThrow();
		Alliance oppositeAlliance = startingAlliance == Alliance.Blue ? Alliance.Red : Alliance.Blue;

		verifyAllianceDriveAndAim(startingAlliance);
		verifyAllianceDriveAndAim(oppositeAlliance);
		verifyAllianceDriveAndAim(startingAlliance);
	}

	private void verifyAllianceDriveAndAim(Alliance alliance) {
		DriverStationSim.setEnabled(false);
		DriverStationSim
				.setAllianceStationId(alliance == Alliance.Red ? AllianceStationID.Red1 : AllianceStationID.Blue1);
		DriverStationSim.notifyNewData();
		DriverStation.refreshData();
		runCycles(5);

		assertEquals(alliance, DriverStation.getAlliance().orElseThrow(),
				"Driver Station alliance must update while disabled without restarting robot code");
		assertEquals(FieldConstants.Hub.hubPosition(alliance), FieldConstants.Hub.hubPosition(),
				"Pose-based shooting must select the newly selected alliance hub");

		DriverStationSim.setAutonomous(false);
		DriverStationSim.setTest(false);
		DriverStationSim.setEnabled(true);
		DriverStationSim.notifyNewData();
		DriverStation.refreshData();

		Pose2d driveStart = new Pose2d(Units.inchesToMeters(325.0), Units.inchesToMeters(80.0), Rotation2d.kZero);
		container.drivetrain.resetPose(driveStart);
		container.drivetrain.stop();
		runCycles(SETTLE_CYCLES);
		controller.setLeftY(-0.65);
		runCycles(60);
		controller.setLeftY(0.0);
		runCycles(SETTLE_CYCLES);

		double driveDeltaX = container.drivetrain.getPhysicsPose().getX() - driveStart.getX();
		double expectedDirection = alliance == Alliance.Blue ? 1.0 : -1.0;
		assertTrue(driveDeltaX * expectedDirection > 0.05,
				() -> alliance + " driver-forward movement used the wrong field direction: deltaX=" + driveDeltaX);
		assertEquals(alliance == Alliance.Red, container.drivetrain.shouldFlipAutoPath(),
				"Path mirroring must follow the newly selected alliance");

		Translation2d hub = FieldConstants.Hub.hubPosition(alliance);
		double hubToRobotDirection = alliance == Alliance.Blue ? 1.0 : -1.0;
		Pose2d aimStart = new Pose2d(hub.plus(new Translation2d(hubToRobotDirection * 2.0, 0.8)),
				Rotation2d.fromDegrees(90.0));
		container.drivetrain.resetPose(aimStart);
		container.drivetrain.stop();
		runCycles(SETTLE_CYCLES);

		Command aim = DriveCommands.autoAimToHub(container.drivetrain,
				frc.robot.constants.drive.DrivetrainConstants.MAX_SPEED);
		scheduler.schedule(aim);
		// CTRE status arrives on native worker threads. Wait for stable convergence
		// within six simulated seconds, yielding wall time for those workers.
		int stableCycles = 0;
		for (int cycle = 0; cycle < 300 && stableCycles < 10; cycle++) {
			runCycles(1);
			try {
				Thread.sleep(2);
			} catch (InterruptedException e) {
				Thread.currentThread().interrupt();
				throw new AssertionError(e);
			}
			Pose2d pose = container.drivetrain.getState().Pose;
			Rotation2d target = ShooterCalculator.calculate(pose, hub).targetHeading();
			stableCycles = Math.abs(
					pose.getRotation().minus(target).getDegrees()) <= ShooterConstants.HEADING_READY_TOLERANCE_DEGREES
							? stableCycles + 1
							: 0;
		}
		assertEquals(10, stableCycles, "Auto aim must settle at the CAD-aligned heading");
		Pose2d aimedPose = container.drivetrain.getState().Pose;
		Pose2d shooterPose = aimedPose
				.transformBy(new Transform2d(ShooterConstants.SHOOTER_EXIT_TRANSLATION, Rotation2d.kZero));
		Rotation2d targetBearing = hub.minus(shooterPose.getTranslation()).getAngle();
		Rotation2d physicalShooterHeading = aimedPose.getRotation()
				.plus(ShooterConstants.aimingYawForMode(frc.robot.constants.Constants.Mode.SIM));
		double aimErrorDegrees = Math.abs(physicalShooterHeading.minus(targetBearing).getDegrees());
		scheduler.cancel(aim);
		container.drivetrain.stop();

		assertTrue(aimErrorDegrees <= ShooterConstants.HEADING_READY_TOLERANCE_DEGREES,
				() -> alliance + " physical shooter aimed away from its hub after a live alliance change: error="
						+ aimErrorDegrees + " degrees, pose=" + aimedPose + ", hub=" + hub);
	}

	private void verifyShooterPresetStopsOnRelease() {
		controller.setXButton(true);
		runCycles(SETTLE_CYCLES);
		assertEquals(ShooterConstants.SHOOTER_PRESET.HUB.get(), shooter.getVelocitySetpoint(), 1e-6);
		assertTrue(Math.abs(shooter.getVelocity()) > 1.0,
				() -> "The simulated shooter did not physically spin up: velocity=" + shooter.getVelocity()
						+ ", setpoint=" + shooter.getVelocitySetpoint() + ", battery="
						+ edu.wpi.first.wpilibj.RobotController.getBatteryVoltage() + ", command="
						+ (shooter.getCurrentCommand() == null ? "none" : shooter.getCurrentCommand().getName()));

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
		assertTrue(shooter.getVelocitySetpoint() > 0.0, "A should keep the shooter prepared after override release");
		// The path may continue automatically if normal readiness becomes valid after
		// the override is released; that is the intended one-button shooting behavior.

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

	private void verifyCalibrationModeUsesLiveSetpointsAndRequiresForceFeed() {
		DriverStationSim.setAutonomous(false);
		DriverStationSim.notifyNewData();
		DriverStation.refreshData();

		double calibrationHoodRotations = 0.031;
		double calibrationFlywheelRps = 37.0;
		setCalibrationInputs(true, calibrationHoodRotations, calibrationFlywheelRps);

		controller.setAButton(true);
		runCycles(SETTLE_CYCLES);
		assertEquals(calibrationHoodRotations, hood.getDesiredPosition(), 1e-6);
		assertEquals(calibrationFlywheelRps, shooter.getVelocitySetpoint(), 1e-6);
		assertNeutral(bottomIndexer, "Bottom indexer during calibration without force feed");
		assertNeutral(topIndexer, "Top indexer during calibration without force feed");
		assertNeutral(conveyor, "Conveyor during calibration without force feed");
		assertEquals("CALIBRATION_MODE", SmartDashboard.getString("Shooter/Readiness", "missing"));

		controller.setRightBumperButton(true);
		runCycles(SETTLE_CYCLES);
		assertOutputActive(bottomIndexer, "Bottom indexer during calibration force feed");
		assertOutputActive(topIndexer, "Top indexer during calibration force feed");
		assertOutputActive(conveyor, "Conveyor during calibration force feed");

		controller.setRightBumperButton(false);
		controller.setAButton(false);
		runCycles(SETTLE_CYCLES);
		assertNeutral(shooter, "Shooter after calibration release");
		assertNeutral(bottomIndexer, "Bottom indexer after calibration release");
		assertNeutral(topIndexer, "Top indexer after calibration release");
		assertNeutral(conveyor, "Conveyor after calibration release");
		assertFalse(SmartDashboard.getBoolean("Shooter/CalibrationMode", true),
				"Calibration-active telemetry must clear when A is released");
		setCalibrationInputs(false, calibrationHoodRotations, calibrationFlywheelRps);
	}

	private static void setCalibrationInputs(boolean enabled, double hoodRotations, double flywheelRps) {
		ShooterConstants.CALIBRATION.ENABLED.set(enabled);
		ShooterConstants.CALIBRATION.HOOD_ROTATIONS.set(hoodRotations);
		ShooterConstants.CALIBRATION.FLYWHEEL_RPS.set(flywheelRps);
		ShooterConstants.CALIBRATION.ENABLED.periodic();
		ShooterConstants.CALIBRATION.HOOD_ROTATIONS.periodic();
		ShooterConstants.CALIBRATION.FLYWHEEL_RPS.periodic();
	}

	private void verifyAutonomousShotTimesOutSafely() {
		Translation2d hub = FieldConstants.Hub.hubPosition();
		Alliance alliance = DriverStation.getAlliance().orElseThrow();
		// Use a calibrated mid-range point so static-friction chatter at the hood's
		// zero stop
		// cannot make this whole-robot timing test nondeterministic.
		double shooterDistanceMeters = Units.inchesToMeters(123.24);
		double robotCenterDistanceMeters = shooterDistanceMeters + ShooterConstants.SHOOTER_EXIT_TRANSLATION.getX();
		double fieldDirection = alliance == Alliance.Blue ? 1.0 : -1.0;
		Rotation2d startingHeading = alliance == Alliance.Blue ? Rotation2d.kPi : Rotation2d.kZero;
		container.drivetrain.resetPose(new Pose2d(
				hub.plus(new Translation2d(fieldDirection * robotCenterDistanceMeters, 0.0)), startingHeading));
		container.drivetrain.stop();
		runCycles(50);

		Command prepareShot = PositionJoint
				.holdPosition(hood,
						() -> ShooterCalculator.calculate(container.drivetrain.getState().Pose).hoodRotations())
				.alongWith(Flywheel.holdVelocity(shooter, () -> ShooterCalculator
						.calculate(container.drivetrain.getState().Pose).flywheelRotationsPerSecond()));
		scheduler.schedule(prepareShot);
		runCycles(150);
		scheduler.cancel(prepareShot);

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
		DriverStationSim.setAutonomous(false);
		DriverStationSim.notifyNewData();
		DriverStation.refreshData();
	}

	private void verifyDefaultAutonomousRefusesUninitializedPose() {
		try {
			Vision vision = getField(container, "vision", Vision.class);
			Field poseReady = Vision.class.getDeclaredField("poseReady");
			poseReady.setAccessible(true);
			poseReady.setBoolean(vision, false);

			DriverStationSim.setAutonomous(true);
			DriverStationSim.notifyNewData();
			DriverStation.refreshData();
			Command auto = container.getAutonomousCommand();
			assertEquals("AutonomousHubShot", auto.getName(),
					"The chooser default must be the coordinated standalone hub-shot command");
			auto.initialize();
			auto.execute();

			assertNeutral(shooter, "Shooter during default auto without a field-aligned pose");
			assertEquals(ShooterConstants.HOOD_PRESET.STOW.get(), hood.getDesiredPosition(), 1e-6,
					"The default autonomous must keep the hood stowed until vision initializes the field pose");
			auto.end(true);
			DriverStationSim.setAutonomous(false);
			DriverStationSim.notifyNewData();
			DriverStation.refreshData();
		} catch (ReflectiveOperationException exception) {
			throw new AssertionError("Unable to set the simulated vision initialization state", exception);
		}
	}

	private void verifySystemCheckInterlocksAndAbort() {
		try {
			SystemCheckManager manager = getField(container, "systemCheckManager", SystemCheckManager.class);
			DriverStationSim.setAutonomous(false);
			DriverStationSim.setTest(true);
			DriverStationSim.setEnabled(true);
			RoboRioSim.setVInVoltage(12.5);
			DriverStationSim.notifyNewData();
			DriverStation.refreshData();
			clearSystemCheckControls();
			setSystemCheckBoolean("Controls/SetupConfirmed", true);
			setSystemCheckBoolean("Controls/Armed", true);
			setSystemCheckBoolean("PracticeShot/Armed", true);
			container.systemCheckTestInit();
			assertFalse(getSystemCheckBoolean("Controls/SetupConfirmed"),
					"Entering Test mode must clear a stale setup confirmation");
			assertFalse(getSystemCheckBoolean("Controls/Armed"), "Entering Test mode must clear a stale pit arm");
			assertFalse(getSystemCheckBoolean("PracticeShot/Armed"),
					"Entering Test mode must clear a stale live-shot arm");
			setSystemCheckBoolean("Controls/SetupConfirmed", true);
			setSystemCheckBoolean("Controls/Armed", true);
			container.updateDashboardOutputs();
			assertEquals(SystemCheckRunState.READY, manager.getRunState());

			controller.setXButton(true);
			runCycles(10);
			assertFalse(shooter.isVelocityControlActive(), "Normal shooter controls must be unavailable in Test mode");
			assertEquals(0.0, shooter.getCommandedVoltage(), VOLTAGE_TOLERANCE);
			controller.setXButton(false);

			setSystemCheckBoolean("SimulationFaults/LowBattery", true);
			setSystemCheckBoolean("Controls/Start", true);
			container.updateDashboardOutputs();
			assertEquals(SystemCheckRunState.BLOCKED, manager.getRunState());

			setSystemCheckBoolean("SimulationFaults/LowBattery", false);
			container.updateDashboardOutputs();
			setSystemCheckBoolean("Controls/Start", true);
			container.updateDashboardOutputs();
			assertEquals(SystemCheckRunState.COUNTDOWN, manager.getRunState());
			assertFalse(getSystemCheckBoolean("Controls/Armed"), "An accepted start must consume the pit arm");
			runCycles(1);

			setSystemCheckBoolean("Controls/Abort", true);
			container.updateDashboardOutputs();
			assertEquals(SystemCheckRunState.ABORTED, manager.getRunState());
			assertNeutral(shooter, "Shooter after system-check abort");
			assertTrue(hood.isOpenLoopMode());
			assertEquals(0.0, hood.getCommandedVoltage(), VOLTAGE_TOLERANCE);
			clearSystemCheckControls();
			DriverStationSim.setTest(false);
		} catch (ReflectiveOperationException exception) {
			throw new AssertionError("Unable to inspect system-check integration", exception);
		}
	}

	private static void clearSystemCheckControls() {
		String[] keys = {"Controls/SetupConfirmed", "Controls/Armed", "Controls/Start", "Controls/Abort",
				"PracticeShot/Armed", "PracticeShot/AreaClearConfirmed", "PracticeShot/ExactlyOneFuelConfirmed",
				"PracticeShot/Start", "PracticeShot/Made", "PracticeShot/Missed", "SimulationFaults/DisconnectedMotor",
				"SimulationFaults/FrozenEncoder", "SimulationFaults/ExcessiveCurrent",
				"SimulationFaults/HighTemperature", "SimulationFaults/MissingVision", "SimulationFaults/LowBattery"};
		for (String key : keys) {
			setSystemCheckBoolean(key, false);
		}
	}

	private static void setSystemCheckBoolean(String key, boolean value) {
		NetworkTableInstance.getDefault().getTable("SystemCheck").getEntry(key).setBoolean(value);
	}

	private static boolean getSystemCheckBoolean(String key) {
		return NetworkTableInstance.getDefault().getTable("SystemCheck").getEntry(key).getBoolean(false);
	}

	private void runCycles(int cycles) {
		controller.notifyNewData();
		for (int i = 0; i < cycles; i++) {
			SimHooks.stepTiming(0.02);
			DriverStationSim.notifyNewData();
			DriverStation.refreshData();
			scheduler.run();
			container.simulationPeriodic();
		}
	}

	private void runSystemCheckCycle() {
		controller.notifyNewData();
		SimHooks.stepTiming(0.02);
		DriverStationSim.notifyNewData();
		DriverStation.refreshData();
		scheduler.run();
		container.updateDashboardOutputs();
		container.simulationPeriodic();
		// Allow the native CTRE status workers to publish between simulation steps.
		try {
			Thread.sleep(2);
		} catch (InterruptedException e) {
			Thread.currentThread().interrupt();
			throw new AssertionError(e);
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
