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
	private frc.robot.simulation.IntakeSim simulatedIntake;

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
		Object simulation = getField(container, "simulation", RobotSimulation.class);
		var launcher = getField(simulation, "launchedFuelSim", frc.robot.simulation.LaunchedFuelSim.class);
		simulatedIntake = getField(launcher, "intakeSim", frc.robot.simulation.IntakeSim.class);
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
		verifyPracticeFuelResets();
		pilotAAppliesLiveHubTablesAndFeedsOnlyWithBumper();
		verifyRackSettlesWithoutJitter();
		verifyDriveBindingMovesRobot();
		verifyAllianceCanChangeWithoutRestartingSimulation();
		verifyShooterPresetLatchesUntilStopped();
		verifyAcquireDeploysAndStopsOnRelease();
		verifyEjectStopsOnRelease();
		verifyDefaultAutonomousRefusesUninitializedPose();
		verifyAutonomousShotTimesOutSafely();
		verifyIndependentAgitateAndAim();
		verifyCalibrationModeUsesLiveSetpointsAndRequiresForceFeed();
		verifySystemCheckInterlocksAndAbort();
		verifyFullSystemCheckCompletesWithoutSimulationOnlyFailures();
	}

	private void verifyPracticeFuelResets() {
		assertNeutralFuelCount();
		for (int repetition = 0; repetition < 2; repetition++) {
			SimulatedArena.getInstance().clearGamePieces();
			Pose2d pose = container.drivetrain.getPhysicsPose();
			container.simulationTeleopInit();
			assertNeutralFuelCount();
			assertEquals(pose, container.drivetrain.getPhysicsPose(), "Fuel reset must not teleport the robot");
			SimulatedArena.getInstance().clearGamePieces();
			container.simulationAutonomousInit(edu.wpi.first.wpilibj2.command.Commands.none());
			assertNeutralFuelCount();
		}
	}

	private void assertNeutralFuelCount() {
		double minX = FieldConstants.Trench.neutralEdgeX(Alliance.Blue);
		double maxX = FieldConstants.Trench.neutralEdgeX(Alliance.Red);
		long count = java.util.Arrays.stream(SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"))
				.filter(pose -> pose.getX() > minX && pose.getX() < maxX).count();
		assertEquals(400, count, "Practice field must contain exactly 400 neutral-zone fuel");
	}

	private void pilotAAppliesLiveHubTablesAndFeedsOnlyWithBumper() {
		Pose2d initialPose = container.drivetrain.getPhysicsPose();
		Translation2d hub = FieldConstants.Hub.hubPosition();
		controller.setAButton(true);
		double previousRps = 0;
		for (double distance : new double[]{2.4, 3.8}) {
			container.drivetrain.resetPose(new Pose2d(hub.plus(new Translation2d(-distance, 0)), Rotation2d.kZero));
			container.drivetrain.stop();
			runCycles(100);
			ShotSolution expected = ShooterCalculator.calculate(container.drivetrain.getState().Pose);
			assertEquals(expected.flywheelRotationsPerSecond(), shooter.getVelocitySetpoint(), .15);
			assertEquals(expected.hoodRotations(), hood.getDesiredPosition(), .001);
			assertEquals(expected.flywheelRotationsPerSecond(), shooter.getVelocity(), 1.5,
					"Simulated flywheel must reach the interpolated speed");
			assertEquals(expected.hoodRotations(), hood.getPosition(), .004,
					"Simulated hood must reach the interpolated angle");
			assertTrue(shooter.getVelocitySetpoint() > previousRps + 1);
			previousRps = shooter.getVelocitySetpoint();
			assertNeutral(conveyor, "A alone must not feed");
			assertNeutral(bottomIndexer, "A alone must not run bottom indexer");
			assertNeutral(topIndexer, "A alone must not run top indexer");
		}
		controller.setRightBumperButton(true);
		runCycles(50);
		assertOutputActive(conveyor, "A plus RB feeds when up to speed");
		assertOutputActive(bottomIndexer, "A plus RB runs bottom indexer");
		assertOutputActive(topIndexer, "A plus RB runs top indexer");
		Command aimingCommand = shooter.getCurrentCommand();
		Command feedingCommand = conveyor.getCurrentCommand();
		for (int cycle = 0; cycle < 250; cycle++) {
			runCycles(1);
			assertEquals(aimingCommand, shooter.getCurrentCommand(), "Held A must not restart the shot command");
			assertEquals(feedingCommand, conveyor.getCurrentCommand(), "Held RB must not restart feeding");
			assertTrue(shooter.isVelocityControlActive(), "Flywheel must stay in velocity control");
			assertTrue(shooter.getVelocitySetpoint() > 0, "Flywheel target must remain positive");
			assertTrue(shooter.getVelocity() > 1, "Simulated flywheel must not stop");
			assertOutputActive(conveyor, "Conveyor must stay on throughout the volley");
			assertOutputActive(bottomIndexer, "Bottom indexer must stay on throughout the volley");
			assertOutputActive(topIndexer, "Top indexer must stay on throughout the volley");
		}
		controller.setRightBumperButton(false);
		runCycles(5);
		assertNeutral(conveyor, "RB release stops feeding while A stays held");
		assertTrue(shooter.isVelocityControlActive());
		controller.setAButton(false);
		runCycles(5);
		assertNeutral(shooter, "A release stops the automatic flywheel command");
		container.drivetrain.resetPose(initialPose);
		container.drivetrain.stop();
		runCycles(SETTLE_CYCLES);
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

	private void verifyShooterPresetLatchesUntilStopped() {
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
		assertTrue(shooter.isVelocityControlActive(), "Releasing X keeps PID active");
		assertEquals(ShooterConstants.SHOOTER_PRESET.HUB.get(), shooter.getVelocitySetpoint(), 1e-6);
		controller.setYButton(true);
		runCycles(2);
		controller.setYButton(false);
		runCycles(SETTLE_CYCLES);
		assertEquals(ShooterConstants.SHOOTER_PRESET.FERRY.get(), shooter.getVelocitySetpoint(), 1e-6);
		controller.setBButton(true);
		runCycles(2);
		controller.setBButton(false);
		runCycles(SETTLE_CYCLES);
		assertNeutral(shooter, "B stops the latched preset");
		assertEquals(ShooterConstants.HOOD_PRESET.STOW.get(), hood.getDesiredPosition(), 1e-6,
				() -> "Hood should return to its default safe position after X is released; current command="
						+ (hood.getCurrentCommand() == null ? "none" : hood.getCurrentCommand().getName())
						+ ", default active=" + (hood.getCurrentCommand() == hood.getDefaultCommand()) + ", goal="
						+ positionGoal(hood));
	}

	private void verifyAcquireDeploysAndStopsOnRelease() {
		controller.setRightTriggerAxis(1.0);
		runCycles(SETTLE_CYCLES);

		assertEquals(IntakeConstants.RACK_PRESETS.DEPLOY.get(), intakeRack.getDesiredPosition(), 1e-6);
		assertEquals(-ShooterConstants.INDEXER_PRESET.FEED_BOTTOM.get() / 2.0, bottomIndexer.getCommandedVoltage(),
				1e-6);
		assertEquals(-ShooterConstants.INDEXER_PRESET.FEED_TOP.get() / 2.0, topIndexer.getCommandedVoltage(), 1e-6);
		assertOutputActive(intake, "Intake roller while right trigger is held");
		assertOutputActive(conveyor, "Conveyor while right trigger is held");
		assertOutputActive(bottomIndexer, "Bottom indexer while right trigger is held");
		assertOutputActive(topIndexer, "Top indexer while right trigger is held");

		controller.setRightTriggerAxis(0.0);
		runCycles(SETTLE_CYCLES);
		assertNeutral(intake, "Intake roller after right trigger release");
		assertNeutral(conveyor, "Conveyor after right trigger release");
		assertNeutral(bottomIndexer, "Bottom indexer after right trigger release");
		assertNeutral(topIndexer, "Top indexer after right trigger release");
		assertEquals(IntakeConstants.RACK_PRESETS.DEPLOY.get(), intakeRack.getDesiredPosition(), 1e-6,
				"The intake rack should remain deployed after acquisition is released");
	}

	private void verifyIndependentAgitateAndAim() {
		simulatedIntake.intakeSimulation.setGamePiecesCount(6);
		controller.setAButton(true);
		runCycles(SETTLE_CYCLES);
		assertTrue(shooter.isVelocityControlActive(), "A aims and spins up using the shot table");
		assertNeutral(conveyor, "A does not feed");
		controller.setAButton(false);
		runCycles(2);

		controller.setRightBumperButton(true);
		runCycles(2);
		assertOutputActive(intake, "Agitation runs the intake immediately without a preset");
		assertEquals(IntakeConstants.RACK_PRESETS.DEPLOY.get(),
				intakeRack.diagnosticMeasurements().get("ClampedTarget"), 1e-6,
				"Rack waits for feed rollers before retracting");
		assertNeutral(conveyor, "Feed waits for a positive active RPM target");
		boolean sawBlockedPulse = false;
		boolean sawBlockedGap = false;
		for (int cycle = 0; cycle < 30; cycle++) {
			runCycles(1);
			double rumble = controller.getRumble(GenericHID.RumbleType.kBothRumble);
			sawBlockedPulse |= rumble > 0.70;
			sawBlockedGap |= rumble < 0.01;
			assertNeutral(conveyor, "Blocked rumble must not start feeding");
		}
		assertTrue(sawBlockedPulse && sawBlockedGap, "Blocked feedback pulses instead of staying silent or steady");
		assertEquals("SELECT_SHOOTER_PRESET", SmartDashboard.getString("Shooter/FeedBlockedReason", "missing"));
		assertEquals(6, simulatedIntake.getStoredFuelCount(), "Blocked RB must not launch stored fuel");
		controller.setYButton(true);
		runCycles(2);
		controller.setYButton(false);
		for (int cycle = 0; cycle < 150 && Math.abs(bottomIndexer.getCommandedVoltage()) < .01; cycle++) {
			runCycles(1);
		}
		assertOutputActive(bottomIndexer, "Indexer feeds with Y preset and RB alone");

		assertOutputActive(topIndexer, "Indexer feeds with preset and RB alone");
		assertOutputActive(conveyor, "Conveyor feeds with preset and RB alone");
		assertEquals(0.35, controller.getRumble(GenericHID.RumbleType.kBothRumble), 1e-4,
				"Active feeding has steady weaker rumble");
		assertEquals("", SmartDashboard.getString("Shooter/FeedBlockedReason", "missing"));
		boolean observedRetraction = false;
		double previousPosition = intakeRack.getPosition();
		for (int cycle = 0; cycle < 200; cycle++) {
			runCycles(1);
			observedRetraction |= intakeRack.getPosition() < previousPosition - 1e-5;
			previousPosition = intakeRack.getPosition();
			assertOutputActive(intake, "Intake keeps spinning throughout retraction");
			assertOutputActive(conveyor, "Conveyor keeps spinning throughout retraction");
			assertOutputActive(bottomIndexer, "Bottom indexer keeps spinning throughout retraction");
			assertOutputActive(topIndexer, "Top indexer keeps spinning throughout retraction");
		}
		assertTrue(observedRetraction, "Rack must actually move inward while rollers run");
		assertTrue(simulatedIntake.getStoredFuelCount() < 6, "Y plus RB must launch fuel from the simulated hopper");
		assertEquals(IntakeConstants.RACK_PRESETS.SAFE.get(), intakeRack.getPosition(), .02);
		controller.setRightBumperButton(false);
		runCycles(SETTLE_CYCLES);
		assertNeutral(intake, "Intake stops on agitation release");
		assertEquals(0.0, controller.getRumble(GenericHID.RumbleType.kBothRumble), 1e-6,
				"Releasing RB clears haptic feedback");
		assertNeutral(bottomIndexer, "Indexer stops on agitation release");
		assertNeutral(topIndexer, "Indexer stops on agitation release");
		assertNeutral(conveyor, "Conveyor stops on agitation release");
		simulatedIntake.intakeSimulation.setGamePiecesCount(6);
		runCycles(25);
		assertEquals(6, simulatedIntake.getStoredFuelCount(), "RB release must stop simulated launches");
		assertTrue(shooter.isVelocityControlActive(), "Agitation release keeps preset RPM");
		assertEquals(IntakeConstants.RACK_PRESETS.DEPLOY.get(), intakeRack.getDesiredPosition(), 1e-6);
		runCycles(100);
		assertEquals(IntakeConstants.RACK_PRESETS.DEPLOY.get(), intakeRack.getPosition(), .02);

		controller.setRightBumperButton(true);
		runCycles(SETTLE_CYCLES);
		controller.setBButton(true);
		runCycles(3);
		assertNeutral(shooter, "B leaves shooter in zero-voltage mode even while RB is held");
		assertNeutral(conveyor, "B stops feed even while RB is held");
		assertNeutral(bottomIndexer, "B stops indexers even while RB is held");
		controller.setBButton(false);
		controller.setRightBumperButton(false);
		runCycles(SETTLE_CYCLES);
	}

	private void verifyEjectStopsOnRelease() {
		controller.setLeftTriggerAxis(1.0);
		runCycles(SETTLE_CYCLES);
		assertOutputActive(intake, "Intake roller while eject is held");
		assertOutputActive(conveyor, "Conveyor while eject is held");
		assertOutputActive(bottomIndexer, "Bottom indexer while eject is held");
		assertOutputActive(topIndexer, "Top indexer while eject is held");

		assertEquals(IntakeConstants.ROLLER_PRESETS.EJECT.get(), intake.getCommandedVoltage(), 1e-6);
		assertEquals(IntakeConstants.ROLLER_PRESETS.INTAKE.get() / 2.0, conveyor.getCommandedVoltage(), 1e-6);
		assertEquals(-ShooterConstants.INDEXER_PRESET.FEED_BOTTOM.get(), bottomIndexer.getCommandedVoltage(), 1e-6);
		assertEquals(-ShooterConstants.INDEXER_PRESET.FEED_TOP.get(), topIndexer.getCommandedVoltage(), 1e-6);

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

		controller.setXButton(true);
		runCycles(2);
		controller.setXButton(false);
		runCycles(SETTLE_CYCLES);
		assertEquals(calibrationHoodRotations, hood.getDesiredPosition(), 1e-6);
		assertEquals(calibrationFlywheelRps, shooter.getVelocitySetpoint(), 1e-6);
		assertNeutral(bottomIndexer, "Bottom indexer during calibration without force feed");
		assertNeutral(topIndexer, "Top indexer during calibration without force feed");
		assertNeutral(conveyor, "Conveyor during calibration without force feed");

		controller.setRightBumperButton(true);
		runCycles(SETTLE_CYCLES);
		assertOutputActive(bottomIndexer, "Bottom indexer during calibration force feed");
		assertOutputActive(topIndexer, "Top indexer during calibration force feed");
		assertOutputActive(conveyor, "Conveyor during calibration force feed");
		// A new speed target creates a speed deficit, but must not interrupt the
		// volley.
		setCalibrationInputs(true, calibrationHoodRotations, 50.0);
		runCycles(1);
		assertOutputActive(conveyor, "Conveyor stays feeding through speed changes with RB held");
		assertOutputActive(bottomIndexer, "Bottom indexer stays feeding with RB held");
		assertOutputActive(topIndexer, "Top indexer stays feeding with RB held");

		controller.setRightBumperButton(false);
		controller.setBButton(true);
		runCycles(2);
		controller.setBButton(false);
		runCycles(SETTLE_CYCLES);
		assertNeutral(shooter, "Shooter after calibration release");
		assertNeutral(bottomIndexer, "Bottom indexer after calibration release");
		assertNeutral(topIndexer, "Top indexer after calibration release");
		assertNeutral(conveyor, "Conveyor after calibration release");
		assertFalse(SmartDashboard.getBoolean("Shooter/CalibrationMode", true),
				"Calibration-active telemetry must clear when B stops the preset");
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
		// PathPlanner shooting stages must not inherit the default auto's one-second
		// cutoff.
		for (String stage : new String[]{"FeedRollers", "Agitate"}) {
			Command timedStage = com.pathplanner.lib.auto.NamedCommands.getCommand(stage);
			scheduler.schedule(timedStage);
			boolean started = false;
			int feedingCycles = 0;
			Set<String> stageStates = new LinkedHashSet<>();
			for (int cycle = 0; cycle < 600 && feedingCycles < 150; cycle++) {
				runCycles(1);
				assertTrue(timedStage.isScheduled(), stage + " must wait for its PathPlanner deadline");
				stageStates.add(SmartDashboard.getString("Shooter/Readiness", "missing"));
				boolean feeding = Math.abs(conveyor.getCommandedVoltage()) > VOLTAGE_TOLERANCE;
				if (feeding)
					started = true;
				if (started) {
					assertTrue(feeding, stage + " must not pause an established volley");
					assertEquals(ShooterConstants.INDEXER_PRESET.FEED_BOTTOM.get(), bottomIndexer.getCommandedVoltage(),
							VOLTAGE_TOLERANCE);
					assertEquals(ShooterConstants.INDEXER_PRESET.FEED_TOP.get(), topIndexer.getCommandedVoltage(),
							VOLTAGE_TOLERANCE);
					assertEquals(ShooterConstants.CONVEYOR_PRESET.FEED.get(), conveyor.getCommandedVoltage(),
							VOLTAGE_TOLERANCE);
					assertEquals(IntakeConstants.ROLLER_PRESETS.INTAKE.get(), intake.getCommandedVoltage(),
							VOLTAGE_TOLERANCE);
					feedingCycles++;
				}
			}
			assertTrue(feedingCycles > 100, stage + " should feed continuously for more than two seconds: "
					+ feedingCycles + " cycles; " + stageStates);
			// Inject a measured speed sag after initial readiness without changing the
			// request.
			try {
				var inputs = getField(shooter, "inputs",
						frc.robot.subsystems.flywheel.FlywheelIOInputsAutoLogged.class);
				inputs.velocity = 0.0;
				timedStage.execute();
				assertEquals(ShooterConstants.CONVEYOR_PRESET.FEED.get(), conveyor.getCommandedVoltage(),
						VOLTAGE_TOLERANCE);
				assertEquals(ShooterConstants.INDEXER_PRESET.FEED_BOTTOM.get(), bottomIndexer.getCommandedVoltage(),
						VOLTAGE_TOLERANCE);
				assertEquals(ShooterConstants.INDEXER_PRESET.FEED_TOP.get(), topIndexer.getCommandedVoltage(),
						VOLTAGE_TOLERANCE);
			} catch (ReflectiveOperationException exception) {
				throw new AssertionError(exception);
			}
			scheduler.cancel(timedStage);
			runCycles(1);
			assertNeutral(intake, stage + " intake cancellation");
			assertNeutral(conveyor, stage + " conveyor cancellation");
			assertNeutral(bottomIndexer, stage + " bottom indexer cancellation");
			assertNeutral(topIndexer, stage + " top indexer cancellation");
			assertNeutral(shooter, stage + " shooter cancellation");
		}

		// Load the actual Moses stop, including concurrent rack retraction and its
		// feeding-based timer.
		container.drivetrain.resetPose(new Pose2d(
				hub.plus(new Translation2d(fieldDirection * robotCenterDistanceMeters, 0.0)), startingHeading));
		container.drivetrain.stop();
		Command aimPreparation = com.pathplanner.lib.auto.NamedCommands.getCommand("AutoAim");
		scheduler.schedule(aimPreparation);
		int stablePreparationCycles = 0;
		for (int cycle = 0; cycle < 500 && stablePreparationCycles < 10; cycle++) {
			runCycles(1);
			var prepared = ShooterCalculator.calculate(container.drivetrain.getState().Pose);
			var speeds = container.drivetrain.getPhysicsSpeeds();
			boolean ready = Math
					.abs(shooter.getVelocity()
							- prepared.flywheelRotationsPerSecond()) < ShooterConstants.FLYWHEEL_READY_TOLERANCE_RPS
					&& Math.abs(hood.getPosition()
							- prepared.hoodRotations()) < ShooterConstants.HOOD_READY_TOLERANCE_ROTATIONS
					&& Math.abs(container.drivetrain.getState().Pose.getRotation().minus(prepared.targetHeading())
							.getDegrees()) < ShooterConstants.HEADING_READY_TOLERANCE_DEGREES
					&& Math.hypot(speeds.vxMetersPerSecond,
							speeds.vyMetersPerSecond) < ShooterConstants.MAX_SHOOTING_TRANSLATION_METERS_PER_SECOND
					&& Math.abs(
							speeds.omegaRadiansPerSecond) < ShooterConstants.MAX_SHOOTING_ROTATION_RADIANS_PER_SECOND;
			stablePreparationCycles = ready ? stablePreparationCycles + 1 : 0;
		}
		assertEquals(10, stablePreparationCycles, "Timing regression requires an actually settled initial shot");
		scheduler.cancel(aimPreparation);
		try {
			var document = (org.json.simple.JSONObject) new org.json.simple.parser.JSONParser()
					.parse(java.nio.file.Files
							.readString(java.nio.file.Path.of("src/main/deploy/pathplanner/autos/Moses 3 Cycle.auto")));
			var root = (org.json.simple.JSONObject) document.get("command");
			var stages = (org.json.simple.JSONArray) ((org.json.simple.JSONObject) root.get("data")).get("commands");
			for (int pathStage : new int[]{4}) {
				var approach = (org.json.simple.JSONObject) stages.get(pathStage);
				var approachCommands = (org.json.simple.JSONArray) ((org.json.simple.JSONObject) approach.get("data"))
						.get("commands");
				var preparation = (org.json.simple.JSONObject) approachCommands.get(2);
				assertEquals("PrepareHubShot", ((org.json.simple.JSONObject) preparation.get("data")).get("name"));
				var next = (org.json.simple.JSONObject) stages.get(pathStage + 1);
				assertEquals("named", next.get("type"), "Approach must transition directly to shooting without a wait");
				assertEquals("HubVolley", ((org.json.simple.JSONObject) next.get("data")).get("name"));
			}
			assertEquals(7, stages.size(), "Moses must end after its final path without another aim/shot stage");
			var finalPath = (org.json.simple.JSONObject) stages.get(6);
			var finalCommands = (org.json.simple.JSONArray) ((org.json.simple.JSONObject) finalPath.get("data"))
					.get("commands");
			assertEquals(2, finalCommands.size(), "Final path must not prepare another shot");
			assertEquals("Moses 2",
					((org.json.simple.JSONObject) ((org.json.simple.JSONObject) finalCommands.get(0)).get("data"))
							.get("pathName"));
			Command stop = com.pathplanner.lib.auto.CommandUtil
					.commandFromJson((org.json.simple.JSONObject) stages.get(3), false, false);
			scheduler.schedule(stop);
			boolean feeding = false;
			int firstFeedCycle = -1;
			int endCycle = -1;
			boolean rackRetracting = false;
			for (int cycle = 0; cycle < 310; cycle++) {
				runCycles(1);
				if (!stop.isScheduled()) {
					endCycle = cycle;
					break;
				}
				if (cycle > 1) {
					assertTrue(shooter.isVelocityControlActive(), "Shooter must stay commanded throughout the volley");
				}
				if (Math.abs(conveyor.getCommandedVoltage()) > VOLTAGE_TOLERANCE && !feeding) {
					feeding = true;
					firstFeedCycle = cycle;
				}
				if (feeding) {
					assertOutputActive(conveyor, "Continuous conveyor");
					assertOutputActive(bottomIndexer, "Continuous bottom indexer");
					assertOutputActive(topIndexer, "Continuous top indexer");
					assertOutputActive(intake, "Continuous intake");
					rackRetracting |= Math.abs(positionGoal(intakeRack)
							- frc.robot.commands.ShotIntakeSettings.load().holdPositionMeters()) < 0.001;
				}
			}
			assertTrue(firstFeedCycle >= 0, "Volley must actually feed before completing");
			assertTrue(rackRetracting, "Rack must retract during feeding without an extra timed stage");
			assertTrue(endCycle >= 0, "Volley must finish within its bounded timeout");
			assertEquals(3.0, (endCycle - firstFeedCycle) * 0.02, 0.06,
					"Move on after three seconds of feeding, not six seconds at the shooting stop");
			runCycles(1);
			assertNeutral(shooter, "Merged stop shooter");
			assertNeutral(conveyor, "Merged stop conveyor");
			assertNeutral(bottomIndexer, "Merged stop bottom indexer");
			assertNeutral(topIndexer, "Merged stop top indexer");
			assertNeutral(intake, "Merged stop intake");
		} catch (java.io.IOException | org.json.simple.parser.ParseException exception) {
			throw new AssertionError(exception);
		}

		Command blockedVolley = frc.robot.commands.shooter.ShooterCommands.autonomousHubVolley(container.drivetrain,
				hood, shooter, bottomIndexer, topIndexer, conveyor, intake, intakeRack, () -> false);
		scheduler.schedule(blockedVolley);
		for (int cycle = 0; cycle < 160; cycle++) {
			runCycles(1);
			assertEquals(0.0, conveyor.getCommandedVoltage(), VOLTAGE_TOLERANCE, "Unready volley must not feed");
		}
		assertFalse(blockedVolley.isScheduled(), "Missing readiness must abort after three seconds");
		assertNeutral(intake, "Blocked volley intake");
		assertNeutral(shooter, "Blocked volley shooter");

		Command preparation = com.pathplanner.lib.auto.NamedCommands.getCommand("PrepareHubShot");
		assertFalse(preparation.getRequirements().contains(container.drivetrain),
				"Preparation must not interrupt the path");
		scheduler.schedule(preparation);
		for (int cycle = 0; cycle < 130; cycle++) {
			runCycles(1);
			assertTrue(preparation.isScheduled(), "Approach preparation must not expire after two seconds");
			assertTrue(shooter.isVelocityControlActive(), "Prepare the shooter throughout the approach");
			assertEquals(0.0, conveyor.getCommandedVoltage(), VOLTAGE_TOLERANCE);
			assertEquals(0.0, bottomIndexer.getCommandedVoltage(), VOLTAGE_TOLERANCE);
			assertEquals(0.0, topIndexer.getCommandedVoltage(), VOLTAGE_TOLERANCE);
		}
		scheduler.cancel(preparation);
		runCycles(1);
		assertNeutral(shooter, "Approach preparation cancellation");

		// The Robot calls this after the scheduler when the selected auto is done.
		scheduler.schedule(com.pathplanner.lib.auto.NamedCommands.getCommand("AutoAim"));
		runCycles(10);
		for (int cycle = 0; cycle < 10; cycle++) {
			container.stopAutonomousOutputs();
			runCycles(1);
			container.stopAutonomousOutputs();
			assertNeutral(shooter, "End-of-auto shooter");
			assertNeutral(intake, "End-of-auto intake");
			assertNeutral(conveyor, "End-of-auto conveyor");
			assertNeutral(bottomIndexer, "End-of-auto bottom indexer");
			assertNeutral(topIndexer, "End-of-auto top indexer");
			assertTrue(hood.isOpenLoopMode(), "Hood must not return to a stow command after auto");
			assertEquals(0.0, hood.getCommandedVoltage(), VOLTAGE_TOLERANCE);
			assertTrue(intakeRack.isOpenLoopMode());
			assertEquals(0.0, intakeRack.getCommandedVoltage(), VOLTAGE_TOLERANCE);
			assertEquals(null, container.drivetrain.getCurrentCommand(), "Auto aim must be canceled");
		}

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
