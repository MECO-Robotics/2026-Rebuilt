package frc.robot.systemcheck;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.shooter.ShooterCalculator;
import frc.robot.commands.shooter.ShooterCommands;
import frc.robot.commands.shooter.ShotTargetSelector;
import frc.robot.constants.subsystems.IntakeConstants;
import frc.robot.constants.subsystems.ShooterConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.position_joint.PositionJoint;
import frc.robot.subsystems.vision.Vision;
import java.io.IOException;
import java.time.Instant;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.UUID;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Owns the Test-mode pit sequence, passive health reporting, and safe shutdown.
 */
public class SystemCheckManager {
	private final CommandSwerveDrivetrain drivetrain;
	private final PositionJoint intakeRack;
	private final Flywheel intakeRoller;
	private final Flywheel conveyor;
	private final Flywheel bottomIndexer;
	private final Flywheel topIndexer;
	private final PositionJoint hood;
	private final Flywheel shooter;
	private final Vision vision;
	private final BooleanSupplier shotPoseReady;
	private final SystemCheckFaultInjection faultInjection = new SystemCheckFaultInjection();
	private final SystemCheckCanMonitor canMonitor = new SystemCheckCanMonitor();
	private final SystemCheckReportWriter reportWriter;

	private final BooleanEntry setupConfirmed = booleanEntry("Controls/SetupConfirmed");
	private final BooleanEntry armed = booleanEntry("Controls/Armed");
	private final BooleanEntry start = booleanEntry("Controls/Start");
	private final BooleanEntry abort = booleanEntry("Controls/Abort");
	private final BooleanEntry practiceArmed = booleanEntry("PracticeShot/Armed");
	private final BooleanEntry practiceAreaClear = booleanEntry("PracticeShot/AreaClearConfirmed");
	private final BooleanEntry practiceOneFuel = booleanEntry("PracticeShot/ExactlyOneFuelConfirmed");
	private final BooleanEntry practiceStart = booleanEntry("PracticeShot/Start");
	private final BooleanEntry practiceMade = booleanEntry("PracticeShot/Made");
	private final BooleanEntry practiceMissed = booleanEntry("PracticeShot/Missed");

	private final NetworkTable dashboard = NetworkTableInstance.getDefault().getTable("SystemCheck");
	private final Map<String, CheckResult> results = new LinkedHashMap<>();
	private final List<Stage> stages = new ArrayList<>();
	private final Timer runTimer = new Timer();
	private final Timer stageTimer = new Timer();
	private final Timer stallTimer = new Timer();
	private final Timer practiceFeedTimer = new Timer();

	private SystemCheckRunState runState = SystemCheckRunState.DISARMED;
	private CheckStatus practiceStatus = CheckStatus.NOT_RUN;
	private Command activeCommand;
	private Command practiceCommand;
	private int stageIndex = -1;
	private int baselineRioBusOffCount;
	private Map<String, Integer> baselineBusOffCounts = Map.of();
	private List<CanBusSnapshot> canBusSnapshots = List.of();
	private double lastCanPollSeconds = Double.NEGATIVE_INFINITY;
	private double stageStableSince = Double.NaN;
	private double stageFailureConditionSince = Double.NaN;
	private boolean visionAcceptedDuringStage;
	private boolean practiceFed;
	private boolean restoreSafeHold;
	private boolean previousStart;
	private boolean previousAbort;
	private boolean previousPracticeStart;
	private boolean previousPracticeMade;
	private boolean previousPracticeMissed;
	private String blockingReason = "Enter enabled Test mode and confirm the checklist";
	private String runId = "";
	private Instant runStartedAt = Instant.EPOCH;
	private boolean reportWritten;
	private String reportIndexUrl = "";
	private String reportFallbackIndexUrl = "";
	private String latestReportHtmlUrl = "";
	private String latestReportJsonUrl = "";
	private String latestReportCsvUrl = "";
	private String reportStorageDirectory = "";
	private String reportError = "";

	/** Creates the check manager for every currently-instantiated mechanism. */
	public SystemCheckManager(CommandSwerveDrivetrain drivetrain, PositionJoint intakeRack, Flywheel intakeRoller,
			Flywheel conveyor, Flywheel bottomIndexer, Flywheel topIndexer, PositionJoint hood, Flywheel shooter,
			Vision vision, BooleanSupplier shotPoseReady) {
		this.drivetrain = drivetrain;
		this.intakeRack = intakeRack;
		this.intakeRoller = intakeRoller;
		this.conveyor = conveyor;
		this.bottomIndexer = bottomIndexer;
		this.topIndexer = topIndexer;
		this.hood = hood;
		this.shooter = shooter;
		this.vision = vision;
		this.shotPoseReady = shotPoseReady;
		SystemCheckReportWriter createdReportWriter = null;
		try {
			createdReportWriter = SystemCheckReportWriter.createDefault();
			reportIndexUrl = createdReportWriter.indexUrl();
			reportFallbackIndexUrl = RobotBase.isReal()
					? "http://10.83.24.2:" + SystemCheckConstants.REPORT_WEB_PORT + "/"
					: reportIndexUrl;
			reportStorageDirectory = createdReportWriter.reportDirectory().toString();
		} catch (IOException | RuntimeException exception) {
			reportError = "Could not initialize report service: " + exception.getMessage();
		}
		reportWriter = createdReportWriter;
		pollCanBuses();
		buildStages();
		publish();
	}

	/** Called every robot loop; passive reporting remains active in every mode. */
	public void periodic() {
		pollCanBusesIfDue();
		publishPassiveHealth();

		boolean abortValue = abort.get();
		if (abortValue && !previousAbort) {
			abort.set(false);
			if (activeCommand != null || practiceCommand != null) {
				abortActive("Operator abort requested");
			}
		}
		previousAbort = abortValue;

		java.util.Optional<String> activeSafetyFault = activeCommand != null || practiceCommand != null
				? safetyFault()
				: java.util.Optional.empty();
		if (activeSafetyFault.isPresent()) {
			abortActive(activeSafetyFault.get());
		}

		if (restoreSafeHold && DriverStation.isTestEnabled()) {
			restoreSafeHold = false;
			scheduleSafeHold();
		}

		boolean startValue = start.get();
		if (startValue && !previousStart) {
			start.set(false);
			requestNormalRun();
		}
		previousStart = startValue;
		boolean practiceStartValue = practiceStart.get();
		if (practiceStartValue && !previousPracticeStart) {
			practiceStart.set(false);
			requestPracticeShot();
		}
		previousPracticeStart = practiceStartValue;
		boolean practiceMadeValue = practiceMade.get();
		if (practiceMadeValue && !previousPracticeMade) {
			practiceMade.set(false);
			if (practiceStatus == CheckStatus.AWAITING_CONFIRMATION) {
				practiceStatus = CheckStatus.PASS;
			}
		}
		previousPracticeMade = practiceMadeValue;
		boolean practiceMissedValue = practiceMissed.get();
		if (practiceMissedValue && !previousPracticeMissed) {
			practiceMissed.set(false);
			if (practiceStatus == CheckStatus.AWAITING_CONFIRMATION) {
				practiceStatus = CheckStatus.FAIL;
			}
		}
		previousPracticeMissed = practiceMissedValue;

		if (activeCommand == null && practiceCommand == null && runState != SystemCheckRunState.COMPLETE
				&& runState != SystemCheckRunState.ABORTED && runState != SystemCheckRunState.BLOCKED) {
			String reason = startInterlockFailure();
			runState = reason.isEmpty() ? SystemCheckRunState.READY : SystemCheckRunState.DISARMED;
			blockingReason = reason;
		}
		publish();
	}

	/**
	 * Neutralizes hardware and prepares dashboard controls when Test mode begins.
	 */
	public void enterTestMode() {
		cancelCommands();
		neutralizeAll();
		resetRunTimersAndMomentaryControls();
		results.clear();
		runId = "";
		stageIndex = -1;
		practiceStatus = CheckStatus.NOT_RUN;
		runState = SystemCheckRunState.DISARMED;
		blockingReason = startInterlockFailure();
	}

	/**
	 * Cancels diagnostics and restores the disarmed state on any Test-mode exit.
	 */
	public void exitTestMode() {
		cancelCommands();
		neutralizeAll();
		resetRunTimersAndMomentaryControls();
		stageIndex = -1;
		runState = SystemCheckRunState.DISARMED;
		blockingReason = "Test mode is not enabled";
	}

	/** Immediately neutralizes everything when the robot is disabled. */
	public void disabled() {
		if (runState == SystemCheckRunState.COMPLETE && practiceCommand == null) {
			cancelCommands();
			neutralizeAll();
		} else if (activeCommand != null || practiceCommand != null) {
			abortActive("Robot disabled");
		} else {
			neutralizeAll();
		}
	}

	/** Current top-level run state, exposed for tests and other robot code. */
	public SystemCheckRunState getRunState() {
		return runState;
	}

	/** Immutable result snapshot for completed and active stages. */
	public Map<String, CheckResult> getResults() {
		return Map.copyOf(results);
	}

	/** Current controlled-area shot status. */
	public CheckStatus getPracticeStatus() {
		return practiceStatus;
	}

	private void requestNormalRun() {
		if (activeCommand != null && runState == SystemCheckRunState.COMPLETE && practiceCommand == null) {
			Command safeHold = activeCommand;
			activeCommand = null;
			safeHold.cancel();
		}
		if (activeCommand != null || practiceCommand != null) {
			blockingReason = "A system check is already active";
			return;
		}
		String reason = startInterlockFailure();
		if (!reason.isEmpty()) {
			runState = SystemCheckRunState.BLOCKED;
			blockingReason = reason;
			return;
		}
		armed.set(false);

		results.clear();
		for (Stage stage : stages) {
			results.put(stage.id(), CheckResult.notRun(stage.id(), stage.subsystem()));
		}
		runId = UUID.randomUUID().toString();
		runStartedAt = Instant.now();
		reportWritten = false;
		reportError = "";
		runState = SystemCheckRunState.COUNTDOWN;
		blockingReason = "";
		stageIndex = -1;
		pollCanBuses();
		baselineBusOffCounts = SystemCheckCanMonitor.busOffCounts(canBusSnapshots);
		baselineRioBusOffCount = RobotController.getCANStatus().busOffCount;
		runTimer.restart();
		stageTimer.restart();
		stallTimer.stop();
		stallTimer.reset();
		activeCommand = Commands.run(this::executeNormalRun, drivetrain, intakeRack, intakeRoller, conveyor,
				bottomIndexer, topIndexer, hood, shooter).finallyDo(this::normalRunEnded)
				.withName("FullRobotSystemCheck");
		CommandScheduler.getInstance().schedule(activeCommand);
	}

	private void executeNormalRun() {
		if (runState == SystemCheckRunState.COUNTDOWN) {
			neutralizeAll();
			if (stageTimer.hasElapsed(SystemCheckConstants.COUNTDOWN_SECONDS)) {
				runState = SystemCheckRunState.RUNNING;
				startStage(0);
			}
			return;
		}
		if (runState == SystemCheckRunState.COMPLETE) {
			holdCompletedSafeState();
			return;
		}
		if (runState != SystemCheckRunState.RUNNING || stageIndex < 0) {
			neutralizeAll();
			return;
		}

		Stage stage = stages.get(stageIndex);
		stage.action().run();
		if (stage.id().equals("vision-observation")) {
			visionAcceptedDuringStage |= vision.hasAcceptedTagLocalization();
		}
		Evaluation current = stage.evaluation().get();
		if (current.status() == CheckStatus.FAIL) {
			if (!Double.isFinite(stageFailureConditionSince)) {
				stageFailureConditionSince = stageTimer.get();
			}
		} else {
			stageFailureConditionSince = Double.NaN;
		}
		if (current.status() == CheckStatus.PASS || current.status() == CheckStatus.WARNING) {
			if (!Double.isFinite(stageStableSince)) {
				stageStableSince = stageTimer.get();
			}
		} else {
			stageStableSince = Double.NaN;
		}

		if (stageTimer.hasElapsed(stage.durationSeconds())) {
			finishStage(stage, current);
		}
	}

	private void startStage(int index) {
		neutralizeAll();
		stageIndex = index;
		stageTimer.restart();
		stageStableSince = Double.NaN;
		stageFailureConditionSince = Double.NaN;
		visionAcceptedDuringStage = false;
		Stage stage = stages.get(index);
		if (!stage.allowed().getAsBoolean()) {
			List<String> failedPrerequisites = stage.prerequisites().stream()
					.filter(prerequisite -> !passed(prerequisite)).toList();
			String explanation = "Skipped because prerequisite check(s) failed: "
					+ String.join(", ", failedPrerequisites);
			FailureContext failure = new FailureContext("PREREQUISITE_FAILURE", 0.0, 0.0, runTimer.get(),
					"Sequence dependency -> failed prerequisite -> combined motion withheld", failedPrerequisites,
					List.of());
			results.put(stage.id(), new CheckResult(stage.id(), stage.subsystem(), CheckStatus.SKIPPED, explanation,
					0.0, Map.of(), failure));
			advanceStage();
			return;
		}
		results.put(stage.id(),
				new CheckResult(stage.id(), stage.subsystem(), CheckStatus.RUNNING, "Running", 0.0, Map.of()));
	}

	private void finishStage(Stage stage, Evaluation evaluation) {
		CheckStatus status = evaluation.status();
		String explanation = evaluation.explanation();
		if (stage.requiredStableSeconds() > 0.0 && (status == CheckStatus.PASS || status == CheckStatus.WARNING)
				&& (!Double.isFinite(stageStableSince)
						|| stageTimer.get() - stageStableSince < stage.requiredStableSeconds())) {
			status = CheckStatus.FAIL;
			explanation = "Passing condition was not stable for the final " + stage.requiredStableSeconds()
					+ " seconds";
		}
		double declaredStageSeconds = stageTimer.get();
		FailureContext failure = status == CheckStatus.FAIL
				? createFailureContext(stage, categorizeFailure(explanation), explanation,
						Double.isFinite(stageFailureConditionSince) ? stageFailureConditionSince : declaredStageSeconds,
						declaredStageSeconds, List.of())
				: FailureContext.none();
		results.put(stage.id(), new CheckResult(stage.id(), stage.subsystem(), status, explanation,
				declaredStageSeconds, evaluation.measurements(), failure));
		advanceStage();
	}

	private void advanceStage() {
		neutralizeAll();
		if (stageIndex + 1 >= stages.size()) {
			runState = SystemCheckRunState.COMPLETE;
			runTimer.stop();
			stageTimer.stop();
			blockingReason = overallStatus() == CheckStatus.PASS ? "System check passed" : "Review recorded results";
			holdCompletedSafeState();
			writeFinalReport();
			return;
		}
		startStage(stageIndex + 1);
	}

	private void normalRunEnded(boolean interrupted) {
		neutralizeAll();
		activeCommand = null;
		if (interrupted && runState != SystemCheckRunState.COMPLETE && runState != SystemCheckRunState.ABORTED) {
			runTimer.stop();
			stageTimer.stop();
			runState = SystemCheckRunState.ABORTED;
			blockingReason = "System check command was interrupted";
			markRunningResultAborted(blockingReason);
			writeFinalReport();
		}
	}

	private void requestPracticeShot() {
		if (practiceCommand != null) {
			blockingReason = "Practice shot already active";
			return;
		}
		String reason = practiceInterlockFailure();
		if (!reason.isEmpty()) {
			practiceStatus = CheckStatus.FAIL;
			blockingReason = reason;
			return;
		}
		practiceArmed.set(false);
		if (activeCommand != null) {
			activeCommand.cancel();
		}
		practiceFed = false;
		practiceFeedTimer.stop();
		practiceFeedTimer.reset();
		practiceStatus = CheckStatus.RUNNING;
		practiceCommand = ShooterCommands.practiceHubShot(drivetrain, hood, shooter, bottomIndexer, topIndexer,
				conveyor, shotPoseReady, feeding -> {
					practiceFed |= feeding;
					if (feeding) {
						practiceFeedTimer.start();
					} else {
						practiceFeedTimer.stop();
					}
				}).finallyDo(this::practiceShotEnded).withName("ControlledAreaSystemCheckShot");
		CommandScheduler.getInstance().schedule(practiceCommand);
	}

	private void practiceShotEnded(boolean interrupted) {
		neutralizeAll();
		writeFinalReport();
		practiceCommand = null;
		if (runState == SystemCheckRunState.ABORTED || interrupted) {
			practiceStatus = CheckStatus.ABORTED;
		} else if (practiceFed && practiceFeedTimer.get() >= SystemCheckConstants.PRACTICE_FEED_SECONDS - 0.02) {
			practiceStatus = CheckStatus.AWAITING_CONFIRMATION;
			blockingReason = "Select Made or Missed to finalize the controlled-area shot";
		} else {
			practiceStatus = CheckStatus.FAIL;
			blockingReason = "The shot did not accumulate three ready feed seconds within the eight-second limit";
		}
		restoreSafeHold = runState != SystemCheckRunState.ABORTED;
	}

	private void scheduleSafeHold() {
		if (activeCommand != null || practiceCommand != null) {
			return;
		}
		activeCommand = Commands
				.run(this::holdCompletedSafeState, drivetrain, intakeRack, intakeRoller, conveyor, bottomIndexer,
						topIndexer, hood, shooter)
				.finallyDo(interrupted -> activeCommand = null).withName("SystemCheckSafeHold");
		CommandScheduler.getInstance().schedule(activeCommand);
	}

	private void abortActive(String reason) {
		runTimer.stop();
		stageTimer.stop();
		practiceFeedTimer.stop();
		runState = SystemCheckRunState.ABORTED;
		blockingReason = reason;
		markRunningResultAborted(reason);
		practiceStatus = practiceStatus == CheckStatus.RUNNING ? CheckStatus.ABORTED : practiceStatus;
		Command active = activeCommand;
		Command practice = practiceCommand;
		activeCommand = null;
		practiceCommand = null;
		if (active != null) {
			active.cancel();
		}
		if (practice != null) {
			practice.cancel();
		}
		neutralizeAll();
	}

	private void resetRunTimersAndMomentaryControls() {
		runTimer.stop();
		runTimer.reset();
		stageTimer.stop();
		stageTimer.reset();
		stallTimer.stop();
		stallTimer.reset();
		practiceFeedTimer.stop();
		practiceFeedTimer.reset();
		start.set(false);
		abort.set(false);
		setupConfirmed.set(false);
		armed.set(false);
		practiceStart.set(false);
		practiceArmed.set(false);
		practiceAreaClear.set(false);
		practiceOneFuel.set(false);
		practiceMade.set(false);
		practiceMissed.set(false);
		previousStart = false;
		previousAbort = false;
		previousPracticeStart = false;
		previousPracticeMade = false;
		previousPracticeMissed = false;
	}

	private void cancelCommands() {
		Command active = activeCommand;
		Command practice = practiceCommand;
		activeCommand = null;
		practiceCommand = null;
		if (active != null) {
			active.cancel();
		}
		if (practice != null) {
			practice.cancel();
		}
		restoreSafeHold = false;
	}

	private void markRunningResultAborted(String reason) {
		if (stageIndex >= 0 && stageIndex < stages.size()) {
			Stage stage = stages.get(stageIndex);
			if (results.getOrDefault(stage.id(), CheckResult.notRun(stage.id(), stage.subsystem()))
					.status() == CheckStatus.RUNNING) {
				double declaredStageSeconds = stageTimer.get();
				FailureContext failure = createFailureContext(stage, categorizeFailure(reason), reason,
						Double.isFinite(stageFailureConditionSince) ? stageFailureConditionSince : declaredStageSeconds,
						declaredStageSeconds, List.of());
				results.put(stage.id(), new CheckResult(stage.id(), stage.subsystem(), CheckStatus.ABORTED, reason,
						declaredStageSeconds, Map.of(), failure));
			}
		}
	}

	private String startInterlockFailure() {
		String environmentFailure = environmentInterlockFailure();
		if (!environmentFailure.isEmpty()) {
			return environmentFailure;
		}
		if (!setupConfirmed.get()) {
			return "Confirm blocks, no fuel/obstructions, clear personnel, and Disable operator readiness";
		}
		if (!armed.get()) {
			return "Arm the system check";
		}
		return "";
	}

	private String environmentInterlockFailure() {
		if (!DriverStation.isDSAttached()) {
			return "Driver Station is not connected";
		}
		if (!DriverStation.isTestEnabled()) {
			return "Enable Test mode in Driver Station";
		}
		if (DriverStation.isFMSAttached()) {
			return "FMS is attached";
		}
		if (DriverStation.isEStopped()) {
			return "Robot is e-stopped";
		}
		if (RobotController.isBrownedOut()) {
			return "Robot is browned out";
		}
		if (batteryVoltage() < SystemCheckConstants.MINIMUM_START_BATTERY_VOLTS) {
			return "Battery must be at least 11.5 V";
		}
		return "";
	}

	private String practiceInterlockFailure() {
		String common = environmentInterlockFailure();
		if (!common.isEmpty()) {
			return common;
		}
		if (runState != SystemCheckRunState.COMPLETE) {
			return "Complete the unloaded pit check first";
		}
		if (!requiredPracticeChecksPassed()) {
			return "Drivetrain, hood, shooter, conveyor, and indexer checks must pass";
		}
		if (!requiredPracticeHealthReady()) {
			return "A required practice-shot device has a current connection, fault, temperature, or telemetry failure";
		}
		if (!DriverStation.getAlliance().isPresent()) {
			return "Select an alliance in Driver Station";
		}
		if (!shotPoseReady.getAsBoolean()) {
			return "A valid field pose is required";
		}
		if (!ShooterCalculator.calculate(drivetrain.getState().Pose,
				ShotTargetSelector.hub(DriverStation.getAlliance().orElseThrow())).calibrated()) {
			return "Robot must be positioned inside the calibrated hub-shot range";
		}
		if (!practiceArmed.get()) {
			return "Arm the controlled-area shot separately";
		}
		if (!practiceAreaClear.get()) {
			return "Confirm the controlled practice area is clear";
		}
		if (!practiceOneFuel.get()) {
			return "Confirm exactly one fuel is manually loaded; a three-second feed launches all remaining fuel";
		}
		return "";
	}

	private java.util.Optional<String> safetyFault() {
		if (!DriverStation.isDSAttached()) {
			return java.util.Optional.of("Driver Station connection lost");
		}
		if (!DriverStation.isTestEnabled()) {
			return java.util.Optional.of("Enabled Test mode exited");
		}
		if (DriverStation.isFMSAttached()) {
			return java.util.Optional.of("FMS attached during system check");
		}
		if (DriverStation.isEStopped()) {
			return java.util.Optional.of("Robot e-stopped");
		}
		if (RobotController.isBrownedOut()) {
			return java.util.Optional.of("Robot browned out");
		}
		if (batteryVoltage() < SystemCheckConstants.ABORT_BATTERY_VOLTS) {
			return java.util.Optional.of("Battery fell below 9.5 V");
		}
		if (RobotController.getCANStatus().busOffCount > baselineRioBusOffCount) {
			return java.util.Optional.of("A CAN bus-off occurred on rio");
		}
		for (CanBusSnapshot bus : canBusSnapshots) {
			int baseline = baselineBusOffCounts.getOrDefault(bus.name(), bus.busOffCount());
			if (bus.busOffCount() > baseline) {
				return java.util.Optional.of("A CAN bus-off occurred on " + bus.name());
			}
		}
		for (DeviceHealth health : allHealth()) {
			if (health.hasNonFiniteFastMeasurement()) {
				return java.util.Optional.of(health.name() + " reported a non-finite measurement");
			}
			if (health.maximumTemperatureCelsius() >= SystemCheckConstants.TEMPERATURE_ABORT_CELSIUS) {
				return java.util.Optional.of(health.name() + " reached 85 C");
			}
		}
		if (hasNonFiniteRobotMeasurement()) {
			return java.util.Optional.of("A robot pose, encoder, or mechanism measurement became non-finite");
		}
		boolean stalled = allHealth().stream()
				.anyMatch(health -> health.hasStalledMotor(SystemCheckConstants.STALL_CURRENT_FRACTION,
						SystemCheckConstants.STALL_MINIMUM_NORMALIZED_VELOCITY));
		if (stalled) {
			if (!stallTimer.isRunning()) {
				stallTimer.restart();
			}
			if (stallTimer.hasElapsed(SystemCheckConstants.STALL_DEBOUNCE_SECONDS)) {
				return java.util.Optional.of("A mechanism stayed near its current limit without moving");
			}
		} else {
			stallTimer.stop();
			stallTimer.reset();
		}
		return java.util.Optional.empty();
	}

	private boolean hasNonFiniteRobotMeasurement() {
		SwerveDriveState state = drivetrain.getState();
		if (!Double.isFinite(state.Pose.getX()) || !Double.isFinite(state.Pose.getY())
				|| !Double.isFinite(state.Pose.getRotation().getRadians())
				|| !Double.isFinite(state.Speeds.vxMetersPerSecond) || !Double.isFinite(state.Speeds.vyMetersPerSecond)
				|| !Double.isFinite(state.Speeds.omegaRadiansPerSecond)) {
			return true;
		}
		for (var module : state.ModuleStates) {
			if (!Double.isFinite(module.speedMetersPerSecond) || !Double.isFinite(module.angle.getRadians())) {
				return true;
			}
		}
		return !Double.isFinite(intakeRack.getPosition()) || !Double.isFinite(intakeRack.getVelocity())
				|| !Double.isFinite(hood.getPosition()) || !Double.isFinite(hood.getVelocity())
				|| !Double.isFinite(intakeRoller.getVelocity()) || !Double.isFinite(conveyor.getVelocity())
				|| !Double.isFinite(bottomIndexer.getVelocity()) || !Double.isFinite(topIndexer.getVelocity())
				|| !Double.isFinite(shooter.getVelocity());
	}

	private void buildStages() {
		stages.add(stage("preflight", "Robot", SystemCheckConstants.PREFLIGHT_SECONDS, this::neutralizeAll,
				this::evaluatePreflight));
		stages.add(stableStage("swerve-forward-orientation", "Drivetrain", SystemCheckConstants.SWERVE_STAGE_SECONDS,
				() -> drivetrain.systemCheckPointWheels(Rotation2d.kZero), () -> evaluateSwerveAngle(Rotation2d.kZero),
				0.25));
		stages.add(stableStage("swerve-sideways-orientation", "Drivetrain", SystemCheckConstants.SWERVE_STAGE_SECONDS,
				() -> drivetrain.systemCheckPointWheels(Rotation2d.fromDegrees(90.0)),
				() -> evaluateSwerveAngle(Rotation2d.fromDegrees(90.0)), 0.25));
		stages.add(stableStage("swerve-x-lock", "Drivetrain", SystemCheckConstants.SWERVE_STAGE_SECONDS,
				drivetrain::systemCheckXLock, this::evaluateSwerveTargets, 0.25));
		stages.add(stableStage("swerve-forward-drive", "Drivetrain", SystemCheckConstants.SWERVE_STAGE_SECONDS,
				() -> drivetrain.systemCheckDrive(SystemCheckConstants.SWERVE_TEST_METERS_PER_SECOND),
				() -> evaluateSwerveSpeed(SystemCheckConstants.SWERVE_TEST_METERS_PER_SECOND), 0.25));
		stages.add(stableStage("swerve-reverse-drive", "Drivetrain", SystemCheckConstants.SWERVE_STAGE_SECONDS,
				() -> drivetrain.systemCheckDrive(-SystemCheckConstants.SWERVE_TEST_METERS_PER_SECOND),
				() -> evaluateSwerveSpeed(-SystemCheckConstants.SWERVE_TEST_METERS_PER_SECOND), 0.25));
		stages.add(jointStage("intake-rack-stow", "IntakeRack", 0.00));
		stages.add(jointStage("intake-rack-safe", "IntakeRack", 0.13));
		stages.add(jointStage("intake-rack-deploy", "IntakeRack", 0.30));
		stages.add(jointStage("intake-rack-return", "IntakeRack", 0.00));
		stages.add(rollerStage("intake-roller", "IntakeRoller", intakeRoller, -SystemCheckConstants.ROLLER_TEST_VOLTS));
		stages.add(rollerStage("conveyor", "Conveyor", conveyor, -SystemCheckConstants.ROLLER_TEST_VOLTS));
		stages.add(
				rollerStage("bottom-indexer", "BottomIndexer", bottomIndexer, -SystemCheckConstants.ROLLER_TEST_VOLTS));
		stages.add(rollerStage("top-indexer", "TopIndexer", topIndexer, SystemCheckConstants.ROLLER_TEST_VOLTS));
		stages.add(hoodStage("hood-test-position", SystemCheckConstants.HOOD_TEST_ROTATIONS));
		stages.add(hoodStage("hood-return", 0.0));
		stages.add(stableStage("shooter-flywheel", "ShooterFlywheel", SystemCheckConstants.SHOOTER_STAGE_SECONDS,
				() -> shooter.setVelocity(SystemCheckConstants.SHOOTER_TEST_RPS), this::evaluateShooter, 0.25));
		stages.add(new Stage("coordinated-acquisition", "IntakePath", SystemCheckConstants.ROLLER_STAGE_SECONDS,
				this::runAcquisitionCheck, this::evaluateAcquisitionCheck,
				() -> passed("intake-rack-deploy") && passed("intake-roller") && passed("conveyor")
						&& passed("bottom-indexer") && passed("top-indexer"),
				0.0, List.of("intake-rack-deploy", "intake-roller", "conveyor", "bottom-indexer", "top-indexer")));
		stages.add(new Stage("coordinated-feed-path", "FeedPath", SystemCheckConstants.ROLLER_STAGE_SECONDS,
				this::runFeedPathCheck, this::evaluateFeedPathCheck,
				() -> passed("conveyor") && passed("bottom-indexer") && passed("top-indexer"), 0.0,
				List.of("conveyor", "bottom-indexer", "top-indexer")));
		stages.add(stage("vision-observation", "Vision", SystemCheckConstants.VISION_STAGE_SECONDS, () -> {
		}, this::evaluateVision));
		stages.add(stage("neutral-verification", "Robot", SystemCheckConstants.NEUTRAL_STAGE_SECONDS,
				this::neutralizeAll, this::evaluateNeutral));
	}

	private Stage stage(String id, String subsystem, double duration, Runnable action,
			Supplier<Evaluation> evaluation) {
		return new Stage(id, subsystem, duration, action, evaluation, () -> true, 0.0, List.of());
	}

	private Stage stableStage(String id, String subsystem, double duration, Runnable action,
			Supplier<Evaluation> evaluation, double stableSeconds) {
		return new Stage(id, subsystem, duration, action, evaluation, () -> true, stableSeconds, List.of());
	}

	private Stage jointStage(String id, String subsystem, double position) {
		return new Stage(id, subsystem, SystemCheckConstants.JOINT_STAGE_SECONDS,
				() -> intakeRack.setPosition(position),
				() -> evaluateJoint(intakeRack, position, SystemCheckConstants.RACK_TOLERANCE_METERS), () -> true, 0.0,
				List.of());
	}

	private Stage hoodStage(String id, double position) {
		return new Stage(id, "Hood", SystemCheckConstants.HOOD_STAGE_SECONDS, () -> hood.setPosition(position),
				() -> evaluateJoint(hood, position, SystemCheckConstants.HOOD_TOLERANCE_ROTATIONS), () -> true, 0.0,
				List.of());
	}

	private Stage rollerStage(String id, String subsystem, Flywheel mechanism, double volts) {
		return new Stage(id, subsystem, SystemCheckConstants.ROLLER_STAGE_SECONDS, () -> mechanism.setVoltage(volts),
				() -> evaluateRoller(mechanism), () -> true, 0.0, List.of());
	}

	private Evaluation evaluatePreflight() {
		List<String> failures = new ArrayList<>();
		List<String> warnings = new ArrayList<>();
		for (DeviceHealth health : allHealth()) {
			if (!health.allConnected()) {
				failures.add(disconnectedDescription(health));
			}
			if (health.hasActiveFaults()) {
				failures.add(health.name() + " vendor fault (IDs " + Arrays.toString(health.deviceIds()) + ")");
			}
			if (health.maximumTemperatureCelsius() > SystemCheckConstants.TEMPERATURE_WARNING_CELSIUS) {
				warnings.add(health.name() + " above 70 C");
			}
		}
		if (RobotController.getCANStatus().percentBusUtilization > SystemCheckConstants.CAN_UTILIZATION_WARNING) {
			warnings.add("CAN utilization above 80%");
		}
		if (!DriverStation.isJoystickConnected(0)) {
			warnings.add("Driver controller on USB 0 is not connected");
		}
		if (!DriverStation.isJoystickConnected(1)) {
			warnings.add("Copilot controller on USB 1 is not connected");
		}
		Map<String, Double> values = values("BatteryVolts", batteryVoltage(), "CANUtilization",
				RobotController.getCANStatus().percentBusUtilization, "CANBusOffCount",
				RobotController.getCANStatus().busOffCount, "RoboRIOBrownedOut",
				RobotController.isBrownedOut() ? 1.0 : 0.0, "DriverControllerConnected",
				DriverStation.isJoystickConnected(0) ? 1.0 : 0.0, "CopilotControllerConnected",
				DriverStation.isJoystickConnected(1) ? 1.0 : 0.0);
		if (!failures.isEmpty()) {
			return new Evaluation(CheckStatus.FAIL, String.join(", ", failures), values);
		}
		if (!warnings.isEmpty()) {
			return new Evaluation(CheckStatus.WARNING, String.join(", ", warnings), values);
		}
		return new Evaluation(CheckStatus.PASS, "All configured devices responded", values);
	}

	private Evaluation evaluateSwerveAngle(Rotation2d target) {
		var moduleStates = drivetrain.getSystemCheckModuleStates();
		double maxError = Arrays.stream(moduleStates)
				.mapToDouble(module -> Math.abs(module.angle.minus(target).getDegrees())).max()
				.orElse(Double.POSITIVE_INFINITY);
		boolean connected = drivetrain.getHealthSnapshots().stream().allMatch(DeviceHealth::allConnected);
		return threshold(connected && maxError <= SystemCheckConstants.SWERVE_ANGLE_TOLERANCE_DEGREES,
				"All module angles are within tolerance",
				connected ? "Module angle error exceeds 5 degrees" : "A drivetrain device is disconnected",
				values("MaxAngleErrorDegrees", maxError));
	}

	private Evaluation evaluateSwerveTargets() {
		var moduleStates = drivetrain.getSystemCheckModuleStates();
		var moduleTargets = drivetrain.getSystemCheckModuleTargets();
		if (moduleStates.length != moduleTargets.length || moduleStates.length == 0) {
			return new Evaluation(CheckStatus.FAIL, "Module targets are unavailable", Map.of());
		}
		double maxError = 0.0;
		for (int i = 0; i < moduleStates.length; i++) {
			maxError = Math.max(maxError, Math.abs(moduleStates[i].angle.minus(moduleTargets[i].angle).getDegrees()));
		}
		boolean connected = drivetrain.getHealthSnapshots().stream().allMatch(DeviceHealth::allConnected);
		return threshold(connected && maxError <= SystemCheckConstants.SWERVE_ANGLE_TOLERANCE_DEGREES,
				"X-lock angles match optimized targets",
				connected ? "X-lock angle error exceeds 5 degrees" : "A drivetrain device is disconnected",
				values("MaxAngleErrorDegrees", maxError));
	}

	private Evaluation evaluateSwerveSpeed(double target) {
		var moduleStates = drivetrain.getSystemCheckModuleStates();
		if (moduleStates.length == 0) {
			return new Evaluation(CheckStatus.FAIL, "Module targets are unavailable", Map.of());
		}
		double maxError = 0.0;
		for (var moduleState : moduleStates) {
			double actualX = moduleState.speedMetersPerSecond * moduleState.angle.getCos();
			double actualY = moduleState.speedMetersPerSecond * moduleState.angle.getSin();
			maxError = Math.max(maxError, Math.hypot(actualX - target, actualY));
		}
		boolean connected = drivetrain.getHealthSnapshots().stream().allMatch(DeviceHealth::allConnected);
		return threshold(connected && maxError <= SystemCheckConstants.SWERVE_VELOCITY_TOLERANCE_METERS_PER_SECOND,
				"All drive motors and encoders tracked the target",
				connected ? "Module velocity error exceeds 0.15 m/s" : "A drivetrain device is disconnected",
				values("TargetMetersPerSecond", target, "MaxVelocityErrorMetersPerSecond", maxError));
	}

	private Evaluation evaluateJoint(PositionJoint joint, double target, double tolerance) {
		DeviceHealth health = joint.getHealthSnapshot();
		double error = Math.abs(joint.getPosition() - target);
		boolean pass = health.allConnected() && error <= tolerance && !faultInjection.frozenEncoder();
		Map<String, Double> measurements = values("Target", target, "Position", joint.getPosition(), "Error", error,
				"Velocity", joint.getVelocity());
		if (!pass && RobotBase.isSimulation() && joint == intakeRack && health.allConnected()
				&& !faultInjection.frozenEncoder() && error <= SystemCheckConstants.SIM_RACK_WARNING_TOLERANCE_METERS) {
			return new Evaluation(CheckStatus.WARNING,
					"Rack is within the 3 cm simulation band; the real-robot requirement remains 2 cm", measurements);
		}
		return threshold(pass, "Position and hardware are within tolerance", faultInjection.frozenEncoder()
				? "Injected frozen encoder"
				: health.allConnected() ? "Position did not reach tolerance" : "Motor or required encoder disconnected",
				measurements);
	}

	private Evaluation evaluateRoller(Flywheel mechanism) {
		DeviceHealth health = mechanism.getHealthSnapshot();
		double speed = Math.abs(mechanism.getVelocity());
		boolean followersMatch = health.normalizedSpeedsAgree(SystemCheckConstants.FOLLOWER_SPEED_MISMATCH_FRACTION);
		boolean pass = health.allConnected() && speed >= SystemCheckConstants.MINIMUM_ROLLER_SPEED_RPS
				&& followersMatch;
		String reason = !health.allConnected()
				? "Motor disconnected"
				: speed < SystemCheckConstants.MINIMUM_ROLLER_SPEED_RPS
						? "Velocity below 1 RPS"
						: "Paired normalized speeds differ by more than 25%";
		return threshold(pass, "Velocity, connection, and follower checks passed", reason,
				values("VelocityRPS", mechanism.getVelocity(), "MaxCurrentAmps", maximum(health.currentsAmps())));
	}

	private Evaluation evaluateShooter() {
		DeviceHealth health = shooter.getHealthSnapshot();
		double error = Math.abs(shooter.getVelocity() - SystemCheckConstants.SHOOTER_TEST_RPS);
		boolean pass = health.allConnected() && error <= SystemCheckConstants.SHOOTER_TOLERANCE_RPS
				&& health.normalizedSpeedsAgree(SystemCheckConstants.FOLLOWER_SPEED_MISMATCH_FRACTION);
		return threshold(pass, "Shooter held 10 RPS and paired speeds agree",
				!health.allConnected() ? "Shooter motor disconnected" : "Shooter speed/follower tolerance failed",
				values("VelocityRPS", shooter.getVelocity(), "ErrorRPS", error));
	}

	private void runAcquisitionCheck() {
		intakeRack.setPosition(IntakeConstants.RACK_PRESETS.DEPLOY.get());
		intakeRoller.setVoltage(-SystemCheckConstants.ROLLER_TEST_VOLTS);
		conveyor.setVoltage(SystemCheckConstants.ROLLER_TEST_VOLTS);
		bottomIndexer.setVoltage(SystemCheckConstants.ROLLER_TEST_VOLTS);
		topIndexer.setVoltage(-SystemCheckConstants.ROLLER_TEST_VOLTS);
	}

	private Evaluation evaluateAcquisitionCheck() {
		boolean pass = Math
				.abs(intakeRack.getPosition()
						- IntakeConstants.RACK_PRESETS.DEPLOY.get()) <= SystemCheckConstants.RACK_TOLERANCE_METERS
				&& mechanismsMoving(intakeRoller, conveyor, bottomIndexer, topIndexer);
		return threshold(pass, "Complete acquisition path moved inward", "One or more acquisition mechanisms stopped",
				Map.of());
	}

	private void runFeedPathCheck() {
		shooter.stop();
		conveyor.setVoltage(-SystemCheckConstants.ROLLER_TEST_VOLTS);
		bottomIndexer.setVoltage(-SystemCheckConstants.ROLLER_TEST_VOLTS);
		topIndexer.setVoltage(SystemCheckConstants.ROLLER_TEST_VOLTS);
	}

	private Evaluation evaluateFeedPathCheck() {
		boolean pass = Math.abs(shooter.getCommandedVoltage()) < 1e-9
				&& mechanismsMoving(conveyor, bottomIndexer, topIndexer);
		return threshold(pass, "Feed path moved while shooter remained disabled",
				"Feed path stopped or shooter was enabled", Map.of());
	}

	private Evaluation evaluateVision() {
		if (!vision.isConnected()) {
			return new Evaluation(CheckStatus.FAIL, "Vision source disconnected", Map.of());
		}
		Map<String, Double> values = values("LastAbsoluteCorrectionAgeSeconds",
				vision.getLastAbsoluteCorrectionAgeSeconds());
		return visionAcceptedDuringStage
				? new Evaluation(CheckStatus.PASS, "Accepted AprilTag localization observed", values)
				: new Evaluation(CheckStatus.WARNING,
						"Vision connected but no accepted AprilTag localization was observed", values);
	}

	private Evaluation evaluateNeutral() {
		double maxVoltage = maximumAbsolute(intakeRoller.getCommandedVoltage(), conveyor.getCommandedVoltage(),
				bottomIndexer.getCommandedVoltage(), topIndexer.getCommandedVoltage(), shooter.getCommandedVoltage(),
				intakeRack.getCommandedVoltage(), hood.getCommandedVoltage());
		double chassisSpeed = Math.hypot(drivetrain.getState().Speeds.vxMetersPerSecond,
				drivetrain.getState().Speeds.vyMetersPerSecond);
		boolean pass = maxVoltage < 1e-9 && chassisSpeed < 0.05
				&& Math.abs(drivetrain.getState().Speeds.omegaRadiansPerSecond) < Math.toRadians(2.0);
		return threshold(pass, "Every mechanism is neutral", "A mechanism remained commanded or moving",
				values("MaxCommandedVoltage", maxVoltage, "ChassisSpeedMetersPerSecond", chassisSpeed));
	}

	private boolean mechanismsMoving(Flywheel... mechanisms) {
		return Arrays.stream(mechanisms).allMatch(
				mechanism -> Math.abs(mechanism.getVelocity()) >= SystemCheckConstants.MINIMUM_ROLLER_SPEED_RPS);
	}

	private boolean passed(String id) {
		CheckStatus status = results.getOrDefault(id, CheckResult.notRun(id, "")).status();
		return status == CheckStatus.PASS || status == CheckStatus.WARNING;
	}

	private boolean requiredPracticeChecksPassed() {
		return passed("swerve-forward-drive") && passed("swerve-reverse-drive") && passed("hood-test-position")
				&& passed("shooter-flywheel") && passed("conveyor") && passed("bottom-indexer")
				&& passed("top-indexer");
	}

	private boolean requiredPracticeHealthReady() {
		List<DeviceHealth> requiredHealth = allHealth().stream()
				.filter(health -> health.name().startsWith("SwerveModule") || health.name().equals("Pigeon2")
						|| health.name().equals("Hood") || health.name().equals("ShooterFlywheel")
						|| health.name().equals("Conveyor") || health.name().equals("BottomIndexer")
						|| health.name().equals("TopIndexer"))
				.toList();
		return requiredHealth.size() >= 10 && requiredHealth.stream()
				.allMatch(health -> health.allConnected() && !health.hasActiveFaults()
						&& !health.hasNonFiniteFastMeasurement()
						&& health.maximumTemperatureCelsius() < SystemCheckConstants.TEMPERATURE_ABORT_CELSIUS);
	}

	private List<DeviceHealth> allHealth() {
		List<DeviceHealth> health = new ArrayList<>(drivetrain.getHealthSnapshots());
		health.add(intakeRack.getHealthSnapshot());
		health.add(intakeRoller.getHealthSnapshot());
		health.add(conveyor.getHealthSnapshot());
		health.add(bottomIndexer.getHealthSnapshot());
		health.add(topIndexer.getHealthSnapshot());
		health.add(hood.getHealthSnapshot());
		health.add(shooter.getHealthSnapshot());
		health.add(vision.getHealthSnapshot());
		return faultInjection.apply(health);
	}

	private void pollCanBusesIfDue() {
		double now = Timer.getFPGATimestamp();
		if (now - lastCanPollSeconds >= SystemCheckConstants.HEALTH_POLL_PERIOD_SECONDS) {
			pollCanBuses();
		}
	}

	private void pollCanBuses() {
		canBusSnapshots = canMonitor.poll();
		lastCanPollSeconds = Timer.getFPGATimestamp();
	}

	private FailureContext createFailureContext(Stage stage, String category, String explanation,
			double conditionBeganStageSeconds, double declaredStageSeconds, List<String> failedPrerequisites) {
		List<HardwareFault> hardwareFaults = findHardwareFaults(stage, explanation);
		String diagnosticPath = switch (category) {
			case "CAN_BUS_OFF", "DEVICE_DISCONNECTED" -> "Stage -> device health -> CAN network -> configured chain neighbors";
			case "VENDOR_FAULT" -> "Stage -> device health -> vendor fault field -> controller/CAN ID";
			case "OVER_TEMPERATURE" -> "Stage -> motor temperature -> mechanism controller";
			case "STALL_OR_OVERCURRENT" -> "Stage -> motor current limit -> insufficient measured movement";
			case "INVALID_TELEMETRY" -> "Stage -> non-finite sensor/motor telemetry -> source hardware";
			case "POSITION_OR_TIMEOUT" -> "Stage command -> position/angle feedback -> tolerance timeout";
			case "VELOCITY_OR_TIMEOUT" -> "Stage command -> velocity feedback -> tolerance timeout";
			case "SAFETY_ABORT" -> "Run safety interlock -> immediate whole-robot neutralization";
			default -> "Stage command -> measured condition -> final stage evaluation";
		};
		return new FailureContext(category, conditionBeganStageSeconds, declaredStageSeconds, runTimer.get(),
				diagnosticPath, failedPrerequisites, hardwareFaults);
	}

	private List<HardwareFault> findHardwareFaults(Stage stage, String explanation) {
		List<HardwareFault> faults = new ArrayList<>();
		for (DeviceHealth health : healthForStage(stage)) {
			boolean[] connections = health.connections();
			for (int i = 0; i < connections.length; i++) {
				if (!connections[i]) {
					int index = i;
					SystemCheckCanTopology.findDevice(health.name(), index, false).ifPresentOrElse(
							device -> faults.add(SystemCheckCanTopology.hardwareFault(device, "DISCONNECTED",
									"Controller did not return valid health telemetry")),
							() -> faults.add(genericHardwareFault(health.name() + " motor " + index, "DISCONNECTED",
									"Controller did not return valid health telemetry")));
				}
			}
			if (health.encoderExpected() && !health.encoderConnected()) {
				SystemCheckCanTopology.findDevice(health.name(), -1, true).ifPresentOrElse(
						device -> faults.add(SystemCheckCanTopology.hardwareFault(device, "ENCODER_DISCONNECTED",
								"Required external encoder did not return valid telemetry")),
						() -> faults.add(genericHardwareFault(health.name() + " external encoder",
								"ENCODER_DISCONNECTED", "Required external encoder did not return valid telemetry")));
			}
			String[] vendorFaults = health.activeFaults();
			for (int i = 0; i < vendorFaults.length; i++) {
				if (vendorFaults[i] != null && !vendorFaults[i].isBlank()) {
					int index = Math.min(i, Math.max(0, connections.length - 1));
					String vendorFault = vendorFaults[i];
					SystemCheckCanTopology.findDevice(health.name(), index, false).ifPresentOrElse(
							device -> faults
									.add(SystemCheckCanTopology.hardwareFault(device, "VENDOR_FAULT", vendorFault)),
							() -> faults.add(genericHardwareFault(health.name(), "VENDOR_FAULT", vendorFault)));
				}
			}
			double[] temperatures = health.temperaturesCelsius();
			for (int i = 0; i < temperatures.length; i++) {
				if (temperatures[i] >= SystemCheckConstants.TEMPERATURE_ABORT_CELSIUS) {
					int index = i;
					SystemCheckCanTopology.findDevice(health.name(), index, false).ifPresentOrElse(
							device -> faults.add(SystemCheckCanTopology.hardwareFault(device, "OVER_TEMPERATURE",
									temperatures[index] + " C")),
							() -> faults.add(genericHardwareFault(health.name(), "OVER_TEMPERATURE",
									temperatures[index] + " C")));
				}
			}
			double[] currents = health.currentsAmps();
			double[] currentLimits = health.currentLimitsAmps();
			double[] velocities = health.normalizedVelocities();
			int count = Math.min(Math.min(currents.length, currentLimits.length), velocities.length);
			for (int i = 0; i < count; i++) {
				if (Double.isFinite(currentLimits[i]) && currentLimits[i] > 0.0
						&& currents[i] >= currentLimits[i] * SystemCheckConstants.STALL_CURRENT_FRACTION
						&& Math.abs(velocities[i]) < SystemCheckConstants.STALL_MINIMUM_NORMALIZED_VELOCITY) {
					int index = i;
					String details = currents[i] + " A at " + velocities[i] + " normalized velocity";
					SystemCheckCanTopology.findDevice(health.name(), index, false).ifPresentOrElse(
							device -> faults
									.add(SystemCheckCanTopology.hardwareFault(device, "STALL_OR_OVERCURRENT", details)),
							() -> faults.add(genericHardwareFault(health.name(), "STALL_OR_OVERCURRENT", details)));
				}
			}
		}
		if (faults.isEmpty()) {
			faults.add(genericHardwareFault(stage.subsystem(), categorizeFailure(explanation), explanation));
		}
		return List.copyOf(faults);
	}

	private List<DeviceHealth> healthForStage(Stage stage) {
		return allHealth().stream().filter(health -> switch (stage.subsystem()) {
			case "Robot" -> true;
			case "Drivetrain" -> health.name().startsWith("SwerveModule") || health.name().equals("Pigeon2");
			case "IntakePath" -> health.name().equals("IntakeRack") || health.name().equals("IntakeRoller")
					|| health.name().equals("Conveyor") || health.name().equals("BottomIndexer")
					|| health.name().equals("TopIndexer");
			case "FeedPath" -> health.name().equals("Conveyor") || health.name().equals("BottomIndexer")
					|| health.name().equals("TopIndexer") || health.name().equals("ShooterFlywheel");
			default -> health.name().equals(stage.subsystem());
		}).toList();
	}

	private static HardwareFault genericHardwareFault(String hardware, String faultType, String details) {
		return new HardwareFault(hardware, faultType, details, "not a localized CAN fault", -1, -1, "unknown",
				"unknown", false);
	}

	private static String categorizeFailure(String reason) {
		String lower = reason.toLowerCase(java.util.Locale.ROOT);
		if (lower.contains("can bus-off")) {
			return "CAN_BUS_OFF";
		}
		if (lower.contains("disconnect")) {
			return "DEVICE_DISCONNECTED";
		}
		if (lower.contains("vendor fault")) {
			return "VENDOR_FAULT";
		}
		if (lower.contains("temperature") || lower.contains("reached 85")) {
			return "OVER_TEMPERATURE";
		}
		if (lower.contains("current limit") || lower.contains("without moving") || lower.contains("stall")) {
			return "STALL_OR_OVERCURRENT";
		}
		if (lower.contains("non-finite")) {
			return "INVALID_TELEMETRY";
		}
		if (lower.contains("position") || lower.contains("angle") || lower.contains("target")
				|| lower.contains("stable")) {
			return "POSITION_OR_TIMEOUT";
		}
		if (lower.contains("velocity") || lower.contains("speed") || lower.contains("rps")) {
			return "VELOCITY_OR_TIMEOUT";
		}
		if (lower.contains("abort") || lower.contains("disabled") || lower.contains("e-stop") || lower.contains("brown")
				|| lower.contains("battery") || lower.contains("test mode") || lower.contains("fms")
				|| lower.contains("driver station")) {
			return "SAFETY_ABORT";
		}
		return "FUNCTIONAL_FAILURE";
	}

	private void writeFinalReport() {
		if (reportWritten || runId.isBlank()) {
			return;
		}
		reportWritten = true;
		if (reportWriter == null) {
			reportError = "Report service is unavailable; the AdvantageKit WPILOG still contains SystemCheck data";
			return;
		}
		List<DeviceHealth> finalHealth = allHealth();
		SystemCheckRunReport report = new SystemCheckRunReport(runId, runStartedAt, Instant.now(), runState,
				overallStatus(), blockingReason, runTimer.get(), new ArrayList<>(results.values()), finalHealth,
				canBusSnapshots, SystemCheckCanTopology.diagnoseChains(finalHealth));
		try {
			SystemCheckReportWriter.ReportFiles files = reportWriter.write(report);
			latestReportHtmlUrl = files.htmlUrl();
			latestReportJsonUrl = files.jsonUrl();
			latestReportCsvUrl = files.csvUrl();
			reportError = "";
		} catch (IOException | RuntimeException exception) {
			reportError = "Could not write system-check report: " + exception.getMessage();
		}
	}

	private double batteryVoltage() {
		return faultInjection.batteryVoltage(RobotController.getBatteryVoltage());
	}

	private void neutralizeAll() {
		drivetrain.stop();
		intakeRack.stop();
		intakeRoller.stop();
		conveyor.stop();
		bottomIndexer.stop();
		topIndexer.stop();
		hood.stop();
		shooter.stop();
	}

	private void holdCompletedSafeState() {
		drivetrain.stop();
		intakeRoller.stop();
		conveyor.stop();
		bottomIndexer.stop();
		topIndexer.stop();
		shooter.stop();
		intakeRack.setPosition(IntakeConstants.RACK_PRESETS.STOW.get());
		hood.setPosition(ShooterConstants.HOOD_PRESET.STOW.get());
	}

	private CheckStatus overallStatus() {
		return CheckResult.aggregate(runState, results.values());
	}

	private void publishPassiveHealth() {
		for (DeviceHealth health : allHealth()) {
			String path = "Passive/" + health.name();
			dashboard.getEntry(path + "/Connected").setBoolean(health.allConnected());
			dashboard.getEntry(path + "/MaxTemperatureCelsius").setDouble(health.maximumTemperatureCelsius());
			dashboard.getEntry(path + "/CurrentsAmps").setDoubleArray(health.currentsAmps());
			dashboard.getEntry(path + "/NormalizedVelocities").setDoubleArray(health.normalizedVelocities());
			dashboard.getEntry(path + "/ActiveFaults").setStringArray(health.activeFaults());
			dashboard.getEntry(path + "/DeviceIds").setString(Arrays.toString(health.deviceIds()));
			Logger.recordOutput("SystemCheck/Passive/" + health.name() + "/Connected", health.allConnected());
			Logger.recordOutput("SystemCheck/Passive/" + health.name() + "/MaxTemperatureCelsius",
					health.maximumTemperatureCelsius());
			Logger.recordOutput("SystemCheck/Passive/" + health.name() + "/CurrentsAmps", health.currentsAmps());
			Logger.recordOutput("SystemCheck/Passive/" + health.name() + "/NormalizedVelocities",
					health.normalizedVelocities());
			Logger.recordOutput("SystemCheck/Passive/" + health.name() + "/ActiveFaults", health.activeFaults());
			Logger.recordOutput("SystemCheck/Passive/" + health.name() + "/DeviceIds",
					Arrays.toString(health.deviceIds()));
		}
		dashboard.getEntry("Passive/Vision/PoseInitialized").setBoolean(vision.isPoseReady());
		dashboard.getEntry("Passive/Vision/LastAbsoluteCorrectionAgeSeconds")
				.setDouble(vision.getLastAbsoluteCorrectionAgeSeconds());
		Logger.recordOutput("SystemCheck/Passive/Vision/PoseInitialized", vision.isPoseReady());
		Logger.recordOutput("SystemCheck/Passive/Vision/LastAbsoluteCorrectionAgeSeconds",
				vision.getLastAbsoluteCorrectionAgeSeconds());
		for (CanBusSnapshot bus : canBusSnapshots) {
			String path = "CAN/" + networkTableKey(bus.name());
			dashboard.getEntry(path + "/Name").setString(bus.name());
			dashboard.getEntry(path + "/Available").setBoolean(bus.available());
			dashboard.getEntry(path + "/Status").setString(bus.status());
			dashboard.getEntry(path + "/Utilization").setDouble(bus.utilization());
			dashboard.getEntry(path + "/BusOffCount").setDouble(bus.busOffCount());
			dashboard.getEntry(path + "/TxFullCount").setDouble(bus.txFullCount());
			dashboard.getEntry(path + "/ReceiveErrorCount").setDouble(bus.receiveErrorCount());
			dashboard.getEntry(path + "/TransmitErrorCount").setDouble(bus.transmitErrorCount());
			Logger.recordOutput("SystemCheck/" + path + "/Available", bus.available());
			Logger.recordOutput("SystemCheck/" + path + "/Status", bus.status());
			Logger.recordOutput("SystemCheck/" + path + "/Utilization", bus.utilization());
			Logger.recordOutput("SystemCheck/" + path + "/BusOffCount", bus.busOffCount());
			Logger.recordOutput("SystemCheck/" + path + "/TxFullCount", bus.txFullCount());
			Logger.recordOutput("SystemCheck/" + path + "/ReceiveErrorCount", bus.receiveErrorCount());
			Logger.recordOutput("SystemCheck/" + path + "/TransmitErrorCount", bus.transmitErrorCount());
		}
	}

	private void publish() {
		faultInjection.log();
		String currentStep = stageIndex >= 0 && stageIndex < stages.size() ? stages.get(stageIndex).id() : "None";
		double progress = stages.isEmpty() ? 0.0 : Math.max(0.0, Math.min(1.0, (stageIndex + 1.0) / stages.size()));
		dashboard.getEntry("RunState").setString(runState.toString());
		dashboard.getEntry("OverallStatus").setString(overallStatus().toString());
		dashboard.getEntry("CurrentStep").setString(currentStep);
		dashboard.getEntry("Progress").setDouble(progress);
		dashboard.getEntry("CountdownSeconds")
				.setDouble(runState == SystemCheckRunState.COUNTDOWN
						? Math.max(0.0, SystemCheckConstants.COUNTDOWN_SECONDS - stageTimer.get())
						: 0.0);
		dashboard.getEntry("BlockingReason").setString(blockingReason);
		dashboard.getEntry("RunId").setString(runId);
		dashboard.getEntry("RunElapsedSeconds").setDouble(runTimer.get());
		dashboard.getEntry("Report/IndexUrl").setString(reportIndexUrl);
		dashboard.getEntry("Report/FallbackIndexUrl").setString(reportFallbackIndexUrl);
		dashboard.getEntry("Report/LatestHtmlUrl").setString(latestReportHtmlUrl);
		dashboard.getEntry("Report/LatestJsonUrl").setString(latestReportJsonUrl);
		dashboard.getEntry("Report/LatestCsvUrl").setString(latestReportCsvUrl);
		dashboard.getEntry("Report/StorageDirectory").setString(reportStorageDirectory);
		dashboard.getEntry("Report/Error").setString(reportError);
		dashboard.getEntry("Report/CanTopologyPhysicalOrderVerified")
				.setBoolean(SystemCheckCanTopology.PHYSICAL_ORDER_VERIFIED);
		dashboard.getEntry("Report/Instructions")
				.setString("Open IndexUrl in a browser; use Download full JSON log or Download stage CSV.");
		dashboard.getEntry("PracticeShot/Status").setString(practiceStatus.toString());
		dashboard.getEntry("PracticeShot/Warning")
				.setString("Exactly one fuel only: the three-second feed can launch every fuel left in the robot.");
		Logger.recordOutput("SystemCheck/RunState", runState.toString());
		Logger.recordOutput("SystemCheck/OverallStatus", overallStatus().toString());
		Logger.recordOutput("SystemCheck/CurrentStep", currentStep);
		Logger.recordOutput("SystemCheck/Progress", progress);
		Logger.recordOutput("SystemCheck/BlockingReason", blockingReason);
		Logger.recordOutput("SystemCheck/RunId", runId);
		Logger.recordOutput("SystemCheck/RunElapsedSeconds", runTimer.get());
		Logger.recordOutput("SystemCheck/Report/IndexUrl", reportIndexUrl);
		Logger.recordOutput("SystemCheck/Report/FallbackIndexUrl", reportFallbackIndexUrl);
		Logger.recordOutput("SystemCheck/Report/LatestHtmlUrl", latestReportHtmlUrl);
		Logger.recordOutput("SystemCheck/Report/LatestJsonUrl", latestReportJsonUrl);
		Logger.recordOutput("SystemCheck/Report/LatestCsvUrl", latestReportCsvUrl);
		Logger.recordOutput("SystemCheck/Report/StorageDirectory", reportStorageDirectory);
		Logger.recordOutput("SystemCheck/Report/Error", reportError);
		Logger.recordOutput("SystemCheck/Report/CanTopologyPhysicalOrderVerified",
				SystemCheckCanTopology.PHYSICAL_ORDER_VERIFIED);
		Logger.recordOutput("SystemCheck/PracticeShot/Status", practiceStatus.toString());
		Logger.recordOutput("SystemCheck/Controls/SetupConfirmed", setupConfirmed.get());
		Logger.recordOutput("SystemCheck/Controls/Armed", armed.get());
		Logger.recordOutput("SystemCheck/Controls/Start", start.get());
		Logger.recordOutput("SystemCheck/Controls/Abort", abort.get());
		Logger.recordOutput("SystemCheck/PracticeShot/AreaClearConfirmed", practiceAreaClear.get());
		Logger.recordOutput("SystemCheck/PracticeShot/Armed", practiceArmed.get());
		Logger.recordOutput("SystemCheck/PracticeShot/ExactlyOneFuelConfirmed", practiceOneFuel.get());
		Logger.recordOutput("SystemCheck/Thresholds/MinimumStartBatteryVolts",
				SystemCheckConstants.MINIMUM_START_BATTERY_VOLTS);
		Logger.recordOutput("SystemCheck/Thresholds/AbortBatteryVolts", SystemCheckConstants.ABORT_BATTERY_VOLTS);
		Logger.recordOutput("SystemCheck/Thresholds/TemperatureWarningCelsius",
				SystemCheckConstants.TEMPERATURE_WARNING_CELSIUS);
		Logger.recordOutput("SystemCheck/Thresholds/TemperatureAbortCelsius",
				SystemCheckConstants.TEMPERATURE_ABORT_CELSIUS);
		Logger.recordOutput("SystemCheck/Thresholds/StallDebounceSeconds", SystemCheckConstants.STALL_DEBOUNCE_SECONDS);
		for (CheckResult result : results.values()) {
			String path = "Results/" + result.id();
			dashboard.getEntry(path + "/Status").setString(result.status().toString());
			dashboard.getEntry(path + "/Explanation").setString(result.explanation());
			dashboard.getEntry(path + "/ElapsedSeconds").setDouble(result.elapsedSeconds());
			Logger.recordOutput("SystemCheck/" + path + "/Status", result.status().toString());
			Logger.recordOutput("SystemCheck/" + path + "/Explanation", result.explanation());
			Logger.recordOutput("SystemCheck/" + path + "/ElapsedSeconds", result.elapsedSeconds());
			FailureContext failure = result.failureContext();
			dashboard.getEntry(path + "/FailureCategory").setString(failure.present() ? failure.category() : "NONE");
			dashboard.getEntry(path + "/FailureConditionBeganSeconds").setDouble(failure.conditionBeganStageSeconds());
			dashboard.getEntry(path + "/FailureDeclaredStageSeconds").setDouble(failure.declaredStageSeconds());
			dashboard.getEntry(path + "/FailureDeclaredRunSeconds").setDouble(failure.declaredRunSeconds());
			dashboard.getEntry(path + "/DiagnosticPath").setString(failure.diagnosticPath());
			Logger.recordOutput("SystemCheck/" + path + "/FailureCategory",
					failure.present() ? failure.category() : "NONE");
			Logger.recordOutput("SystemCheck/" + path + "/FailureConditionBeganSeconds",
					failure.conditionBeganStageSeconds());
			Logger.recordOutput("SystemCheck/" + path + "/FailureDeclaredStageSeconds", failure.declaredStageSeconds());
			Logger.recordOutput("SystemCheck/" + path + "/FailureDeclaredRunSeconds", failure.declaredRunSeconds());
			Logger.recordOutput("SystemCheck/" + path + "/DiagnosticPath", failure.diagnosticPath());
			for (Map.Entry<String, Double> measurement : result.measurements().entrySet()) {
				dashboard.getEntry(path + "/Measurements/" + measurement.getKey()).setDouble(measurement.getValue());
				Logger.recordOutput("SystemCheck/" + path + "/Measurements/" + measurement.getKey(),
						measurement.getValue());
			}
		}
	}

	private static Evaluation threshold(boolean pass, String passMessage, String failMessage,
			Map<String, Double> measurements) {
		return new Evaluation(pass ? CheckStatus.PASS : CheckStatus.FAIL, pass ? passMessage : failMessage,
				measurements);
	}

	private static Map<String, Double> values(Object... entries) {
		Map<String, Double> values = new LinkedHashMap<>();
		for (int i = 0; i + 1 < entries.length; i += 2) {
			values.put((String) entries[i], ((Number) entries[i + 1]).doubleValue());
		}
		return values;
	}

	private static double maximum(double[] values) {
		return Arrays.stream(values).max().orElse(0.0);
	}

	private static double maximumAbsolute(double... values) {
		return Arrays.stream(values).map(Math::abs).max().orElse(0.0);
	}

	private static String disconnectedDescription(DeviceHealth health) {
		List<String> devices = new ArrayList<>();
		boolean[] connections = health.connections();
		int[] ids = health.deviceIds();
		for (int i = 0; i < connections.length; i++) {
			if (!connections[i]) {
				devices.add(i < ids.length ? "CAN " + ids[i] : "motor " + i);
			}
		}
		if (health.encoderExpected() && !health.encoderConnected()) {
			devices.add(ids.length > connections.length ? "encoder ID " + ids[ids.length - 1] : "external encoder");
		}
		return health.name() + " disconnected: " + String.join(", ", devices);
	}

	private static String networkTableKey(String value) {
		return value.replaceAll("[^A-Za-z0-9_-]", "_");
	}

	private static BooleanEntry booleanEntry(String key) {
		BooleanEntry entry = NetworkTableInstance.getDefault().getTable("SystemCheck").getBooleanTopic(key)
				.getEntry(false);
		entry.setDefault(false);
		return entry;
	}

	private record Evaluation(CheckStatus status, String explanation, Map<String, Double> measurements) {
	}

	private record Stage(String id, String subsystem, double durationSeconds, Runnable action,
			Supplier<Evaluation> evaluation, BooleanSupplier allowed, double requiredStableSeconds,
			List<String> prerequisites) {
		private Stage {
			prerequisites = List.copyOf(prerequisites);
		}
	}
}
