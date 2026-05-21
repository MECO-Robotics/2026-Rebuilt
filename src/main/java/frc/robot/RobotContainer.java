package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.drive.choreo.ChoreoTraj;
import frc.robot.commands.flywheel.FlywheelSysIdCommands;
import frc.robot.commands.position_joint.PositionJointSysIdCommands;
import frc.robot.commands.shooter.ShooterCalculator;
import frc.robot.commands.shooter.ShooterCommands;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants.AutoDrivePoses;
import frc.robot.constants.drive.DrivetrainConstants;
import frc.robot.constants.subsystems.IntakeConstants;
import frc.robot.constants.subsystems.ShooterConstants;
import frc.robot.constants.vision.VisionConstants;
import frc.robot.simulation.RobotSimulation;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.position_joint.PositionJoint;
import frc.robot.subsystems.position_joint.PositionJointIO;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOQuestNav;
import frc.robot.util.HubShiftUtil;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	private static final double AUTO_LOOP_PERIOD_SECONDS = 0.020;
	private static final double[] DEFAULT_CLICKED_POSE = new double[]{0.0, 0.0, 0.0};
	private static final String DRIVE_TO_POSE_X_KEY = "Drive To Pose/Target X Meters";
	private static final String DRIVE_TO_POSE_Y_KEY = "Drive To Pose/Target Y Meters";
	private static final String DRIVE_TO_POSE_HEADING_KEY = "Drive To Pose/Target Heading Degrees";
	private static final AutoDriveTarget[] AUTO_DRIVE_TARGETS = new AutoDriveTarget[]{
			new AutoDriveTarget("Fender", AutoDrivePoses.FENDER),
			new AutoDriveTarget("Left Trench", AutoDrivePoses.LEFT_TRENCH),
			new AutoDriveTarget("Left Trench Return", AutoDrivePoses.LEFT_TRENCH_RETURN),
			new AutoDriveTarget("Left Trench Far", AutoDrivePoses.LEFT_TRENCH_FAR),
			new AutoDriveTarget("Left Trench Corner", AutoDrivePoses.LEFT_TRENCH_CORNER),
			new AutoDriveTarget("Left Trench Cleanup", AutoDrivePoses.LEFT_TRENCH_CLEANUP),
			new AutoDriveTarget("Right Trench", AutoDrivePoses.RIGHT_TRENCH),
			new AutoDriveTarget("Right Trench Return", AutoDrivePoses.RIGHT_TRENCH_RETURN),
			new AutoDriveTarget("Right Trench Far", AutoDrivePoses.RIGHT_TRENCH_FAR),
			new AutoDriveTarget("Right Trench Corner", AutoDrivePoses.RIGHT_TRENCH_CORNER),
			new AutoDriveTarget("Right Trench Cleanup", AutoDrivePoses.RIGHT_TRENCH_CLEANUP),
			new AutoDriveTarget("Left Depot", AutoDrivePoses.LEFT_DEPOT),
			new AutoDriveTarget("Right Depot", AutoDrivePoses.RIGHT_DEPOT),
			new AutoDriveTarget("Left Rush Start", AutoDrivePoses.LEFT_RUSH_START),
			new AutoDriveTarget("Right Rush Start", AutoDrivePoses.RIGHT_RUSH_START),
			new AutoDriveTarget("Left Bump", AutoDrivePoses.LEFT_BUMP),
			new AutoDriveTarget("Left Bump Far", AutoDrivePoses.LEFT_BUMP_FAR),
			new AutoDriveTarget("Right Bump", AutoDrivePoses.RIGHT_BUMP),
			new AutoDriveTarget("Right Bump Near", AutoDrivePoses.RIGHT_BUMP_NEAR),
			new AutoDriveTarget("Right Bump Far", AutoDrivePoses.RIGHT_BUMP_FAR),
			new AutoDriveTarget("Right Bump Corner", AutoDrivePoses.RIGHT_BUMP_CORNER),
			new AutoDriveTarget("Right Bump Cleanup", AutoDrivePoses.RIGHT_BUMP_CLEANUP),
			new AutoDriveTarget("Left Prom Counter", AutoDrivePoses.LEFT_PROM_COUNTER),
			new AutoDriveTarget("Left Prom Counter Send", AutoDrivePoses.LEFT_PROM_COUNTER_SEND),
			new AutoDriveTarget("Left Prom Counter Cleanup", AutoDrivePoses.LEFT_PROM_COUNTER_CLEANUP),
			new AutoDriveTarget("Right Prom Counter", AutoDrivePoses.RIGHT_PROM_COUNTER),
			new AutoDriveTarget("Right Prom Counter Send", AutoDrivePoses.RIGHT_PROM_COUNTER_SEND),
			new AutoDriveTarget("Right Prom Counter Cleanup", AutoDrivePoses.RIGHT_PROM_COUNTER_CLEANUP)};

	// Subsystems
	public final CommandSwerveDrivetrain drivetrain = DrivetrainConstants.createDrivetrain();
	private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
			.withDeadband(DrivetrainConstants.MAX_SPEED * 0.05)
			.withRotationalDeadband(DrivetrainConstants.MAX_ANGULAR_RATE * 0.05) // Add a 5% deadband
			.withDriveRequestType(DriveRequestType.Velocity); // Use closed-loop control for drive motors
	private final SwerveRequest.ApplyRobotSpeeds choreoDrive = new SwerveRequest.ApplyRobotSpeeds();
	private final PPHolonomicDriveController choreoController = new PPHolonomicDriveController(
			new PIDConstants(10, 0, 0), new PIDConstants(7, 0, 0), AUTO_LOOP_PERIOD_SECONDS);

	private final Flywheel shooterFlywheel;
	private final Flywheel topIndexer;
	private final Flywheel bottomIndexer;
	private final Flywheel conveyor;
	private final Flywheel intakeRoller;
	private final PositionJoint intakeRack;
	private final PositionJoint hood;
	private final Vision vision;
	private final RobotSimulation simulation;

	// Controller
	private final CommandXboxController controller = new CommandXboxController(0);
	private final CommandXboxController coPilot = new CommandXboxController(1);

	// Dashboard inputs
	private final LoggedDashboardChooser<Command> autoChooser;
	private final LoggedDashboardChooser<Command> sysIdChooser;
	private final SendableChooser<Pose2d> driveToPoseChooser = new SendableChooser<>();
	private final Field2d autoDriveField = new Field2d();
	private final NetworkTable autoDriveTable = NetworkTableInstance.getDefault().getTable("AutoDrive");
	private final DoubleArrayPublisher clickedPosePublisher = autoDriveTable.getDoubleArrayTopic("ClickedPose")
			.publish();
	private final IntegerPublisher clickedPoseRequestPublisher = autoDriveTable.getIntegerTopic("RequestId").publish();
	private final DoubleArraySubscriber clickedPoseSubscriber = autoDriveTable.getDoubleArrayTopic("ClickedPose")
			.subscribe(DEFAULT_CLICKED_POSE);
	private final IntegerSubscriber clickedPoseRequestSubscriber = autoDriveTable.getIntegerTopic("RequestId")
			.subscribe(0);
	private long lastClickedPoseRequestId = 0;
	private final AutoFactory choreoAutoFactory;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		RobotController.setBrownoutVoltage(6.0);

		topIndexer = new Flywheel(FlywheelIO.fromSparkMax("TopIndexer", ShooterConstants.TOP_INDEXER_ROLLER_CONFIG),
				ShooterConstants.INDEXER_ROLLER_GAINS);

		bottomIndexer = new Flywheel(
				FlywheelIO.fromTalonFX("BottomIndexer", ShooterConstants.BOTTOM_INDEXER_ROLLER_CONFIG),
				ShooterConstants.INDEXER_ROLLER_GAINS);

		conveyor = new Flywheel(FlywheelIO.fromSparkMax("Conveyor", ShooterConstants.CONVEYOR_CONFIG),
				ShooterConstants.CONVEYOR_GAINS);

		shooterFlywheel = new Flywheel(
				FlywheelIO.fromTalonFX("ShooterFlywheel", ShooterConstants.FLYWHEEL_ROLLER_CONFIG),
				ShooterConstants.FLYWHEEL_ROLLER_GAINS);

		intakeRoller = new Flywheel(FlywheelIO.fromSparkMax("IntakeRoller", IntakeConstants.INTAKE_ROLLER_CONFIG),
				IntakeConstants.INTAKE_ROLLER_GAINS);

		intakeRack = new PositionJoint(PositionJointIO.fromSparkMax("IntakeRack", IntakeConstants.INTAKE_RACK_CONFIG),
				IntakeConstants.INTAKE_RACK_GAINS);
		hood = new PositionJoint(PositionJointIO.fromSparkMax("Hood", ShooterConstants.HOOD_CONFIG),
				ShooterConstants.HOOD_GAINS);
		vision = new Vision(drivetrain::addVisionMeasurement,
				new VisionIOQuestNav(VisionConstants.robotToQuest,
						VisionIO.limelightMegaTag1WithMegaTag2SingleTagWithSim(VisionConstants.limelightName,
								() -> drivetrain.getState().Pose.getRotation(), VisionConstants.robotToLimelight,
								drivetrain::getPhysicsPose)));
		simulation = RobotSimulation.create(drivetrain, intakeRack, hood, shooterFlywheel);
		simulation.bindCommandHooks();
		choreoAutoFactory = new AutoFactory(() -> drivetrain.getState().Pose, drivetrain::resetPose,
				this::followChoreoSample, true, drivetrain);

		registerNamedCommands();
		// Keep PathPlanner's built-in chooser behavior (default option is "None").
		autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
		sysIdChooser = new LoggedDashboardChooser<>("SysId");
		configureAuto();
		if (Constants.ENABLE_SYSID_AUTOS) {
			configureSysIdChooser();
		}

		SmartDashboard.putBoolean("Flywheel Spinning?", false);
		configureDriveToPoseDashboard();

		// Configure the button bindings
		configureButtonBindings();
	}

	private void configureDriveToPoseDashboard() {
		SmartDashboard.putNumber(DRIVE_TO_POSE_X_KEY, drivetrain.getState().Pose.getX());
		SmartDashboard.putNumber(DRIVE_TO_POSE_Y_KEY, drivetrain.getState().Pose.getY());
		SmartDashboard.putNumber(DRIVE_TO_POSE_HEADING_KEY, drivetrain.getState().Pose.getRotation().getDegrees());
		addDriveToPoseOption("Manual Fields", null, true);
		for (AutoDriveTarget target : AUTO_DRIVE_TARGETS) {
			addDriveToPoseOption(target.name, target.pose, false);
		}
		SmartDashboard.putData("Drive To Pose/Target", driveToPoseChooser);
		SmartDashboard.putData("Drive To Pose/Run",
				DriveCommands.pathfindAndAlignToPose(drivetrain, this::getDashboardDriveToPose)
						.withName("Pathfind And Align To Pose"));
		configureAutoDriveShuffleboard();
		configureClickedPoseInput();
		updateAutoDriveField();
	}

	private void addDriveToPoseOption(String name, Pose2d pose, boolean isDefault) {
		if (isDefault) {
			driveToPoseChooser.setDefaultOption(name, pose);
		} else {
			driveToPoseChooser.addOption(name, pose);
		}
	}

	private Pose2d getDashboardDriveToPose() {
		Pose2d selectedPose = driveToPoseChooser.getSelected();
		if (selectedPose != null) {
			return selectedPose;
		}

		return new Pose2d(SmartDashboard.getNumber(DRIVE_TO_POSE_X_KEY, drivetrain.getState().Pose.getX()),
				SmartDashboard.getNumber(DRIVE_TO_POSE_Y_KEY, drivetrain.getState().Pose.getY()),
				Rotation2d.fromDegrees(SmartDashboard.getNumber(DRIVE_TO_POSE_HEADING_KEY,
						drivetrain.getState().Pose.getRotation().getDegrees())));
	}

	private void configureAutoDriveShuffleboard() {
		ShuffleboardTab autoDriveTab = Shuffleboard.getTab("Auto Drive");
		autoDriveTab.add("Field", autoDriveField).withWidget(BuiltInWidgets.kField).withPosition(0, 0).withSize(8, 5);
		autoDriveTab.add("Target", driveToPoseChooser).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(8, 0)
				.withSize(3, 1);
		autoDriveTab
				.add("Run Selected",
						DriveCommands.pathfindAndAlignToPose(drivetrain, this::getDashboardDriveToPose)
								.withName("Run Selected"))
				.withWidget(BuiltInWidgets.kCommand).withPosition(8, 1).withSize(3, 1);

		for (int i = 0; i < AUTO_DRIVE_TARGETS.length; i++) {
			AutoDriveTarget target = AUTO_DRIVE_TARGETS[i];
			int column = 8 + (i / 7) * 3;
			int row = 2 + (i % 7);
			autoDriveTab
					.add(target.name,
							DriveCommands.pathfindAndAlignToPose(drivetrain, () -> target.pose).withName(target.name))
					.withWidget(BuiltInWidgets.kCommand).withPosition(column, row).withSize(3, 1);
		}
	}

	private void updateAutoDriveField() {
		autoDriveField.setRobotPose(drivetrain.getState().Pose);
		Pose2d targetPose = getDashboardDriveToPose();
		autoDriveField.getObject("Selected Target").setPose(targetPose);
		for (AutoDriveTarget target : AUTO_DRIVE_TARGETS) {
			autoDriveField.getObject(target.name).setPose(target.pose);
		}
	}

	private void configureClickedPoseInput() {
		clickedPosePublisher.set(DEFAULT_CLICKED_POSE);
		clickedPoseRequestPublisher.set(lastClickedPoseRequestId);
	}

	private void pollClickedPoseInput() {
		long requestId = clickedPoseRequestSubscriber.get();
		if (requestId == lastClickedPoseRequestId) {
			return;
		}
		lastClickedPoseRequestId = requestId;

		double[] clickedPose = clickedPoseSubscriber.get();
		if (clickedPose.length < 2 || DriverStation.isDisabled()) {
			return;
		}

		double headingDegrees = clickedPose.length >= 3
				? clickedPose[2]
				: drivetrain.getState().Pose.getRotation().getDegrees();
		Pose2d targetPose = new Pose2d(clickedPose[0], clickedPose[1], Rotation2d.fromDegrees(headingDegrees));
		SmartDashboard.putNumber(DRIVE_TO_POSE_X_KEY, targetPose.getX());
		SmartDashboard.putNumber(DRIVE_TO_POSE_Y_KEY, targetPose.getY());
		SmartDashboard.putNumber(DRIVE_TO_POSE_HEADING_KEY, targetPose.getRotation().getDegrees());
		autoDriveField.getObject("Clicked Target").setPose(targetPose);
		CommandScheduler.getInstance()
				.schedule(DriveCommands.pathfindAndAlignToPose(drivetrain, () -> targetPose).withName("Clicked Pose"));
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by instantiating a {@link GenericHID} or one of its subclasses
	 * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
	 * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureButtonBindings() {
		// ************************** DRIVETRAIN KEYBINDS **************************
		// Default command, normal field-relative drive
		drivetrain.setDefaultCommand(
				// Drivetrain will execute this command periodically
				drivetrain
						.applyRequest(() -> drive.withVelocityX(-controller.getLeftY() * DrivetrainConstants.MAX_SPEED) // Drive
																														// forward
								// with negative
								// Y (forward)
								.withVelocityY(-controller.getLeftX() * DrivetrainConstants.MAX_SPEED) // Drive left
																										// with
																										// negative X
																										// (left)
								.withRotationalRate(-controller.getRightX() * DrivetrainConstants.MAX_ANGULAR_RATE) // Drive
																													// counterclockwise
																													// with
																													// negative
																													// X
																													// (left)
						));
		// Reset heading
		controller.start().onTrue(DriveCommands.resetHeading(drivetrain));

		// ************************** INTAKE KEYBINDS **************************
		intakeRack.setDefaultCommand(PositionJoint.setVelocity(intakeRack, () -> 0.0));
		intakeRoller.setDefaultCommand(Flywheel.setVelocity(intakeRoller, () -> 0.0));
		// Run intake
		controller.leftBumper().whileTrue(IntakeCommands.spinIntake(intakeRoller))
				.whileFalse(IntakeCommands.idleIntake(intakeRoller));
		controller.leftTrigger().whileTrue(IntakeCommands.reverseIntake(intakeRoller))
				.whileFalse(IntakeCommands.idleIntake(intakeRoller));

		// Deploy intake setpoint
		controller.povUp().whileTrue(IntakeCommands.deployIntake(intakeRack, intakeRoller));
		// Stow intake setpoint
		controller.povDown().whileTrue(IntakeCommands.stowIntake(intakeRack, intakeRoller, conveyor))
				.onFalse(IntakeCommands.idle(intakeRack, intakeRoller, conveyor));

		// Deploy intake backup for intake skipping
		coPilot.povUp().whileTrue(IntakeCommands.deployIntakeVelocity(intakeRack, intakeRoller));
		// Stow intake backup for intake skipping
		coPilot.povDown().whileTrue(IntakeCommands.stowIntakeVelocity(intakeRack, intakeRoller, conveyor))
				.onFalse(IntakeCommands.idle(intakeRack, intakeRoller, conveyor));

		// ************************** SHOOTER KEYBINDS **************************
		// Feed shooter
		controller.rightBumper()
				.whileTrue(ShooterCommands.agitateIntake(bottomIndexer, topIndexer)
						.alongWith(IntakeCommands.stowIntake(intakeRack, intakeRoller, conveyor)))
				.whileFalse(ShooterCommands.idleRollers(bottomIndexer, topIndexer, conveyor)
						.alongWith(IntakeCommands.deployIntake(intakeRack, intakeRoller)));

		controller.a()
				.whileTrue(Commands.parallel(
						DriveCommands.joystickAimToHub(drivetrain, () -> -controller.getLeftY(),
								() -> -controller.getLeftX(), DrivetrainConstants.MAX_SPEED),
						ShooterCalculator.calculateAndShoot(drivetrain, hood, shooterFlywheel)))
				.whileFalse(ShooterCommands.shooterIdle(shooterFlywheel, hood));

		controller.y().onTrue(ShooterCommands.ferryPreset(shooterFlywheel, hood).repeatedly());

		// Shooter presets
		controller.b().or(coPilot.b()).whileTrue(ShooterCommands.shooterIdle(shooterFlywheel, hood));

		controller.x().or(coPilot.x()).onTrue(ShooterCommands.hubPreset(shooterFlywheel, hood).repeatedly());

		coPilot.y().onTrue(ShooterCommands.ferryPreset(shooterFlywheel, hood).repeatedly());

		coPilot.a().onTrue(ShooterCommands.trenchPreset(shooterFlywheel, hood).repeatedly());
	}

	public void updateDashboardOutputs() {
		pollClickedPoseInput();
		updateAutoDriveField();
		HubShiftUtil.ShiftInfo shiftInfo = HubShiftUtil.getShiftedShiftInfo();

		// Publish match time
		double matchTime = Math.max(0.0, shiftInfo.matchTime());
		int minutes = (int) matchTime / 60;
		int seconds = (int) matchTime % 60;
		int tenths = (int) ((matchTime * 10) % 10);
		SmartDashboard.putString("Match Time", String.format("%02d:%02d.%d", minutes, seconds, tenths));

		// Update from HubShiftUtil
		SmartDashboard.putString("Shifts/Remaining Shift Time",
				String.format("%.1f", Math.max(shiftInfo.remainingTime(), 0.0)));
		SmartDashboard.putBoolean("Shifts/Shift Active", shiftInfo.active());
		SmartDashboard.putString("Shifts/Game State",
				DriverStation.isAutonomous() ? "Autonomous" : shiftInfo.currentShift().toString());
		SmartDashboard.putBoolean("Shifts/Active First?",
				DriverStation.getAlliance().orElse(Alliance.Blue) == HubShiftUtil.getFirstActiveAlliance());
		SmartDashboard.putBoolean("AutoDrive/Is Red Alliance",
				DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red);
		SmartDashboard.putBoolean("AutoDrive/Enabled", DriverStation.isEnabled());
		SmartDashboard.putString("AutoDrive/Mode",
				DriverStation.isAutonomous() ? "Autonomous" : DriverStation.isTeleop() ? "Teleop" : "Disabled");

		SmartDashboard.putString("Shifts/Match Time Color", shiftInfo.matchTimeColor());
		SmartDashboard.putString("Shifts/Shift Time Color", shiftInfo.shiftTimeColor());
	}

	private static class AutoDriveTarget {
		private final String name;
		private final Pose2d pose;

		private AutoDriveTarget(String name, Pose2d pose) {
			this.name = name;
			this.pose = pose;
		}
	}

	private void registerNamedCommands() {
		NamedCommands.registerCommand("DeployIntake", IntakeCommands.deployIntake(intakeRack, intakeRoller));
		NamedCommands.registerCommand("StowIntake", IntakeCommands.stowIntake(intakeRack, intakeRoller, conveyor));
		NamedCommands.registerCommand("FeedRollers",
				ShooterCommands.feedRollers(bottomIndexer, topIndexer, conveyor).repeatedly());
		NamedCommands.registerCommand("IdleRollers", ShooterCommands.idleRollers(bottomIndexer, topIndexer, conveyor));
		NamedCommands.registerCommand("Agitate", ShooterCommands.agitateIntake(bottomIndexer, topIndexer));
		NamedCommands.registerCommand("SpinIntake", IntakeCommands.spinIntake(intakeRoller));
		NamedCommands.registerCommand("AutoSpinUp", ShooterCommands.hubPreset(shooterFlywheel, hood).withTimeout(2));
		NamedCommands.registerCommand("Fender", ShooterCommands.hubPreset(shooterFlywheel, hood).withTimeout(2));
		// NamedCommands.registerCommand("AutoAim",
		// DriveCommands.autoAimToHub(drivetrain,
		// DrivetrainConstants.MAX_SPEED).withTimeout(2));
		NamedCommands
				.registerCommand(
						"AutoAim", Commands
								.parallel(DriveCommands.autoAimToHub(drivetrain, DrivetrainConstants.MAX_SPEED),
										ShooterCalculator.calculateAndShoot(drivetrain, hood, shooterFlywheel))
								.withTimeout(2));

	}

	private Command createLeftBlueBumpShootAuto() {
		String trajectoryName = ChoreoTraj.LeftBlueBump.name();
		return Commands.sequence(choreoAutoFactory.resetOdometry(trajectoryName),
				Commands.runOnce(
						() -> choreoController.reset(drivetrain.getState().Pose, drivetrain.getState().Speeds)),
				choreoAutoFactory.trajectoryCmd(trajectoryName), Commands.runOnce(drivetrain::stop, drivetrain),
				createTimedHubShot()).withName("ChoreoLeftBlueBumpShoot");
	}

	private Command createTimedHubShot() {
		return Commands.sequence(
				Commands.deadline(Commands.waitSeconds(1.0), ShooterCommands.hubPreset(shooterFlywheel, hood)),
				Commands.deadline(Commands.waitSeconds(1.0), ShooterCommands.hubPreset(shooterFlywheel, hood),
						ShooterCommands.feedRollers(bottomIndexer, topIndexer, conveyor)),
				Commands.parallel(ShooterCommands.idleRollers(bottomIndexer, topIndexer, conveyor).withTimeout(0.02),
						ShooterCommands.shooterIdle(shooterFlywheel, hood).withTimeout(0.02)));
	}

	private void followChoreoSample(SwerveSample sample) {
		PathPlannerTrajectoryState targetState = new PathPlannerTrajectoryState();
		targetState.timeSeconds = sample.t;
		targetState.pose = sample.getPose();
		targetState.fieldSpeeds = sample.getChassisSpeeds();
		targetState.linearVelocity = Math.hypot(sample.vx, sample.vy);
		targetState.heading = targetState.linearVelocity > 1e-6
				? new Rotation2d(sample.vx, sample.vy)
				: targetState.pose.getRotation();
		targetState.feedforwards = new DriveFeedforwards(new double[4], new double[4], new double[4],
				sample.moduleForcesX(), sample.moduleForcesY());

		drivetrain.setControl(choreoDrive.withSpeeds(ChassisSpeeds.discretize(
				choreoController.calculateRobotRelativeSpeeds(drivetrain.getState().Pose, targetState),
				AUTO_LOOP_PERIOD_SECONDS)));
	}

	public void configureAuto() {
		autoChooser.addDefaultOption("Fender I HARDLY KNOW HER -JAVI", Commands
				.sequence(ShooterCommands.hubPreset(shooterFlywheel, hood), Commands.waitSeconds(15),
						ShooterCommands.feedRollers(bottomIndexer, topIndexer, conveyor).repeatedly())
				.alongWith(
						drivetrain.applyRequest(() -> drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0))));

		autoChooser.addOption("Choreo LeftBlueBump + Shoot", createLeftBlueBumpShootAuto());
		autoChooser.addOption("You better hit the A stop before this -Manny (none)", Commands.none());
	}

	private void configureSysIdChooser() {
		sysIdChooser.addDefaultOption("Disabled", null);
		sysIdChooser.addOption("Translation Quasistatic Forward", drivetrain.sysIdQuasistatic(
				CommandSwerveDrivetrain.SysIdRoutineType.TRANSLATION, SysIdRoutine.Direction.kForward));
		sysIdChooser.addOption("Translation Quasistatic Reverse", drivetrain.sysIdQuasistatic(
				CommandSwerveDrivetrain.SysIdRoutineType.TRANSLATION, SysIdRoutine.Direction.kReverse));
		sysIdChooser.addOption("Translation Dynamic Forward", drivetrain
				.sysIdDynamic(CommandSwerveDrivetrain.SysIdRoutineType.TRANSLATION, SysIdRoutine.Direction.kForward));
		sysIdChooser.addOption("Translation Dynamic Reverse", drivetrain
				.sysIdDynamic(CommandSwerveDrivetrain.SysIdRoutineType.TRANSLATION, SysIdRoutine.Direction.kReverse));
		sysIdChooser.addOption("Steer Quasistatic Forward", drivetrain
				.sysIdQuasistatic(CommandSwerveDrivetrain.SysIdRoutineType.STEER, SysIdRoutine.Direction.kForward));
		sysIdChooser.addOption("Steer Quasistatic Reverse", drivetrain
				.sysIdQuasistatic(CommandSwerveDrivetrain.SysIdRoutineType.STEER, SysIdRoutine.Direction.kReverse));
		sysIdChooser.addOption("Steer Dynamic Forward", drivetrain
				.sysIdDynamic(CommandSwerveDrivetrain.SysIdRoutineType.STEER, SysIdRoutine.Direction.kForward));
		sysIdChooser.addOption("Steer Dynamic Reverse", drivetrain
				.sysIdDynamic(CommandSwerveDrivetrain.SysIdRoutineType.STEER, SysIdRoutine.Direction.kReverse));
		sysIdChooser.addOption("Rotation Quasistatic Forward", drivetrain
				.sysIdQuasistatic(CommandSwerveDrivetrain.SysIdRoutineType.ROTATION, SysIdRoutine.Direction.kForward));
		sysIdChooser.addOption("Rotation Quasistatic Reverse", drivetrain
				.sysIdQuasistatic(CommandSwerveDrivetrain.SysIdRoutineType.ROTATION, SysIdRoutine.Direction.kReverse));
		sysIdChooser.addOption("Rotation Dynamic Forward", drivetrain
				.sysIdDynamic(CommandSwerveDrivetrain.SysIdRoutineType.ROTATION, SysIdRoutine.Direction.kForward));
		sysIdChooser.addOption("Rotation Dynamic Reverse", drivetrain
				.sysIdDynamic(CommandSwerveDrivetrain.SysIdRoutineType.ROTATION, SysIdRoutine.Direction.kReverse));
		addFlywheelSysIdOptions("ShooterFlywheel", shooterFlywheel);
		addFlywheelSysIdOptions("TopIndexer", topIndexer);
		addFlywheelSysIdOptions("BottomIndexer", bottomIndexer);
		addFlywheelSysIdOptions("Conveyor", conveyor);
		addFlywheelSysIdOptions("IntakeRoller", intakeRoller);
		addPositionJointSysIdOptions("IntakeRack", intakeRack);
		addPositionJointSysIdOptions("Hood", hood);
	}

	private void addFlywheelSysIdOptions(String label, Flywheel flywheel) {
		sysIdChooser.addOption(label + " Quasistatic Forward",
				FlywheelSysIdCommands.quasistatic(flywheel, SysIdRoutine.Direction.kForward));
		sysIdChooser.addOption(label + " Quasistatic Reverse",
				FlywheelSysIdCommands.quasistatic(flywheel, SysIdRoutine.Direction.kReverse));
		sysIdChooser.addOption(label + " Dynamic Forward",
				FlywheelSysIdCommands.dynamic(flywheel, SysIdRoutine.Direction.kForward));
		sysIdChooser.addOption(label + " Dynamic Reverse",
				FlywheelSysIdCommands.dynamic(flywheel, SysIdRoutine.Direction.kReverse));
	}

	private void addPositionJointSysIdOptions(String label, PositionJoint positionJoint) {
		sysIdChooser.addOption(label + " Quasistatic Forward",
				PositionJointSysIdCommands.quasistatic(positionJoint, SysIdRoutine.Direction.kForward));
		sysIdChooser.addOption(label + " Quasistatic Reverse",
				PositionJointSysIdCommands.quasistatic(positionJoint, SysIdRoutine.Direction.kReverse));
		sysIdChooser.addOption(label + " Dynamic Forward",
				PositionJointSysIdCommands.dynamic(positionJoint, SysIdRoutine.Direction.kForward));
		sysIdChooser.addOption(label + " Dynamic Reverse",
				PositionJointSysIdCommands.dynamic(positionJoint, SysIdRoutine.Direction.kReverse));
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		if (Constants.ENABLE_SYSID_AUTOS) {
			Command sysIdCommand = sysIdChooser.get();
			if (sysIdCommand != null) {
				return sysIdCommand;
			}
		}
		Command autoCommand = autoChooser.get();
		return autoCommand != null ? autoCommand : Commands.none();
	}

	/** Logs robot and component transforms for the custom Robot_Remy asset. */
	public void updateVisualization() {
		if (Constants.currentMode != Constants.Mode.REAL) {
			simulation.visualizationPeriodic();
		}
	}

	/** Runs simulation-specific autonomous setup if simulation is active. */
	public void simulationAutonomousInit(Command autonomousCommand) {
		if (Constants.currentMode != Constants.Mode.REAL) {
			simulation.autonomousInit(autonomousCommand);
		}
	}

	/**
	 * Runs simulation periodic updates and telemetry if simulation is active.
	 */
	public void simulationPeriodic() {
		if (Constants.currentMode != Constants.Mode.REAL) {
			simulation.simulationPeriodic();
		}
	}
}
