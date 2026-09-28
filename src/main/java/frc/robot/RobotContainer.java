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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
		vision = new Vision(drivetrain::addVisionMeasurement, () -> drivetrain.getState().Pose,
				new VisionIOQuestNav(VisionConstants.robotToQuest,
						VisionIO.limelightMegaTag1WithMegaTag2SingleTagWithSim(VisionConstants.limelightName,
								() -> drivetrain.getState().Pose.getRotation(), VisionConstants.robotToLimelight,
								drivetrain::getPhysicsPose),
						() -> drivetrain.getState().Pose));
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

		// Configure the button bindings
		configureButtonBindings();
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
		intakeRoller.setDefaultCommand(Flywheel.setVoltage(intakeRoller, () -> 0.0));
		conveyor.setDefaultCommand(Flywheel.setVoltage(conveyor, () -> 0.0));
		topIndexer.setDefaultCommand(Flywheel.setVoltage(topIndexer, () -> 0.0));
		bottomIndexer.setDefaultCommand(Flywheel.setVoltage(bottomIndexer, () -> 0.0));
		shooterFlywheel.setDefaultCommand(Flywheel.holdVelocity(shooterFlywheel, () -> 0.0));
		hood.setDefaultCommand(PositionJoint.holdPosition(hood, ShooterConstants.HOOD_PRESET.STOW));

		// Deploy and run the complete acquisition path while held. On release, the
		// motors idle and the rack finishes at its deployed setpoint.
		controller.leftBumper()
				.whileTrue(IntakeCommands.acquire(intakeRack, intakeRoller, conveyor, bottomIndexer, topIndexer))
				.onFalse(IntakeCommands.deployRack(intakeRack));
		controller.leftTrigger().whileTrue(IntakeCommands.eject(intakeRoller, conveyor, bottomIndexer, topIndexer));

		// Deploy intake backup for intake skipping
		coPilot.povUp().whileTrue(IntakeCommands.deployIntakeVelocity(intakeRack, intakeRoller));
		// Stow intake backup for intake skipping
		coPilot.povDown().whileTrue(IntakeCommands.stowIntakeVelocity(intakeRack, intakeRoller, conveyor))
				.onFalse(IntakeCommands.idle(intakeRack, intakeRoller, conveyor));
		coPilot.povLeft().onTrue(IntakeCommands.deployRack(intakeRack));
		coPilot.povRight().onTrue(IntakeCommands.stowRack(intakeRack));

		// ************************** SHOOTER KEYBINDS **************************
		controller.a()
				.whileTrue(ShooterCommands.coordinatedHubShot(drivetrain, hood, shooterFlywheel, bottomIndexer,
						topIndexer, conveyor, () -> -controller.getLeftY(), () -> -controller.getLeftX(),
						this::isShotPoseReady, controller.rightBumper()::getAsBoolean,
						ready -> controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, ready ? 0.35 : 0.0)));

		controller.y().whileTrue(ShooterCommands.ferryPreset(shooterFlywheel, hood));

		// Shooter presets
		controller.b().or(coPilot.b()).whileTrue(ShooterCommands.shooterIdle(shooterFlywheel, hood));

		controller.x().or(coPilot.x()).whileTrue(ShooterCommands.hubPreset(shooterFlywheel, hood));

		coPilot.y().whileTrue(ShooterCommands.ferryPreset(shooterFlywheel, hood));

		coPilot.a().whileTrue(ShooterCommands.trenchPreset(shooterFlywheel, hood));
	}

	public void updateDashboardOutputs() {
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

		SmartDashboard.putString("Shifts/Match Time Color", shiftInfo.matchTimeColor());
		SmartDashboard.putString("Shifts/Shift Time Color", shiftInfo.shiftTimeColor());
	}

	private void registerNamedCommands() {
		NamedCommands.registerCommand("DeployIntake", IntakeCommands.deployRack(intakeRack));
		NamedCommands.registerCommand("StowIntake", IntakeCommands.stowRack(intakeRack));
		NamedCommands.registerCommand("FeedRollers", createAutonomousHubShot());
		NamedCommands.registerCommand("IdleRollers", ShooterCommands.idleRollers(bottomIndexer, topIndexer, conveyor));
		NamedCommands.registerCommand("Agitate", ShooterCommands.agitateIntake(bottomIndexer, topIndexer));
		NamedCommands.registerCommand("SpinIntake", IntakeCommands.spinIntake(intakeRoller, conveyor));
		NamedCommands.registerCommand("AutoSpinUp",
				ShooterCalculator.calculateAndShoot(drivetrain, hood, shooterFlywheel).withTimeout(2));
		NamedCommands.registerCommand("Fender", ShooterCommands.hubPreset(shooterFlywheel, hood).withTimeout(2));
		NamedCommands.registerCommand("AutoAim",
				Commands.parallel(DriveCommands.autoAimToHub(drivetrain, DrivetrainConstants.MAX_SPEED),
						ShooterCalculator.calculateAndShoot(drivetrain, hood, shooterFlywheel)));
		NamedCommands.registerCommand("CoordinatedHubShot", createAutonomousHubShot());

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
		return createAutonomousHubShot();
	}

	private Command createAutonomousHubShot() {
		return ShooterCommands.autonomousHubShot(drivetrain, hood, shooterFlywheel, bottomIndexer, topIndexer, conveyor,
				this::isShotPoseReady);
	}

	private boolean isShotPoseReady() {
		// PathPlanner/Choreo explicitly seed autonomous odometry. Teleop requires a
		// field-aligned vision source, with A+right-bumper available as an override.
		return DriverStation.isAutonomousEnabled() || vision.isPoseReady();
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
		autoChooser.addDefaultOption("Fender I HARDLY KNOW HER -JAVI", createAutonomousHubShot());

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
