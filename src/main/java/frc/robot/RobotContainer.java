package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.constants.vision.VisionConstants.robotToArducam;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.flywheel.FlywheelSysIdCommands;
import frc.robot.commands.position_joint.PositionJointSysIdCommands;
import frc.robot.commands.shooter.ShooterCalculator;
import frc.robot.commands.shooter.ShooterCommands;
import frc.robot.constants.drive.TunerConstants;
import frc.robot.constants.subsystems.IntakeConstants;
import frc.robot.constants.subsystems.ShooterConstants;
import frc.robot.constants.vision.VisionConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.position_joint.PositionJoint;
import frc.robot.subsystems.position_joint.PositionJointIO;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
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
	private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
																						// speed
	private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
																						// max angular velocity

	// Subsystems
	public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
	private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDeadband(MaxSpeed * 0.1)
			.withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

	private final Flywheel shooterFlywheel;
	private final Flywheel topIndexer;
	private final Flywheel bottomIndexer;
	private final Flywheel conveyor;
	private final Flywheel intakeRoller;
	private final PositionJoint intakeRack;
	private final PositionJoint hood;
	private final Vision vision;
	// private final RobotSimulation simulation;

	// Controller
	private final CommandXboxController controller = new CommandXboxController(0);

	// Dashboard inputs
	private final LoggedDashboardChooser<Command> autoChooser;
	private final LoggedDashboardChooser<Command> sysIdChooser;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		topIndexer = new Flywheel(FlywheelIO.fromSparkMax("TopIndexer", ShooterConstants.TOP_INDEXER_ROLLER_CONFIG),
				ShooterConstants.INDEXER_ROLLER_GAINS);

		bottomIndexer = new Flywheel(
				FlywheelIO.fromSparkMax("BottomIndexer", ShooterConstants.BOTTOM_INDEXER_ROLLER_CONFIG),
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

		vision = new Vision(drivetrain::addVisionMeasurement, VisionIO.questNavWithPhoton(VisionConstants.arducamName,
				VisionConstants.robotToQuest, robotToArducam, () -> drivetrain.getState().Pose));
		// simulation = RobotSimulation.create(drive, intakeRack, hood,
		// shooterFlywheel);
		// simulation.bindCommandHooks();

		configureAuto();
		// Keep PathPlanner's built-in chooser behavior (default option is "None").
		autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

		sysIdChooser = new LoggedDashboardChooser<>("SysId Choices");
		configureSysIdChooser();

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
		// Default command, normal field-relative drive
		drivetrain.setDefaultCommand(
				// Drivetrain will execute this command periodically
				drivetrain.applyRequest(() -> drive.withVelocityX(-controller.getLeftY() * MaxSpeed) // Drive forward
																										// with negative
																										// Y (forward)
						.withVelocityY(-controller.getLeftX() * MaxSpeed) // Drive left with negative X (left)
						.withRotationalRate(controller.getRightX() * MaxAngularRate) // Drive counterclockwise with
																						// negative X (left)
				));

		// Lock to 0Â° when A button is held
		// controller
		// .x()
		// .whileTrue(
		// DriveCommands.joystickDriveAtAngle(
		// drive,
		// () -> -controller.getLeftY(),
		// () -> -controller.getLeftX(),
		// () -> Rotation2d.kZero));

		// Auto-aim to hub when Y button is held
		controller.y()
				.whileTrue(DriveCommands
						.joystickAimToHub(drivetrain, () -> -controller.getLeftY(), () -> -controller.getLeftX(),
								MaxSpeed)
						.alongWith(ShooterCalculator.calculateAndShoot(drivetrain, hood, shooterFlywheel)))
				.whileFalse(Flywheel.setVoltage(shooterFlywheel, () -> 0)
						.alongWith(PositionJoint.setPosition(hood, () -> 0)));
		// .whileFalse(ShooterCommands.stopShooting(shooterFlywheel, hood));

		// * INTAKE BINDS */
		controller.rightBumper().whileTrue(ShooterCommands.feedRollers(bottomIndexer, topIndexer, conveyor))
				.whileFalse(ShooterCommands.idleRollers(bottomIndexer, topIndexer, conveyor));

		controller.leftBumper().whileTrue(Commands.run(() -> intakeRoller.setVoltage(10), intakeRoller))
				.whileFalse(Commands.run(() -> intakeRoller.setVoltage(0), intakeRoller));

		controller.povUp().whileTrue(IntakeCommands.deployIntake(intakeRack, intakeRoller));
		controller.povDown().whileTrue(IntakeCommands.stowIntake(intakeRack, intakeRoller, conveyor))
				.whileFalse(ShooterCommands.idleRollers(bottomIndexer, topIndexer, conveyor));
		// controller.povRight().whileTrue(IntakeCommands.feedIntake(intakeRack,
		// intakeRoller));
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

		// Controller disconnected alerts
		// primaryDisconnected.set(!DriverStation.isJoystickConnected(primary.getHID().getPort()));
		// secondaryDisconnected.set(!DriverStation.isJoystickConnected(secondary.getHID().getPort()));
		// overrideDisconnected.set(!overrides.isConnected());
	}

	public void configureAuto() {
		NamedCommands.registerCommand("DeployIntake", IntakeCommands.deployIntake(intakeRack, intakeRoller));
		NamedCommands.registerCommand("StowIntake", IntakeCommands.stowIntake(intakeRack, intakeRoller, conveyor));
		NamedCommands.registerCommand("AgitateIntake", IntakeCommands.agitateIntake(intakeRack, intakeRoller));
		NamedCommands.registerCommand("FeedRollers", ShooterCommands.feedRollers(bottomIndexer, topIndexer, conveyor));
		NamedCommands.registerCommand("IdleRollers", ShooterCommands.idleRollers(bottomIndexer, topIndexer, conveyor));
		// NamedCommands.registerCommand("Flywheel",
		// ShooterCalculator.calculateAndShoot(drive, hood, shooterFlywheel));

		// NamedCommands.registerCommand("AutoSpinUp",
		// ShooterCalculator.calculateAndShoot(drive, hood, shooterFlywheel));

		NamedCommands.registerCommand("AutoAim", DriveCommands.autoAimToHub(drivetrain, MaxSpeed).withTimeout(2));
	}

	public void configureSysIdChooser() {
		sysIdChooser.addDefaultOption("None", null);

		// sysIdChooser.addOption("Drive Quasistatic Forward",
		// DriveSysIdCommands.driveQuasistatic(drive, Direction.kForward));
		// sysIdChooser.addOption("Drive Quasistatic Reverse",
		// DriveSysIdCommands.driveQuasistatic(drive, Direction.kReverse));
		// sysIdChooser.addOption("Drive Dynamic Forward",
		// DriveSysIdCommands.driveDynamic(drive, Direction.kForward));
		// sysIdChooser.addOption("Drive Dynamic Reverse",
		// DriveSysIdCommands.driveDynamic(drive, Direction.kReverse));
		// sysIdChooser.addOption("Azimuth Quasistatic Forward",
		// DriveSysIdCommands.azimuthQuasistatic(drive, Direction.kForward));
		// sysIdChooser.addOption("Azimuth Quasistatic Reverse",
		// DriveSysIdCommands.azimuthQuasistatic(drive, Direction.kReverse));
		// sysIdChooser.addOption("Azimuth Dynamic Forward",
		// DriveSysIdCommands.azimuthDynamic(drive, Direction.kForward));
		// sysIdChooser.addOption("Azimuth Dynamic Reverse",
		// DriveSysIdCommands.azimuthDynamic(drive, Direction.kReverse));

		sysIdChooser.addOption("Shooter Flywheel Quasistatic Forward",
				FlywheelSysIdCommands.quasistatic(shooterFlywheel, Direction.kForward));
		sysIdChooser.addOption("Shooter Flywheel Quasistatic Reverse",
				FlywheelSysIdCommands.quasistatic(shooterFlywheel, Direction.kReverse));
		sysIdChooser.addOption("Shooter Flywheel Dynamic Forward",
				FlywheelSysIdCommands.dynamic(shooterFlywheel, Direction.kForward));
		sysIdChooser.addOption("Shooter Flywheel Dynamic Reverse",
				FlywheelSysIdCommands.dynamic(shooterFlywheel, Direction.kReverse));

		sysIdChooser.addOption("Top Indexer Quasistatic Forward",
				FlywheelSysIdCommands.quasistatic(topIndexer, Direction.kForward));
		sysIdChooser.addOption("Top Indexer Quasistatic Reverse",
				FlywheelSysIdCommands.quasistatic(topIndexer, Direction.kReverse));
		sysIdChooser.addOption("Top Indexer Dynamic Forward",
				FlywheelSysIdCommands.dynamic(topIndexer, Direction.kForward));
		sysIdChooser.addOption("Top Indexer Dynamic Reverse",
				FlywheelSysIdCommands.dynamic(topIndexer, Direction.kReverse));

		sysIdChooser.addOption("Bottom Indexer Quasistatic Forward",
				FlywheelSysIdCommands.quasistatic(bottomIndexer, Direction.kForward));
		sysIdChooser.addOption("Bottom Indexer Quasistatic Reverse",
				FlywheelSysIdCommands.quasistatic(bottomIndexer, Direction.kReverse));
		sysIdChooser.addOption("Bottom Indexer Dynamic Forward",
				FlywheelSysIdCommands.dynamic(bottomIndexer, Direction.kForward));
		sysIdChooser.addOption("Bottom Indexer Dynamic Reverse",
				FlywheelSysIdCommands.dynamic(bottomIndexer, Direction.kReverse));

		sysIdChooser.addOption("Conveyor Quasistatic Forward",
				FlywheelSysIdCommands.quasistatic(conveyor, Direction.kForward));
		sysIdChooser.addOption("Conveyor Quasistatic Reverse",
				FlywheelSysIdCommands.quasistatic(conveyor, Direction.kReverse));
		sysIdChooser.addOption("Conveyor Dynamic Forward", FlywheelSysIdCommands.dynamic(conveyor, Direction.kForward));
		sysIdChooser.addOption("Conveyor Dynamic Reverse", FlywheelSysIdCommands.dynamic(conveyor, Direction.kReverse));

		sysIdChooser.addOption("Intake Roller Quasistatic Forward",
				FlywheelSysIdCommands.quasistatic(intakeRoller, Direction.kForward));
		sysIdChooser.addOption("Intake Roller Quasistatic Reverse",
				FlywheelSysIdCommands.quasistatic(intakeRoller, Direction.kReverse));
		sysIdChooser.addOption("Intake Roller Dynamic Forward",
				FlywheelSysIdCommands.dynamic(intakeRoller, Direction.kForward));
		sysIdChooser.addOption("Intake Roller Dynamic Reverse",
				FlywheelSysIdCommands.dynamic(intakeRoller, Direction.kReverse));

		sysIdChooser.addOption("Intake Rack Quasistatic Forward",
				PositionJointSysIdCommands.quasistatic(intakeRack, Direction.kForward));
		sysIdChooser.addOption("Intake Rack Quasistatic Reverse",
				PositionJointSysIdCommands.quasistatic(intakeRack, Direction.kReverse));
		sysIdChooser.addOption("Intake Rack Dynamic Forward",
				PositionJointSysIdCommands.dynamic(intakeRack, Direction.kForward));
		sysIdChooser.addOption("Intake Rack Dynamic Reverse",
				PositionJointSysIdCommands.dynamic(intakeRack, Direction.kReverse));

		sysIdChooser.addOption("Hood Quasistatic Forward",
				PositionJointSysIdCommands.quasistatic(hood, Direction.kForward));
		sysIdChooser.addOption("Hood Quasistatic Reverse",
				PositionJointSysIdCommands.quasistatic(hood, Direction.kReverse));
		sysIdChooser.addOption("Hood Dynamic Forward", PositionJointSysIdCommands.dynamic(hood, Direction.kForward));
		sysIdChooser.addOption("Hood Dynamic Reverse", PositionJointSysIdCommands.dynamic(hood, Direction.kReverse));
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		Command sysIdCommand = sysIdChooser.get();
		if (sysIdCommand != null) {
			return sysIdCommand;
		}
		Command autoCommand = autoChooser.get();
		return autoCommand != null ? autoCommand : Commands.none();
	}

	/** Logs robot and component transforms for the custom Robot_Remy asset. */
	public void updateVisualization() {
		// simulation.visualizationPeriodic();
	}

	/** Runs simulation-specific autonomous setup if simulation is active. */
	public void simulationAutonomousInit(Command autonomousCommand) {
		// simulation.autonomousInit(autonomousCommand);
	}

	/**
	 * Runs simulation periodic updates and telemetry if simulation is active.
	 */
	public void simulationPeriodic() {
		// simulation.simulationPeriodic();
	}
}
