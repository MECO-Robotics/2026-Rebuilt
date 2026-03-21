package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.shooter.ShooterCommands;
import frc.robot.constants.drive.TunerConstants;
import frc.robot.constants.subsystems.IntakeConstants;
import frc.robot.constants.subsystems.ShooterConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.position_joint.PositionJoint;
import frc.robot.subsystems.position_joint.PositionJointIO;
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
	private double MaxAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond); // 3/4 of a rotation per second
	// Origionally 1.5

	// Subsystems
	public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
	private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDeadband(MaxSpeed * 0.05)
			.withRotationalDeadband(MaxAngularRate * 0.05) // Add a 5% deadband
			.withDriveRequestType(DriveRequestType.Velocity); // Use closed-loop control for drive motors

	private final Flywheel shooterFlywheel;
	private final Flywheel topIndexer;
	private final Flywheel bottomIndexer;
	private final Flywheel conveyor;
	private final Flywheel intakeRoller;
	private final PositionJoint intakeRack;
	private final PositionJoint hood;
	// private final RobotSimulation simulation;

	// Controller
	private final CommandXboxController controller = new CommandXboxController(0);
	private final CommandXboxController coPilot = new CommandXboxController(1);

	// Dashboard inputs
	private final LoggedDashboardChooser<Command> autoChooser;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
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

		// Keep PathPlanner's built-in chooser behavior (default option is "None").
		autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
		configureAuto();

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
				drivetrain.applyRequest(() -> drive.withVelocityX(controller.getLeftY() * MaxSpeed) // Drive forward
																									// with negative
																									// Y (forward)
						.withVelocityY(controller.getLeftX() * MaxSpeed) // Drive left with negative X (left)
						.withRotationalRate(-controller.getRightX() * MaxAngularRate) // Drive counterclockwise with
																						// negative X (left)
				));

		intakeRack.setDefaultCommand(PositionJoint.setVelocity(intakeRack, () -> 0.0));
		intakeRoller.setDefaultCommand(Flywheel.setVelocity(intakeRoller, () -> 0.0));
		// Feed shooter
		controller.rightBumper()
				.whileTrue(ShooterCommands.agitateIntake(bottomIndexer, topIndexer)
						.alongWith(IntakeCommands.stowIntake(intakeRack, intakeRoller, conveyor)))
				.whileFalse(ShooterCommands.idleRollers(bottomIndexer, topIndexer, conveyor)
						.alongWith(IntakeCommands.deployIntake(intakeRack, intakeRoller)));

		// Run intake
		controller.leftBumper().whileTrue(IntakeCommands.spinIntake(intakeRoller))
				.whileFalse(IntakeCommands.idleIntake(intakeRoller));

		// Deploy and stow intake
		controller.povUp().whileTrue(IntakeCommands.deployIntake(intakeRack, intakeRoller));

		coPilot.povUp().whileTrue(IntakeCommands.deployIntakeVelocity(intakeRack, intakeRoller));

		controller.povDown().whileTrue(IntakeCommands.stowIntake(intakeRack, intakeRoller, conveyor))
				.onFalse(IntakeCommands.idle(intakeRack, intakeRoller, conveyor));

		coPilot.povDown().whileTrue(IntakeCommands.stowIntakeVelocity(intakeRack, intakeRoller, conveyor))
				.onFalse(IntakeCommands.idle(intakeRack, intakeRoller, conveyor));

		// Shooter presets
		controller.b().or(coPilot.b()).whileTrue(ShooterCommands.shooterIdle(shooterFlywheel, hood));

		controller.x().or(coPilot.x()).whileTrue(ShooterCommands.hubPreset(shooterFlywheel, hood));

		controller.y().or(coPilot.y()).whileTrue(ShooterCommands.ferryPreset(shooterFlywheel, hood));

		controller.start().onTrue(DriveCommands.resetHeading(drivetrain));
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

	public void configureAuto() {
		autoChooser.addDefaultOption("Fender I HARDLY KNOW HER -JAVI", Commands
				.sequence(ShooterCommands.hubPreset(shooterFlywheel, hood), Commands.waitSeconds(15),
						ShooterCommands.feedRollers(bottomIndexer, topIndexer, conveyor))
				.alongWith(
						drivetrain.applyRequest(() -> drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0))));

		autoChooser.addOption("You better hit the A stop before this -Manny (none)", Commands.none());
		
		NamedCommands.registerCommand("DeployIntake", IntakeCommands.deployIntake(intakeRack, intakeRoller));
		NamedCommands.registerCommand("StowIntake", IntakeCommands.stowIntake(intakeRack, intakeRoller, conveyor));
		NamedCommands.registerCommand("FeedRollers", ShooterCommands.feedRollers(bottomIndexer, topIndexer, conveyor));
		NamedCommands.registerCommand("IdleRollers", ShooterCommands.idleRollers(bottomIndexer, topIndexer, conveyor));
		NamedCommands.registerCommand("SpinIntake", IntakeCommands.spinIntake(intakeRoller));
		NamedCommands.registerCommand("Fender", ShooterCommands.hubPreset(shooterFlywheel, hood).withTimeout(2));
		NamedCommands.registerCommand("AutoAim", DriveCommands.autoAimToHub(drivetrain, MaxSpeed).withTimeout(2));
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// Command sysIdCommand = sysIdChooser.get();
		// if (sysIdCommand != null) {
		// return sysIdCommand;
		// }
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
