// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShootCommands;
import frc.robot.commands.ShootCommands.SHOOTER_VOLTS;
import frc.robot.commands.flywheel.FlywheelVoltageCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Module;
import frc.robot.subsystems.drive.azimuth_motor.AzimuthMotorConstants;
import frc.robot.subsystems.drive.azimuth_motor.AzimuthMotorIOReplay;
import frc.robot.subsystems.drive.azimuth_motor.AzimuthMotorIOSim;
import frc.robot.subsystems.drive.azimuth_motor.AzimuthMotorIOTalonFX;
import frc.robot.subsystems.drive.drive_motor.DriveMotorConstants;
import frc.robot.subsystems.drive.drive_motor.DriveMotorIOReplay;
import frc.robot.subsystems.drive.drive_motor.DriveMotorIOSim;
import frc.robot.subsystems.drive.drive_motor.DriveMotorIOTalonFX;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIOPigeon2;
import frc.robot.subsystems.drive.odometry_threads.PhoenixOdometryThread;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelConstants;
import frc.robot.subsystems.flywheel.FlywheelIOReplay;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOSparkMax;
import frc.robot.subsystems.flywheel.FlywheelIOTalonFX;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Flywheel shooterFlywheel;
  private final Flywheel topIndexer;
  private final Flywheel bottomIndexer;
  private final Flywheel conveyor;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations

        // If using REV hardware, set up the Spark Odometry Thread, if using CTRE hardware, set up
        // the Phoenix Odometry Thread, if using a combination of the two, set up both
        drive =
            new Drive(
                new GyroIOPigeon2(13, DriveMotorConstants.canBusName),
                new Module(
                    new DriveMotorIOTalonFX(
                        "FrontLeftDrive", DriveMotorConstants.FRONT_LEFT_CONFIG),
                    new AzimuthMotorIOTalonFX(
                        "FrontLeftSteer", AzimuthMotorConstants.FRONT_LEFT_CONFIG)),
                new Module(
                    new DriveMotorIOTalonFX(
                        "FrontRightDrive", DriveMotorConstants.FRONT_RIGHT_CONFIG),
                    new AzimuthMotorIOTalonFX(
                        "FrontRightSteer", AzimuthMotorConstants.FRONT_RIGHT_CONFIG)),
                new Module(
                    new DriveMotorIOTalonFX("BackLeftDrive", DriveMotorConstants.BACK_LEFT_CONFIG),
                    new AzimuthMotorIOTalonFX(
                        "BackLeftSteer", AzimuthMotorConstants.BACK_LEFT_CONFIG)),
                new Module(
                    new DriveMotorIOTalonFX(
                        "BackRightDrive", DriveMotorConstants.BACK_RIGHT_CONFIG),
                    new AzimuthMotorIOTalonFX(
                        "BackRightSteer", AzimuthMotorConstants.BACK_RIGHT_CONFIG)),
                DriveMotorConstants.EXAMPLE_GAINS,
                AzimuthMotorConstants.EXAMPLE_GAINS,
                PhoenixOdometryThread.getInstance(),
                null);

        topIndexer = new Flywheel(new FlywheelIOSparkMax("TopIndexer", 
            FlywheelConstants.TOP_INDEXER_ROLLER_CONFIG), 
            FlywheelConstants.INDEXER_ROLLER_GAINS);

        bottomIndexer = new Flywheel(new FlywheelIOSparkMax("BottomIndexer", 
            FlywheelConstants.BOTTOM_INDEXER_ROLLER_CONFIG), 
            FlywheelConstants.INDEXER_ROLLER_GAINS);

        conveyor = new Flywheel(new FlywheelIOSparkMax("Conveyor", 
            FlywheelConstants.CONVEYOR_CONFIG), 
            FlywheelConstants.CONVEYOR_GAINS);

        shooterFlywheel = new Flywheel(new FlywheelIOTalonFX("ShooterFlywheel",
            FlywheelConstants.FLYWHEEL_ROLLER_CONFIG),
            FlywheelConstants.FLYWHEEL_ROLLER_GAINS);

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new Module(
                    new DriveMotorIOSim("FrontLeftDrive", DriveMotorConstants.FRONT_LEFT_CONFIG),
                    new AzimuthMotorIOSim("FrontLeftAz", AzimuthMotorConstants.FRONT_LEFT_CONFIG)),
                new Module(
                    new DriveMotorIOSim("FrontRightDrive", DriveMotorConstants.FRONT_RIGHT_CONFIG),
                    new AzimuthMotorIOSim(
                        "FrontRightAz", AzimuthMotorConstants.FRONT_RIGHT_CONFIG)),
                new Module(
                    new DriveMotorIOSim("BackLeftDrive", DriveMotorConstants.BACK_LEFT_CONFIG),
                    new AzimuthMotorIOSim("BackLeftAz", AzimuthMotorConstants.BACK_LEFT_CONFIG)),
                new Module(
                    new DriveMotorIOSim("BackRightDrive", DriveMotorConstants.BACK_RIGHT_CONFIG),
                    new AzimuthMotorIOSim("BackRightAz", AzimuthMotorConstants.BACK_RIGHT_CONFIG)),
                DriveMotorConstants.EXAMPLE_GAINS_SIM,
                AzimuthMotorConstants.EXAMPLE_GAINS_SIM,
                null,
                null);

        topIndexer = new Flywheel(new FlywheelIOSim("TopIndexer", 
            FlywheelConstants.TOP_INDEXER_ROLLER_CONFIG), 
            FlywheelConstants.INDEXER_ROLLER_GAINS);

        bottomIndexer = new Flywheel(new FlywheelIOSim("BottomIndexer", 
            FlywheelConstants.BOTTOM_INDEXER_ROLLER_CONFIG), 
            FlywheelConstants.INDEXER_ROLLER_GAINS);

        conveyor = new Flywheel(new FlywheelIOSim("Conveyor", 
            FlywheelConstants.CONVEYOR_CONFIG), 
            FlywheelConstants.CONVEYOR_GAINS);

        shooterFlywheel = new Flywheel(new FlywheelIOSim("ShooterFlywheel",
            FlywheelConstants.FLYWHEEL_ROLLER_CONFIG),
            FlywheelConstants.FLYWHEEL_ROLLER_GAINS);
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new Module(
                    new DriveMotorIOReplay("FrontLeftDrive"),
                    new AzimuthMotorIOReplay("FrontLeftAz")),
                new Module(
                    new DriveMotorIOReplay("FrontRightDrive"),
                    new AzimuthMotorIOReplay("FrontRightAz")),
                new Module(
                    new DriveMotorIOReplay("BackLeftDrive"),
                    new AzimuthMotorIOReplay("BackLeftAz")),
                new Module(
                    new DriveMotorIOReplay("BackRightDrive"),
                    new AzimuthMotorIOReplay("BackRightAz")),
                null,
                null,
                null,
                null);

        topIndexer = new Flywheel(new FlywheelIOReplay("TopIndexer"), 
            FlywheelConstants.INDEXER_ROLLER_GAINS);

        bottomIndexer = new Flywheel(new FlywheelIOReplay("BottomIndexer"), 
            FlywheelConstants.INDEXER_ROLLER_GAINS);

        conveyor = new Flywheel(new FlywheelIOReplay("Conveyor"), 
            FlywheelConstants.CONVEYOR_GAINS);

        shooterFlywheel = new Flywheel(new FlywheelIOReplay("ShooterFlywheel"),
            FlywheelConstants.FLYWHEEL_ROLLER_GAINS);
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // // Set up SysId routines
    /*
        autoChooser.addOption(
            "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption(
            "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption(
            "Drive SysId (Quasistatic Forward)",
            drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
            "Drive SysId (Quasistatic Reverse)",
            drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
            "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
            "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    */

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // // Lock to 0° when A button is held
    controller
        .x()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero));

    // // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    controller
        .a()
        .whileTrue(
            new FlywheelVoltageCommand(shooterFlywheel, SHOOTER_VOLTS.SHOOT)
        ).whileFalse(
            new FlywheelVoltageCommand(shooterFlywheel, SHOOTER_VOLTS.SLOW)
        );

    controller
    .rightBumper()
    .whileTrue(
        ShootCommands.feedRollers(bottomIndexer, topIndexer, conveyor)
    ).whileFalse(
        ShootCommands.idleRollers(bottomIndexer, topIndexer, conveyor)
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
