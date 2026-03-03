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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.flywheel.FlywheelVoltageCommand;
import frc.robot.commands.shooter.ShooterCalculator;
import frc.robot.commands.shooter.ShooterCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.azimuth_motor.AzimuthMotorConstants;
import frc.robot.subsystems.drive.azimuth_motor.AzimuthMotorIO;
import frc.robot.subsystems.drive.drive_motor.DriveMotorConstants;
import frc.robot.subsystems.drive.drive_motor.DriveMotorIO;
import frc.robot.subsystems.drive.gyro.GyroIOPigeon2;
import frc.robot.subsystems.drive.odometry_threads.PhoenixOdometryThread;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelConstants;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.position_joint.PositionJoint;
import frc.robot.subsystems.position_joint.PositionJointConstants;
import frc.robot.subsystems.position_joint.PositionJointIO;
import frc.robot.util.HubShiftUtil;
import frc.robot.util.visualization.RobotRemyVisualizer;
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
  private final Flywheel intakeRoller;
  private final PositionJoint intakeRack;
  private final PositionJoint hood;
  private final RobotRemyVisualizer robotRemyVisualizer;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    var driveGains = DriveMotorConstants.DRIVE_MOTOR_GAINS;
    var azimuthGains = AzimuthMotorConstants.AZIMUTH_MOTOR_GAINS;
    drive =
        Drive.fromModuleConfigs(
            () -> new GyroIOPigeon2(13, DriveMotorConstants.canBusName),
            DriveMotorIO::talonFXFactory,
            AzimuthMotorIO::talonFXFactory,
            DriveMotorConstants.FRONT_LEFT_CONFIG,
            AzimuthMotorConstants.FRONT_LEFT_CONFIG,
            DriveMotorConstants.FRONT_RIGHT_CONFIG,
            AzimuthMotorConstants.FRONT_RIGHT_CONFIG,
            DriveMotorConstants.BACK_LEFT_CONFIG,
            AzimuthMotorConstants.BACK_LEFT_CONFIG,
            DriveMotorConstants.BACK_RIGHT_CONFIG,
            AzimuthMotorConstants.BACK_RIGHT_CONFIG,
            driveGains,
            azimuthGains,
            Constants.currentMode == Constants.Mode.REAL
                ? PhoenixOdometryThread.getInstance()
                : null,
            null);

    topIndexer =
        new Flywheel(
            FlywheelIO.fromSparkMax("TopIndexer", FlywheelConstants.TOP_INDEXER_ROLLER_CONFIG),
            FlywheelConstants.INDEXER_ROLLER_GAINS);

    bottomIndexer =
        new Flywheel(
            FlywheelIO.fromSparkMax(
                "BottomIndexer", FlywheelConstants.BOTTOM_INDEXER_ROLLER_CONFIG),
            FlywheelConstants.INDEXER_ROLLER_GAINS);

    conveyor =
        new Flywheel(
            FlywheelIO.fromSparkMax("Conveyor", FlywheelConstants.CONVEYOR_CONFIG),
            FlywheelConstants.CONVEYOR_GAINS);

    shooterFlywheel =
        new Flywheel(
            FlywheelIO.fromTalonFX("ShooterFlywheel", FlywheelConstants.FLYWHEEL_ROLLER_CONFIG),
            FlywheelConstants.FLYWHEEL_ROLLER_GAINS);

    intakeRoller =
        new Flywheel(
            FlywheelIO.fromSparkMax("IntakeRoller", FlywheelConstants.INTAKE_ROLLER_CONFIG),
            FlywheelConstants.FLYWHEEL_ROLLER_GAINS);

    intakeRack =
        new PositionJoint(
            PositionJointIO.fromSparkMax("IntakeRack", PositionJointConstants.INTAKE_RACK_CONFIG),
            PositionJointConstants.INTAKE_RACK_GAINS);
    hood =
        new PositionJoint(
            PositionJointIO.fromSparkMax("Hood", PositionJointConstants.HOOD_CONFIG),
            PositionJointConstants.HOOD_GAINS);

    robotRemyVisualizer =
        new RobotRemyVisualizer(
            drive::getPose,
            hood::getPosition,
            shooterFlywheel::getPosition,
            intakeRack::getPosition,
            () -> 0);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // // Set up SysId routines
    /*
     * autoChooser.addOption(
     * "Drive Wheel Radius Characterization",
     * DriveCommands.wheelRadiusCharacterization(drive));
     * autoChooser.addOption(
     * "Drive Simple FF Characterization",
     * DriveCommands.feedforwardCharacterization(drive));
     * autoChooser.addOption(
     * "Drive SysId (Quasistatic Forward)",
     * drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
     * autoChooser.addOption(
     * "Drive SysId (Quasistatic Reverse)",
     * drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
     * autoChooser.addOption(
     * "Drive SysId (Dynamic Forward)",
     * drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
     * autoChooser.addOption(
     * "Drive SysId (Dynamic Reverse)",
     * drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
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

    controller
        .y()
        .whileTrue(
            DriveCommands.joystickAimToHub(
                    drive, () -> -controller.getLeftY(), () -> -controller.getLeftX())
                .alongWith(ShooterCalculator.calculateAndShoot(drive, hood, shooterFlywheel)));

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

    // controller
    // .a()
    // .whileTrue(new FlywheelVoltageCommand(shooterFlywheel, SHOOTER_VOLTS.SHOOT))
    // .whileFalse(new FlywheelVoltageCommand(shooterFlywheel, SHOOTER_VOLTS.SLOW));

    controller
        .a()
        .whileTrue(
            new FlywheelVoltageCommand(
                shooterFlywheel, () -> controller.getLeftTriggerAxis() * 12));

    controller
        .rightBumper()
        .whileTrue(ShooterCommands.feedRollers(bottomIndexer, topIndexer, conveyor))
        .whileFalse(ShooterCommands.idleRollers(bottomIndexer, topIndexer, conveyor));

    controller
        .leftBumper()
        .whileTrue(IntakeCommands.deployIntake(intakeRack, intakeRoller))
        .whileFalse(IntakeCommands.stowIntake(intakeRack, intakeRoller));

    controller
        .pov(0)
        .whileTrue(
            Commands.runEnd(
                () -> intakeRack.setVoltage(-6), () -> intakeRack.setVoltage(0), intakeRack));
    controller
        .pov(180)
        .whileTrue(
            Commands.runEnd(
                () -> intakeRack.setVoltage(6), () -> intakeRack.setVoltage(0), intakeRack));
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
    SmartDashboard.putString(
        "Shifts/Remaining Shift Time",
        String.format("%.1f", Math.max(shiftInfo.remainingTime(), 0.0)));
    SmartDashboard.putBoolean("Shifts/Shift Active", shiftInfo.active());
    SmartDashboard.putString(
        "Shifts/Game State",
        DriverStation.isAutonomous() ? "Autonomous" : shiftInfo.currentShift().toString());
    SmartDashboard.putBoolean(
        "Shifts/Active First?",
        DriverStation.getAlliance().orElse(Alliance.Blue) == HubShiftUtil.getFirstActiveAlliance());

    SmartDashboard.putString("Shifts/Match Time Color", shiftInfo.matchTimeColor());
    SmartDashboard.putString("Shifts/Shift Time Color", shiftInfo.shiftTimeColor());

    // Controller disconnected alerts
    // primaryDisconnected.set(!DriverStation.isJoystickConnected(primary.getHID().getPort()));
    // secondaryDisconnected.set(!DriverStation.isJoystickConnected(secondary.getHID().getPort()));
    // overrideDisconnected.set(!overrides.isConnected());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /** Logs robot and component transforms for the custom Robot_Remy asset. */
  public void updateVisualization() {
    robotRemyVisualizer.periodic();
  }
}
