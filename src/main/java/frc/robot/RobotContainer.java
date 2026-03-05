// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.shooter.ShooterCalculator;
import frc.robot.commands.shooter.ShooterCommands;
import frc.robot.constants.Constants;
import frc.robot.constants.drive.AzimuthMotorConstants;
import frc.robot.constants.drive.DriveMotorConstants;
import frc.robot.constants.subsystems.IntakeConstants;
import frc.robot.constants.subsystems.ShooterConstants;
import frc.robot.simulation.Hopper;
import frc.robot.simulation.IntakeSim;
import frc.robot.simulation.LaunchedFuelSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.azimuth_motor.AzimuthMotorIO;
import frc.robot.subsystems.drive.drive_motor.DriveMotorIO;
import frc.robot.subsystems.drive.gyro.GyroIOPigeon2;
import frc.robot.subsystems.drive.odometry_threads.PhoenixOdometryThread;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.position_joint.PositionJoint;
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
  private final IntakeSim intakeSim;
  private final LaunchedFuelSim launchedFuelSim;
  private final Hopper hopper;

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
            FlywheelIO.fromSparkMax("TopIndexer", ShooterConstants.TOP_INDEXER_ROLLER_CONFIG),
            ShooterConstants.INDEXER_ROLLER_GAINS);

    bottomIndexer =
        new Flywheel(
            FlywheelIO.fromSparkMax("BottomIndexer", ShooterConstants.BOTTOM_INDEXER_ROLLER_CONFIG),
            ShooterConstants.INDEXER_ROLLER_GAINS);

    conveyor =
        new Flywheel(
            FlywheelIO.fromSparkMax("Conveyor", ShooterConstants.CONVEYOR_CONFIG),
            ShooterConstants.CONVEYOR_GAINS);

    shooterFlywheel =
        new Flywheel(
            FlywheelIO.fromTalonFX("ShooterFlywheel", ShooterConstants.FLYWHEEL_ROLLER_CONFIG),
            ShooterConstants.FLYWHEEL_ROLLER_GAINS);

    intakeRoller =
        new Flywheel(
            FlywheelIO.fromSparkMax("IntakeRoller", IntakeConstants.INTAKE_ROLLER_CONFIG),
            IntakeConstants.INTAKE_ROLLER_GAINS);

    intakeRack =
        new PositionJoint(
            PositionJointIO.fromSparkMax("IntakeRack", IntakeConstants.INTAKE_RACK_CONFIG),
            IntakeConstants.INTAKE_RACK_GAINS);
    hood =
        new PositionJoint(
            PositionJointIO.fromSparkMax("Hood", ShooterConstants.HOOD_CONFIG),
            ShooterConstants.HOOD_GAINS);

    if (drive.getSimulation() != null) {
      intakeSim = new IntakeSim(drive.getSimulation());
      launchedFuelSim = new LaunchedFuelSim(drive, intakeSim, hood, shooterFlywheel);
      hopper = new Hopper(intakeSim::getStoredFuelCount, intakeRack::getPosition, drive::getPose);
    } else {
      intakeSim = null;
      launchedFuelSim = null;
      hopper = null;
    }

    IntakeCommands.setIntakeSimulation(intakeSim);
    ShooterCommands.setLaunchedFuelSimulation(launchedFuelSim);

    robotRemyVisualizer =
        new RobotRemyVisualizer(
            drive::getPose,
            hood::getPosition,
            shooterFlywheel::getPosition,
            intakeRack::getPosition,
            () -> 0);

    configureAuto();
    // Keep PathPlanner's built-in chooser behavior (default option is "None").
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

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
            () -> controller.getLeftY(),
            () -> controller.getLeftX(),
            () -> controller.getRightX()));

    // Lock to 0° when A button is held
    controller
        .x()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero));

    // Auto-aim to hub when Y button is held
    controller
        .y()
        .whileTrue(
            DriveCommands.joystickAimToHub(
                    drive, () -> -controller.getLeftY(), () -> -controller.getLeftX())
                .alongWith(ShooterCalculator.calculateAndShoot(drive, hood, shooterFlywheel)));

    // * INTAKE BINDS */
    controller
        .rightBumper()
        .whileTrue(ShooterCommands.feedRollers(bottomIndexer, topIndexer, conveyor))
        .whileFalse(ShooterCommands.idleRollers(bottomIndexer, topIndexer, conveyor));

    controller.povUp().whileTrue(IntakeCommands.deployIntake(intakeRack, intakeRoller));
    controller.povDown().whileTrue(IntakeCommands.stowIntake(intakeRack, intakeRoller));
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

  public void configureAuto() {
    NamedCommands.registerCommand(
        "DeployIntake", IntakeCommands.deployIntake(intakeRack, intakeRoller));
    NamedCommands.registerCommand(
        "StowIntake", IntakeCommands.stowIntake(intakeRack, intakeRoller));
    NamedCommands.registerCommand(
        "AgitateIntake", IntakeCommands.agitateIntake(intakeRack, intakeRoller));
    NamedCommands.registerCommand(
        "FeedRollers", ShooterCommands.feedRollers(bottomIndexer, topIndexer, conveyor));
    NamedCommands.registerCommand(
        "IdleRollers", ShooterCommands.idleRollers(bottomIndexer, topIndexer, conveyor));
    NamedCommands.registerCommand(
        "Flywheel", ShooterCalculator.calculateAndShoot(drive, hood, shooterFlywheel));

    NamedCommands.registerCommand(
        "AutoSpinUp", ShooterCalculator.calculateAndShoot(drive, hood, shooterFlywheel));

    NamedCommands.registerCommand("AutoAim", DriveCommands.autoAimToHub(drive).withTimeout(2));
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
    if (hopper != null) {
      hopper.periodic();
    }
    robotRemyVisualizer.periodic();
  }

  /** Returns field-relative poses of gamepieces currently stored in the simulated hopper. */
  public Pose3d[] getHopperGamePiecePoses() {
    return hopper != null ? hopper.getGamePiecePoses() : new Pose3d[0];
  }

  /** Resets the simulated successful score counter. */
  public void resetSimulationScoreCounter() {
    if (launchedFuelSim != null) {
      launchedFuelSim.resetSuccessfulScoreCount();
    }
  }

  /** Returns the simulated successful score counter. */
  public int getSimulationScoreCounter() {
    return launchedFuelSim != null ? launchedFuelSim.getSuccessfulScoreCount() : 0;
  }

  /**
   * Resets MapleSim drive pose to the selected autonomous initial PathPlanner pose, if available.
   */
  public void resetSimulationPoseToAutonomousInitialPose(Command autonomousCommand) {
    if (Constants.currentMode != Constants.Mode.SIM || drive.getSimulation() == null) {
      return;
    }
    if (!(autonomousCommand instanceof PathPlannerAuto pathPlannerAuto)) {
      return;
    }

    Pose2d startingPose = pathPlannerAuto.getStartingPose();
    if (startingPose == null) {
      return;
    }

    Pose2d allianceAdjustedPose =
        Constants.isAllianceRed() ? FlippingUtil.flipFieldPose(startingPose) : startingPose;
    drive.getSimulation().setSimulationWorldPose(allianceAdjustedPose);
    drive.setPose(allianceAdjustedPose);
  }
}
