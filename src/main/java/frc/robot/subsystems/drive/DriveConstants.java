// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pound;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import frc.robot.subsystems.drive.azimuth_motor.AzimuthMotorConstants;
import frc.robot.subsystems.drive.drive_motor.DriveMotorConstants;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

/** Mechanical, electrical, and path-planning constants for the swerve drivetrain. */
public class DriveConstants {
  public static final double odometryFrequency =
      new CANBus(DriveMotorConstants.canBusName).isNetworkFD()
              && new CANBus(AzimuthMotorConstants.canBusName).isNetworkFD()
          ? 250.0
          : 100.0; // If both Azimuth and Drive use CANFD, sample odometry at 250 Hz, if either
  // loop
  // is not FD, sample odometry at 100 Hz

  // NavX's highest allowed odometry frequency is now 200, this change reflects
  // that
  public static final double navXFrequency =
      new CANBus(DriveMotorConstants.canBusName).isNetworkFD()
              && new CANBus(AzimuthMotorConstants.canBusName).isNetworkFD()
          ? 200.0
          : 100.0;

  public static final double trackWidth = Units.inchesToMeters(26.5);
  public static final double wheelBase = Units.inchesToMeters(26.5);
  public static final double driveBaseRadius = Math.hypot(wheelBase / 2.0, wheelBase / 2.0);
  public static final double driveWheelRadiusMeters = Units.inchesToMeters(2);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
      };

  public static final double kSteerInertia = 0.004;
  public static final double kDriveInertia = 0.025;

  public static final LinearVelocity maxSpeedAt12Volts =
      FeetPerSecond.of(15); // MK4i 16.5 ft/s L3 Kraken FOC With
  // 14t pinion

  // Sim motor config
  public static final DCMotor driveGearbox = DCMotor.getKrakenX60Foc(1);
  public static final DCMotor turnGearbox = DCMotor.getKrakenX60Foc(1);

  // Sim Module config
  public static final int gearingLevel = 3;
  public static final int drivePinionTeeth = 11;
  public static final SwerveModuleSimulationConfig simModule =
      COTS.ofSwerveXFlipped(
          turnGearbox, driveGearbox, COTS.WHEELS.VEX_GRIP_V2.cof, gearingLevel, drivePinionTeeth);

  // Drive motor configuration
  public static final int driveMotorCurrentLimit = 60;
  public static final double driveMotorGearRatio = simModule.DRIVE_GEAR_RATIO;

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor =
      2 * Math.PI / driveMotorGearRatio; // Rotor Rotations ->
  // Wheel Radians
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorGearRatio; // Rotor RPM ->
  // Wheel Rad/Sec

  // Turn motor configuration
  public static final double steerMotorGearRatio = simModule.STEER_GEAR_RATIO;
  public static final int turnMotorCurrentLimit = 40;

  // Turn encoder configuration
  public static final double turnEncoderPositionFactor =
      2 * Math.PI / steerMotorGearRatio; // Rotations -> Radians
  public static final double turnEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / steerMotorGearRatio; // RPM -> Rad/Sec

  // PathPlanner configuration
  public static final Mass robotMass = Pound.of(80); // Convert kg to pounds
  public static final double robotMOI = 6.883;
  public static final double wheelCOF = COTS.WHEELS.VEX_GRIP_V2.cof;
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMass.in(Kilogram),
          robotMOI,
          new ModuleConfig(
              driveWheelRadiusMeters,
              maxSpeedAt12Volts.in(MetersPerSecond),
              wheelCOF,
              driveGearbox.withReduction(driveMotorGearRatio),
              driveMotorCurrentLimit,
              1),
          moduleTranslations);

  public static final PIDConstants translationPID = new PIDConstants(5, 0, 0);
  public static final PIDConstants rotationPID = new PIDConstants(5, 0, 0);

  // Bumper-to-bumper chassis dimensions used by MapleSim collision body.
  public static final double mapleSimBumperLengthInches = 30.0;
  public static final double mapleSimBumperWidthInches = 30.0;

  public static final DriveTrainSimulationConfig mapleSimConfig =
      DriveTrainSimulationConfig.Default()
          .withBumperSize(
              Inches.of(mapleSimBumperLengthInches), Inches.of(mapleSimBumperWidthInches))
          .withCustomModuleTranslations(moduleTranslations)
          .withRobotMass(robotMass)
          .withGyro(COTS.ofPigeon2())
          .withSwerveModule(simModule);
}
