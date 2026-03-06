package frc.robot.constants.drive;

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
import frc.robot.util.mechanical_advantage.swerve.ModuleLimits;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

/**
 * Mechanical, electrical, and path-planning constants for the swerve
 * drivetrain.
 */
public class DriveConstants {
	public static double spinMultipler = 0.50; // Multiplier for max spin speed, used to reduce spin speed for better
												// control during
	// testing
	public static final double odometryFrequency = new CANBus(DriveMotorConstants.canBusName).isNetworkFD()
			&& new CANBus(AzimuthMotorConstants.canBusName).isNetworkFD() ? 250.0 : 100.0; // If both Azimuth and Drive
																							// use CANFD, sample
																							// odometry at 250 Hz, if
																							// either
	// loop
	// is not FD, sample odometry at 100 Hz

	// NavX's highest allowed odometry frequency is now 200, this change reflects
	// that
	public static final double navXFrequency = new CANBus(DriveMotorConstants.canBusName).isNetworkFD()
			&& new CANBus(AzimuthMotorConstants.canBusName).isNetworkFD() ? 200.0 : 100.0;

	public static final double trackWidth = Units.inchesToMeters(26.5);
	public static final double wheelBase = Units.inchesToMeters(26.5);
	public static final double driveBaseRadius = Math.hypot(wheelBase / 2.0, wheelBase / 2.0);
	public static final double driveWheelRadiusMeters = Units.inchesToMeters(2);
	public static final Translation2d[] moduleTranslations = new Translation2d[]{
			new Translation2d(trackWidth / 2.0, wheelBase / 2.0), new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
			new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
			new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)};

	public static final double kSteerInertia = 0.004;
	public static final double kDriveInertia = 0.025;

	public static final LinearVelocity maxSpeedAt12Volts = FeetPerSecond.of(15); // X2i 16.89
	// 11t pinion

	// Sim motor config
	public static final DCMotor driveGearbox = DCMotor.getKrakenX60Foc(1);
	public static final DCMotor turnGearbox = DCMotor.getKrakenX60Foc(1);

	// Sim Module config
	public static final int gearingLevel = 3;
	public static final int drivePinionTeeth = 11;
	public static final SwerveModuleSimulationConfig simModule = COTS.ofSwerveXFlipped(turnGearbox, driveGearbox,
			COTS.WHEELS.VEX_GRIP_V2.cof, gearingLevel, drivePinionTeeth);

	// Drive motor configuration
	public static final int driveMotorCurrentLimit = 80;
	public static final double driveMotorGearRatio = simModule.DRIVE_GEAR_RATIO;

	// Drive encoder configuration
	public static final double driveEncoderPositionFactor = 2 * Math.PI / driveMotorGearRatio; // Rotor Rotations ->
	// Wheel Radians
	public static final double driveEncoderVelocityFactor = (2 * Math.PI) / 60.0 / driveMotorGearRatio; // Rotor RPM ->
	// Wheel Rad/Sec

	// Turn motor configuration
	public static final double steerMotorGearRatio = simModule.STEER_GEAR_RATIO;
	public static final int turnMotorCurrentLimit = 60;

	// Turn encoder configuration
	public static final double turnEncoderPositionFactor = 2 * Math.PI / steerMotorGearRatio; // Rotations -> Radians
	public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0 / steerMotorGearRatio; // RPM -> Rad/Sec

	// PathPlanner configuration
	public static final Mass robotMass = Pound.of(80); // Convert pounds to kg
	public static final double robotMoiLbInSq = 10124.402142;
	public static final double robotMOI = Units.lbsToKilograms(robotMoiLbInSq) * Math.pow(Units.inchesToMeters(1.0), 2);
	public static final double wheelCOF = 2.255;
	public static final RobotConfig ppConfig = createPathPlannerConfig();

	public static final PIDConstants translationPID = new PIDConstants(5, 0, 0);
	public static final PIDConstants rotationPID = new PIDConstants(5, 0, 0);

	public static final ModuleLimits defaultModuleLimits = new ModuleLimits(10, 4, 9, 8);

	// Bumper-to-bumper chassis dimensions used by MapleSim collision body.
	public static final double mapleSimBumperLengthInches = 30.0;
	public static final double mapleSimBumperWidthInches = 30.0;
	// MapleSim contact tuning (dyn4j fixture values). Keep defaults unless you need
	// less
	// sliding/bounce.
	public static final double mapleSimBumperFriction = 0.65;
	public static final double mapleSimBumperRestitution = 0.08;

	public static final DriveTrainSimulationConfig mapleSimConfig = DriveTrainSimulationConfig.Default()
			.withBumperSize(Inches.of(mapleSimBumperLengthInches), Inches.of(mapleSimBumperWidthInches))
			.withCustomModuleTranslations(moduleTranslations).withRobotMass(robotMass).withGyro(COTS.ofPigeon2())
			.withSwerveModule(simModule);

	private static RobotConfig createPathPlannerConfig() {
		try {
			// Keep runtime robot config aligned with the PathPlanner GUI settings used to
			// author paths.
			return RobotConfig.fromGUISettings();
		} catch (Exception e) {
			System.err.println(
					"Failed to load PathPlanner GUI robot config, using DriveConstants fallback: " + e.getMessage());
			return new RobotConfig(robotMass.in(Kilogram), robotMOI,
					new ModuleConfig(driveWheelRadiusMeters, maxSpeedAt12Volts.in(MetersPerSecond), wheelCOF,
							driveGearbox.withReduction(driveMotorGearRatio), driveMotorCurrentLimit, 1),
					moduleTranslations);
		}
	}
}
