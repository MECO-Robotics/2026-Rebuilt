package frc.robot.constants.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Units;
import frc.robot.constants.types.FlywheelConstants.FlywheelGains;
import frc.robot.constants.types.FlywheelConstants.FlywheelHardwareConfig;
import frc.robot.constants.types.PositionJointConstants.EncoderType;
import frc.robot.constants.types.PositionJointConstants.MechanismType;
import frc.robot.constants.types.PositionJointConstants.PositionJointGains;
import frc.robot.constants.types.PositionJointConstants.PositionJointHardwareConfig;
import frc.robot.util.UnitInterpolatingMap;
import frc.robot.util.mechanical_advantage.LoggedTunableNumber;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public final class ShooterConstants {
	private static final double SHOOTER_SIM_MOI_KG_METERS_SQUARED = 24.0 * 0.45359237
			* Math.pow(Units.Inches.of(1.0).in(Units.Meters), 2);

	private ShooterConstants() {
	}

	/**
	 * Shooter throat location behind robot center, expressed in robot coordinates.
	 * The projectile still travels toward model-forward (+X).
	 */
	public static final Translation2d SHOOTER_EXIT_TRANSLATION = new Translation2d(-0.19, 0.0);

	/**
	 * The intake, shooter, and Limelight face robot-forward (+X). This fixed
	 * physical offset must not change with alliance.
	 */
	public static final Rotation2d SHOOTER_YAW_OFFSET = Rotation2d.kZero;

	/**
	 * The drivetrain heading convention is 180 degrees from the Remy
	 * model/mechanism-forward convention used by intake and projectile simulation.
	 */
	public static final Rotation2d AUTO_AIM_HEADING_OFFSET = Rotation2d.kPi;
	/** Physical shooter release height used by the ferry landing calculation. */
	public static final double SHOOTER_RELEASE_HEIGHT_METERS = 0.45;
	/** Main shooter-wheel surface speed produced by one flywheel RPS. */
	public static final double MAIN_WHEEL_METERS_PER_SECOND_PER_RPS = 4.0 * Math.PI * 0.0254;
	/** Counter-wheel surface speed divided by main-wheel surface speed. */
	public static final double COUNTER_TO_MAIN_SHOOTER_WHEEL_SPEED_RATIO = (48.0 / 54.0) * (1.5 / 4.0);
	/**
	 * Current projectile-speed scale for the worn shooter tape. The current 30 RPS
	 * close shot replaces the previous 28.5 RPS reference.
	 */
	public static final double SHOOTER_SURFACE_EFFICIENCY_SCALE = 28.5 / 30.0;
	/** Nominal ball speed produced by one flywheel RPS. */
	public static final double PROJECTILE_METERS_PER_SECOND_PER_FLYWHEEL_RPS = MAIN_WHEEL_METERS_PER_SECOND_PER_RPS
			* (1.0 + COUNTER_TO_MAIN_SHOOTER_WHEEL_SPEED_RATIO) / 2.0 * SHOOTER_SURFACE_EFFICIENCY_SCALE;
	/** Hood deflection down from vertical when the hood encoder reads zero. */
	public static final double HOOD_ZERO_DEFLECTION_FROM_VERTICAL_RADIANS = Math.toRadians(21.0 - 5.0);
	public static final double MIN_CALIBRATED_DISTANCE_METERS = Units.Inches.of(46.003).in(Units.Meters);
	public static final double MAX_CALIBRATED_DISTANCE_METERS = Units.Inches.of(236.0).in(Units.Meters);
	public static final double READY_DEBOUNCE_SECONDS = 0.15;
	public static final double FLYWHEEL_READY_TOLERANCE_RPS = 0.75;
	public static final double HOOD_READY_TOLERANCE_ROTATIONS = 0.002;
	public static final double HEADING_READY_TOLERANCE_DEGREES = 2.0;
	public static final double MAX_SHOOTING_TRANSLATION_METERS_PER_SECOND = 0.25;
	public static final double MAX_SHOOTING_ROTATION_RADIANS_PER_SECOND = Units.Degrees.of(10.0).in(Units.Radians);
	public static final double AUTO_READY_TIMEOUT_SECONDS = 2.0;
	public static final double AUTO_FEED_SECONDS = 1.0;
	public static final double AUTO_TOTAL_TIMEOUT_SECONDS = 3.0;
	public static final double MAX_CALIBRATION_FLYWHEEL_RPS = 60.0;

	public static final FlywheelHardwareConfig TOP_INDEXER_ROLLER_CONFIG = new FlywheelHardwareConfig(new int[]{32},
			new boolean[]{false}, 1, 0.025, 30, "");
	public static final FlywheelHardwareConfig BOTTOM_INDEXER_ROLLER_CONFIG = new FlywheelHardwareConfig(new int[]{31},
			new boolean[]{false}, 1, 0.025, 30, "MECO 2");
	public static final FlywheelGains INDEXER_ROLLER_GAINS = new FlywheelGains(0, 0, 0, 0, 0, 0, 0, 0);

	public static final FlywheelHardwareConfig CONVEYOR_CONFIG = new FlywheelHardwareConfig(new int[]{23},
			new boolean[]{true}, 1, 0.025, 30, "");
	public static final FlywheelGains CONVEYOR_GAINS = new FlywheelGains(0.0, 0.0, 0.0, 0.0, 0.065, 0.0, 0.0, 0.0);

	public static final FlywheelHardwareConfig FLYWHEEL_ROLLER_CONFIG = new FlywheelHardwareConfig(new int[]{34, 35},
			new boolean[]{false, true}, 22.0 / 14, SHOOTER_SIM_MOI_KG_METERS_SQUARED, 40, "MECO 2");
	public static final FlywheelGains FLYWHEEL_ROLLER_GAINS = new FlywheelGains(0.8, 0.0, 0.01, 0.33, 0.19, 0.15, 100,
			0.5);

	public static final PositionJointGains HOOD_GAINS = new PositionJointGains(20, 0.0, 0.0, 0.5, 0.1, 0.0, 0.0, 4.0,
			100.0, 0.0, 0.049, 0.002, 0.0);
	public static final PositionJointHardwareConfig HOOD_CONFIG = new PositionJointHardwareConfig(new int[]{33},
			new boolean[]{false}, (21 / 1) * 5, 0.01, 40, EncoderType.INTERNAL, 0, MechanismType.ROTATIONAL, 0.0,
			Rotation2d.fromRotations(0), "");

	public static final UnitInterpolatingMap<DistanceUnit, AngleUnit> hoodMap = new UnitInterpolatingMap<>(Units.Meters,
			Units.Radians);
	public static final UnitInterpolatingMap<DistanceUnit, AngularVelocityUnit> shooterVelocityMap = new UnitInterpolatingMap<>(
			Units.Meters, Units.RevolutionsPerSecond);

	/** Live dashboard inputs used only while deliberately calibrating a shot. */
	public static final class CALIBRATION {
		public static final LoggedNetworkBoolean ENABLED = new LoggedNetworkBoolean(
				"/TunableNumbers/ShooterCalibration/Enabled", false);
		public static final LoggedNetworkNumber HOOD_ROTATIONS = new LoggedNetworkNumber(
				"/TunableNumbers/ShooterCalibration/HoodRotations", 0.000);
		public static final LoggedNetworkNumber FLYWHEEL_RPS = new LoggedNetworkNumber(
				"/TunableNumbers/ShooterCalibration/FlywheelRPS", 30.0);

		private CALIBRATION() {
		}
	}

	/** Conveyor roller preset voltages. */
	public final class CONVEYOR_PRESET {
		public static final LoggedTunableNumber FEED = new LoggedTunableNumber("Presets/Conveyor/IntakeVolts", -11);
		public static final LoggedTunableNumber IDLE = new LoggedTunableNumber("Presets/Conveyor/StopVolts", 0);
	}

	/** Intake rotation preset positions. */
	public static final class HOOD_PRESET {
		public static final LoggedTunableNumber STOW = new LoggedTunableNumber("Presets/Hood/StowPos", 0);
		public static final LoggedTunableNumber HUB = new LoggedTunableNumber("Presets/Hood/HubPos", 0.000);
		public static final LoggedTunableNumber FERRY = new LoggedTunableNumber("Presets/Hood/FerryPos", 0.049);
		public static final LoggedTunableNumber TRENCH = new LoggedTunableNumber("Presets/Hood/TrenchPos", 0.049);
	}

	/** Indexer roller preset voltages. */
	public final class INDEXER_PRESET {
		public static final LoggedTunableNumber FEED_BOTTOM = new LoggedTunableNumber("Presets/Indexer/BottomVolts",
				-10);
		public static final LoggedTunableNumber FEED_TOP = new LoggedTunableNumber("Presets/Indexer/TopVolts", 10);
		public static final LoggedTunableNumber IDLE_BOTTOM = new LoggedTunableNumber("Presets/Indexer/IdleVolts", 0);
		public static final LoggedTunableNumber IDLE_TOP = new LoggedTunableNumber("Presets/Indexer/IdleVolts", 0);
	}

	/**
	 * Shooter roller preset voltages. (NOTE: MAINLY FOR TESTING/SHUTTLE (maybe))
	 */
	public final class SHOOTER_PRESET {
		public static final LoggedTunableNumber HUB = new LoggedTunableNumber("Presets/Shooter/HubVeloc", 30);
		public static final LoggedTunableNumber FERRY = new LoggedTunableNumber("Presets/Shooter/FerryVeloc", 55);
		public static final LoggedTunableNumber IDLE = new LoggedTunableNumber("Presets/Shooter/IdleVeloc", 0);
		public static final LoggedTunableNumber TRENCH = new LoggedTunableNumber("Presets/Shooter/Trench", 35);
	}

	static {
		hoodMap.put(Units.Inches.of(46.003), Units.Rotations.of(0.000)); // Right in front of the hub
		hoodMap.put(Units.Inches.of(123.24), Units.Rotations.of(0.019));
		hoodMap.put(Units.Inches.of(144.60), Units.Rotations.of(0.020));
		hoodMap.put(Units.Inches.of(153.64), Units.Rotations.of(0.020)); // Trench edge
		hoodMap.put(Units.Inches.of(191.86), Units.Rotations.of(0.020));
		hoodMap.put(Units.Inches.of(202.78), Units.Rotations.of(0.020));
		hoodMap.put(Units.Inches.of(210.47), Units.Rotations.of(0.020));
		hoodMap.put(Units.Inches.of(236), Units.Rotations.of(0.049));

		// Values other than the measured shots are provisional values scaled by
		// 30/28.5 for the worn shooter tape.
		shooterVelocityMap.put(Units.Inches.of(46.003), Units.RevolutionsPerSecond.of(30));
		shooterVelocityMap.put(Units.Inches.of(123.24), Units.RevolutionsPerSecond.of(37.9));
		shooterVelocityMap.put(Units.Inches.of(144.60), Units.RevolutionsPerSecond.of(40.0));
		shooterVelocityMap.put(Units.Inches.of(153.64), Units.RevolutionsPerSecond.of(41.0)); // Trench edge
		shooterVelocityMap.put(Units.Inches.of(191.86), Units.RevolutionsPerSecond.of(46.0));
		shooterVelocityMap.put(Units.Inches.of(202.78), Units.RevolutionsPerSecond.of(47.0));
		shooterVelocityMap.put(Units.Inches.of(210.47), Units.RevolutionsPerSecond.of(47.0));
		shooterVelocityMap.put(Units.Inches.of(236), Units.RevolutionsPerSecond.of(53.7));
	}
}
