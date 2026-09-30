package frc.robot.constants.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
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

public final class ShooterConstants {
	private static final double SHOOTER_SIM_MOI_KG_METERS_SQUARED = 24.0 * 0.45359237
			* Math.pow(Units.Inches.of(1.0).in(Units.Meters), 2);

	private ShooterConstants() {
	}

	/** The shooter faces robot-forward, in the same direction as the intake. */
	public static final Rotation2d SHOOTER_YAW_OFFSET = Rotation2d.kZero;
	public static final double MIN_CALIBRATED_DISTANCE_METERS = Units.Inches.of(58.0).in(Units.Meters);
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

	/** Conveyor roller preset voltages. */
	public final class CONVEYOR_PRESET {
		public static final LoggedTunableNumber FEED = new LoggedTunableNumber("Presets/Conveyor/IntakeVolts", -11);
		public static final LoggedTunableNumber IDLE = new LoggedTunableNumber("Presets/Conveyor/StopVolts", 0);
	}

	/** Intake rotation preset positions. */
	public static final class HOOD_PRESET {
		public static final LoggedTunableNumber STOW = new LoggedTunableNumber("Presets/Hood/StowPos", 0);
		public static final LoggedTunableNumber HUB = new LoggedTunableNumber("Presets/Hood/HubPos", 0.005);
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
		hoodMap.put(Units.Inches.of(58.0), Units.Rotations.of(HOOD_PRESET.HUB.get()));
		hoodMap.put(Units.Inches.of(114.25), Units.Rotations.of(0.025));
		hoodMap.put(Units.Inches.of(163), Units.Rotations.of(0.049));
		hoodMap.put(Units.Inches.of(236), Units.Rotations.of(0.049));

		shooterVelocityMap.put(Units.Inches.of(58.0), Units.RevolutionsPerSecond.of(28.5));
		shooterVelocityMap.put(Units.Inches.of(114.25), Units.RevolutionsPerSecond.of(34));
		shooterVelocityMap.put(Units.Inches.of(163), Units.RevolutionsPerSecond.of(40));
		shooterVelocityMap.put(Units.Inches.of(192), Units.RevolutionsPerSecond.of(46));
		shooterVelocityMap.put(Units.Inches.of(236), Units.RevolutionsPerSecond.of(51));
	}
}
