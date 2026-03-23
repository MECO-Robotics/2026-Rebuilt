package frc.robot.constants.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Units;
import frc.robot.constants.types.FlywheelConstants.FlywheelGains;
import frc.robot.constants.types.FlywheelConstants.FlywheelHardwareConfig;
import frc.robot.constants.types.PositionJointConstants.EncoderType;
import frc.robot.constants.types.PositionJointConstants.GravityType;
import frc.robot.constants.types.PositionJointConstants.PositionJointGains;
import frc.robot.constants.types.PositionJointConstants.PositionJointHardwareConfig;
import frc.robot.util.UnitInterpolatingMap;
import frc.robot.util.mechanical_advantage.LoggedTunableNumber;

public final class ShooterConstants {
	private ShooterConstants() {
	}

	public static final FlywheelHardwareConfig TOP_INDEXER_ROLLER_CONFIG = new FlywheelHardwareConfig(new int[]{32},
			new boolean[]{false}, 1, 0.025, 40, "");
	public static final FlywheelHardwareConfig BOTTOM_INDEXER_ROLLER_CONFIG = new FlywheelHardwareConfig(new int[]{31},
			new boolean[]{false}, 1, 0.025, 40, "MECO CANIvore");
	public static final FlywheelGains INDEXER_ROLLER_GAINS = new FlywheelGains(0, 0, 0, 0, 0, 0, 0, 0);

	public static final FlywheelHardwareConfig CONVEYOR_CONFIG = new FlywheelHardwareConfig(new int[]{23},
			new boolean[]{false}, 1, 0.025, 40, "");
	public static final FlywheelGains CONVEYOR_GAINS = new FlywheelGains(0.0, 0.0, 0.0, 0.0, 0.065, 0.0, 0.0, 0.0);

	public static final FlywheelHardwareConfig FLYWHEEL_ROLLER_CONFIG = new FlywheelHardwareConfig(new int[]{34, 35},
			new boolean[]{false, true}, 22.0 / 14, 0.006421, 40, "MECO CANIvore");
	public static final FlywheelGains FLYWHEEL_ROLLER_GAINS = new FlywheelGains(0.8, 0.0, 0.01, 0.33, 0.19, 0.15, 100,
			0.5);

	public static final PositionJointGains HOOD_GAINS = new PositionJointGains(20, 0.0, 0.0, 0.5, 0.1, 0.0, 0.0, 4.0,
			8.0, 0.0, 0.049, 0.05, 0.0);
	public static final PositionJointHardwareConfig HOOD_CONFIG = new PositionJointHardwareConfig(new int[]{33},
			new boolean[]{false}, (21 / 1) * 5, 0.01, 60, GravityType.COSINE, EncoderType.INTERNAL, 0,
			Rotation2d.fromRotations(0), "");

	// Regression constants for hood and shooter velocity. These are used to
	// calculate the feedforward for the hood and shooter based on distance to
	// target.
	public static final double[] kHOOODREGCALC = {0.018, 0.00042, -0.00000676}; // -0.018 + 4.42E-04x + -6.76E-07x^2
	public static final double[] kSHOOTERVELREGCALC = {22, 0.134, -1.19E-04}; // 22.5 + 0.134x + -1.19E-04x^2

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
		public static final LoggedTunableNumber HUB = new LoggedTunableNumber("Presets/Hood/HubPos", 0.001);
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
		public static final LoggedTunableNumber HUB = new LoggedTunableNumber("Presets/Shooter/HubVeloc", 29);
		public static final LoggedTunableNumber FERRY = new LoggedTunableNumber("Presets/Shooter/FerryVeloc", 45);
		public static final LoggedTunableNumber IDLE = new LoggedTunableNumber("Presets/Shooter/IdleVeloc", 0);
		public static final LoggedTunableNumber TRENCH = new LoggedTunableNumber("Presets/Shooter/Trench", 38);
	}

	static {
		hoodMap.put(Units.Inches.of(58.0), Units.Rotations.of(0.005));
		hoodMap.put(Units.Inches.of(114.25), Units.Rotations.of(0.025));
		hoodMap.put(Units.Inches.of(163), Units.Rotations.of(0.035));
		hoodMap.put(Units.Inches.of(236), Units.Rotations.of(0.049));

		shooterVelocityMap.put(Units.Inches.of(58.0), Units.RevolutionsPerSecond.of(30));
		shooterVelocityMap.put(Units.Inches.of(114.25), Units.RevolutionsPerSecond.of(36));
		shooterVelocityMap.put(Units.Inches.of(163), Units.RevolutionsPerSecond.of(41.5));
		shooterVelocityMap.put(Units.Inches.of(236), Units.RevolutionsPerSecond.of(47.5));
	}
}
