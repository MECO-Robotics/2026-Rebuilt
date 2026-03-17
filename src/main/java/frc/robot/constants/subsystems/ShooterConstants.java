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

public final class ShooterConstants {
	private ShooterConstants() {
	}

	public static final FlywheelHardwareConfig TOP_INDEXER_ROLLER_CONFIG = new FlywheelHardwareConfig(new int[]{32},
			new boolean[]{false}, 1, 0.025, 20, "");
	public static final FlywheelHardwareConfig BOTTOM_INDEXER_ROLLER_CONFIG = new FlywheelHardwareConfig(new int[]{31},
			new boolean[]{false}, 1, 0.025, 20, "");
	public static final FlywheelGains INDEXER_ROLLER_GAINS = new FlywheelGains(0, 0, 0, 0, 0, 0, 0, 0);

	public static final FlywheelHardwareConfig CONVEYOR_CONFIG = new FlywheelHardwareConfig(new int[]{23},
			new boolean[]{false}, 1, 0.025, 20, "");
	public static final FlywheelGains CONVEYOR_GAINS = new FlywheelGains(0.0, 0.0, 0.0, 0.0, 0.065, 0.0, 0.0, 0.0);

	public static final FlywheelHardwareConfig FLYWHEEL_ROLLER_CONFIG = new FlywheelHardwareConfig(new int[]{34, 35},
			new boolean[]{false, true}, 22.0 / 14, 0.006421, 40, "MECO CANIvore");
	public static final FlywheelGains FLYWHEEL_ROLLER_GAINS = new FlywheelGains(0.3, 0.0, 0.0, 0.25, 0.2, 0.0, 10, 0.5);

	public static final PositionJointGains HOOD_GAINS = new PositionJointGains(20, 0.0, 0.0, 0.5, 0.1, 0.0, 0.0, 4.0,
			8.0, 0.0, 0.049, 0.05, 0.0);
	public static final PositionJointHardwareConfig HOOD_CONFIG = new PositionJointHardwareConfig(new int[]{33},
			new boolean[]{false}, (21 / 1) * 5, 0.01, 60, GravityType.COSINE, EncoderType.INTERNAL, 0,
			Rotation2d.fromRotations(0), "");

	public static final UnitInterpolatingMap<DistanceUnit, AngleUnit> hoodMap = new UnitInterpolatingMap<>(Units.Meters,
			Units.Radians);
	public static final UnitInterpolatingMap<DistanceUnit, AngularVelocityUnit> shooterVelocityMap = new UnitInterpolatingMap<>(
			Units.Meters, Units.RevolutionsPerSecond);

	static {
		hoodMap.put(Units.Meters.of(1.35), Units.Degrees.of(0.0));
		hoodMap.put(Units.Meters.of(3.8), Units.Degrees.of(22.0));
		hoodMap.put(Units.Meters.of(5.6), Units.Degrees.of(22.0));

		shooterVelocityMap.put(Units.Meters.of(1.35), Units.RevolutionsPerSecond.of(20.0));
		shooterVelocityMap.put(Units.Meters.of(3.8), Units.RevolutionsPerSecond.of(24.5));
		shooterVelocityMap.put(Units.Meters.of(5.6), Units.RevolutionsPerSecond.of(28));
	}
}
