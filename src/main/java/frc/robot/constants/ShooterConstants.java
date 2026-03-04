package frc.robot.constants;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Units;
import frc.robot.util.UnitInterpolatingMap;

public final class ShooterConstants {
  private ShooterConstants() {}

  public static final UnitInterpolatingMap<DistanceUnit, AngleUnit> hoodMap =
      new UnitInterpolatingMap<>(Units.Meters, Units.Radians);
  public static final UnitInterpolatingMap<DistanceUnit, AngularVelocityUnit> shooterVelocityMap =
      new UnitInterpolatingMap<>(Units.Meters, Units.RevolutionsPerSecond);

  static {
    hoodMap.put(Units.Meters.of(1.35), Units.Degrees.of(0.0));
    hoodMap.put(Units.Meters.of(3.8), Units.Degrees.of(22.0));
    hoodMap.put(Units.Meters.of(5.6), Units.Degrees.of(22.0));

    shooterVelocityMap.put(Units.Meters.of(1.35), Units.RevolutionsPerSecond.of(20.0));
    shooterVelocityMap.put(Units.Meters.of(3.8), Units.RevolutionsPerSecond.of(24.5));
    shooterVelocityMap.put(Units.Meters.of(5.6), Units.RevolutionsPerSecond.of(28));
  }
}
