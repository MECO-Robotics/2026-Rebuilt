package frc.robot.constants;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Units;
import frc.robot.util.UnitInterpolatingMap;

public class ShooterConstants {

  public static final UnitInterpolatingMap<DistanceUnit, AngleUnit> shooterDeflectorAngleMap =
      new UnitInterpolatingMap<>(Units.Meters, Units.Radians);
  public static final UnitInterpolatingMap<DistanceUnit, AngularVelocityUnit> shooterVelocityMap =
      new UnitInterpolatingMap<>(Units.Meters, Units.RevolutionsPerSecond);
  public static final UnitInterpolatingMap<DistanceUnit, TimeUnit> timeOfFlightMap =
      new UnitInterpolatingMap<>(Units.Meters, Units.Seconds);

  static {
  }
}
