package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public class FieldConstants {

  public static class Hub {
    public static final Translation2d centerofHub =
        new Translation2d(Units.inchesToMeters(182.105), Units.inchesToMeters(158.845));

    // measured from floor to top of funnel
    public static final double hubHeight = Units.inchesToMeters(72);
  }

  //   public static Translation2d hubPosition() {
  //     final Optional<Alliance> alliance = DriverStation.getAlliance();
  //     if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
  //       return new Translation2d(Units.inchesToMeters(182.105), Units.inchesToMeters(158.845));
  //     }
  //     return new Translation2d(Units.inchesToMeters(469.115), Units.inchesToMeters(158.845));
  //   }
}
