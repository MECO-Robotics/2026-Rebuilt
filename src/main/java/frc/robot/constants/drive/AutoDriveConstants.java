package frc.robot.constants.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Placeholder constants for higher-level autonomous drive targeting. */
public class AutoDriveConstants {

  // If we want to implement the option of auto driving,
  // we need to add the positions of possible scoring spots
  // in relation to field
  public static Pose2d[][] poses = {{new Pose2d(1, 1, Rotation2d.fromDegrees(180))}};

  public static double DISTANCE_THRESH = 1.0;
}
