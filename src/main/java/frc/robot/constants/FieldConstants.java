package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.List;
import java.util.Optional;

/**
 * Contains various field dimensions and useful reference points. All units are
 * in meters
 */
public class FieldConstants {
	public static class AutoDrivePoses {
		public static final Pose2d FENDER = pose(2.960, 3.938, 0.0);

		public static final Pose2d LEFT_TRENCH = pose(7.159, 4.993, -136.0);
		public static final Pose2d LEFT_TRENCH_RETURN = pose(2.697, 5.633, -39.8);
		public static final Pose2d LEFT_TRENCH_FAR = pose(7.892, 5.944, 130.4);
		public static final Pose2d LEFT_TRENCH_CORNER = pose(2.549, 7.229, -57.8);
		public static final Pose2d LEFT_TRENCH_CLEANUP = pose(8.292, 5.342, 8.7);

		public static final Pose2d RIGHT_TRENCH = pose(7.159, 3.077, 136.0);
		public static final Pose2d RIGHT_TRENCH_RETURN = pose(2.234, 2.416, 35.0);
		public static final Pose2d RIGHT_TRENCH_FAR = pose(7.173, 2.416, -178.8);
		public static final Pose2d RIGHT_TRENCH_CORNER = pose(2.611, 2.480, 37.6);
		public static final Pose2d RIGHT_TRENCH_CLEANUP = pose(8.292, 2.728, -8.7);

		public static final Pose2d LEFT_DEPOT = pose(3.333, 2.670, 45.0);
		public static final Pose2d RIGHT_DEPOT = pose(3.333, 5.400, -45.0);
		public static final Pose2d LEFT_RUSH_START = pose(3.333, 5.364, -45.0);
		public static final Pose2d RIGHT_RUSH_START = pose(3.333, 2.706, 45.0);

		public static final Pose2d LEFT_BUMP = pose(3.206, 5.615, -89.3);
		public static final Pose2d LEFT_BUMP_FAR = pose(7.922, 6.522, 131.5);
		public static final Pose2d RIGHT_BUMP = pose(7.159, 3.077, 136.0);
		public static final Pose2d RIGHT_BUMP_NEAR = pose(2.234, 2.416, 35.0);
		public static final Pose2d RIGHT_BUMP_FAR = pose(7.173, 2.416, -178.8);
		public static final Pose2d RIGHT_BUMP_CORNER = pose(2.611, 2.480, 37.6);
		public static final Pose2d RIGHT_BUMP_CLEANUP = pose(8.292, 2.728, -8.7);

		public static final Pose2d LEFT_PROM_COUNTER = pose(2.985, 5.806, -47.9);
		public static final Pose2d LEFT_PROM_COUNTER_SEND = pose(7.678, 4.686, -41.8);
		public static final Pose2d LEFT_PROM_COUNTER_CLEANUP = pose(5.911, 4.064, -110.0);
		public static final Pose2d RIGHT_PROM_COUNTER = pose(2.985, 2.264, 47.9);
		public static final Pose2d RIGHT_PROM_COUNTER_SEND = pose(7.678, 3.384, 41.8);
		public static final Pose2d RIGHT_PROM_COUNTER_CLEANUP = pose(5.911, 4.006, 110.0);

		private AutoDrivePoses() {
		}

		private static Pose2d pose(double xMeters, double yMeters, double headingDegrees) {
			return new Pose2d(xMeters, yMeters, Rotation2d.fromDegrees(headingDegrees));
		}
	}

	public static class Hub {
		public static final Translation2d BLUE_HUB_POSITION = new Translation2d(Units.inchesToMeters(182.105),
				Units.inchesToMeters(158.845));
		public static final Translation2d RED_HUB_POSITION = new Translation2d(Units.inchesToMeters(469.115),
				Units.inchesToMeters(158.845));

		public static Translation2d hubPosition() {
			final Optional<Alliance> alliance = DriverStation.getAlliance();
			if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
				return BLUE_HUB_POSITION;
			}
			return RED_HUB_POSITION;
		}

		// measured from floor to top of funnel
		public static final double hubHeight = Units.inchesToMeters(72);
	}

	public static class Obstacles {
		private static final double HUB_AVOIDANCE_HALF_WIDTH_METERS = 1.15;

		public static List<Pair<Translation2d, Translation2d>> pathfindingObstacles() {
			return List.of(squareObstacle(Hub.BLUE_HUB_POSITION, HUB_AVOIDANCE_HALF_WIDTH_METERS),
					squareObstacle(Hub.RED_HUB_POSITION, HUB_AVOIDANCE_HALF_WIDTH_METERS));
		}

		private static Pair<Translation2d, Translation2d> squareObstacle(Translation2d center, double halfWidthMeters) {
			return Pair.of(new Translation2d(center.getX() - halfWidthMeters, center.getY() - halfWidthMeters),
					new Translation2d(center.getX() + halfWidthMeters, center.getY() + halfWidthMeters));
		}

		private Obstacles() {
		}
	}
}
