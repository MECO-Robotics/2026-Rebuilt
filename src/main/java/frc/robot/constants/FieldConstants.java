package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.vision.VisionConstants;
import java.util.Optional;

/**
 * Contains various field dimensions and useful reference points. All units are
 * in meters
 */
public class FieldConstants {
	/** Field-relative boundaries for the two alliance zones. */
	public static class AllianceZone {
		public static final double DEPTH_METERS = Units.inchesToMeters(158.6);

		private AllianceZone() {
		}

		/** Returns the neutral-zone-facing X boundary for an alliance zone. */
		public static double boundaryX(Alliance alliance) {
			return alliance == Alliance.Blue
					? DEPTH_METERS
					: VisionConstants.aprilTagLayout.getFieldLength() - DEPTH_METERS;
		}

		/** Returns whether a field pose's center is inside its own alliance zone. */
		public static boolean contains(Alliance alliance, Translation2d position) {
			return alliance == Alliance.Blue
					? position.getX() <= boundaryX(alliance)
					: position.getX() >= boundaryX(alliance);
		}
	}

	public static class Hub {
		/** Returns the hub center for an explicit alliance. */
		public static Translation2d hubPosition(Alliance alliance) {
			if (alliance == Alliance.Blue) {
				return new Translation2d(Units.inchesToMeters(182.105), Units.inchesToMeters(158.845));
			}
			return new Translation2d(Units.inchesToMeters(469.115), Units.inchesToMeters(158.845));
		}

		/** Returns the hub center for the current Driver Station alliance. */
		public static Translation2d hubPosition() {
			final Optional<Alliance> alliance = DriverStation.getAlliance();
			return hubPosition(alliance.orElse(Alliance.Red));
		}

		// measured from floor to top of funnel
		public static final double hubHeight = Units.inchesToMeters(72);
	}

	/** Official trench envelope and alliance-specific field locations. */
	public static class Trench {
		public static final double WIDTH_METERS = Units.inchesToMeters(65.65);
		public static final double DEPTH_METERS = Units.inchesToMeters(47.0);
		public static final double HEIGHT_METERS = Units.inchesToMeters(40.25);
		public static final double FUEL_RADIUS_METERS = Units.inchesToMeters(5.91 / 2.0);

		private Trench() {
		}

		/** Returns the field X center of the alliance's trench reference pair. */
		public static double centerX(Alliance alliance) {
			int firstTag = alliance == Alliance.Blue ? 22 : 6;
			int secondTag = alliance == Alliance.Blue ? 23 : 7;
			double firstX = VisionConstants.aprilTagLayout.getTagPose(firstTag).orElseThrow().getX();
			double secondX = VisionConstants.aprilTagLayout.getTagPose(secondTag).orElseThrow().getX();
			return (firstX + secondX) / 2.0;
		}

		/** Returns the trench edge closest to the neutral-zone field center. */
		public static double neutralEdgeX(Alliance alliance) {
			double directionTowardFieldCenter = alliance == Alliance.Blue ? 1.0 : -1.0;
			return centerX(alliance) + directionTowardFieldCenter * DEPTH_METERS / 2.0;
		}

		/** Returns the trench edge closest to the alliance wall. */
		public static double allianceEdgeX(Alliance alliance) {
			double directionTowardAllianceWall = alliance == Alliance.Blue ? -1.0 : 1.0;
			return centerX(alliance) + directionTowardAllianceWall * DEPTH_METERS / 2.0;
		}
	}
}
