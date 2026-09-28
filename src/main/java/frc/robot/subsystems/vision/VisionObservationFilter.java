package frc.robot.subsystems.vision;

import static frc.robot.constants.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;

/** Shared, source-aware validation for absolute AprilTag pose observations. */
public final class VisionObservationFilter {
	private VisionObservationFilter() {
	}

	/** Returns whether an absolute observation is safe to apply to odometry. */
	public static boolean isValid(PoseObservation observation, int[] tagIds, Pose2d referencePose) {
		if (observation.type() == PoseObservationType.QUESTNAV || !isPoseInsideField(observation)) {
			return false;
		}
		for (int tagId : tagIds) {
			if (aprilTagLayout.getTagPose(tagId).isEmpty()) {
				return false;
			}
		}

		boolean singleTagMegaTag2 = observation.type() == PoseObservationType.MEGATAG_2 && observation.tagCount() == 1;
		if (singleTagMegaTag2) {
			return tagIds.length == 1 && observation.ambiguity() <= maxAmbiguity
					&& observation.averageTagDistance() <= maxSingleTagDistanceMeters
					&& observation.pose().toPose2d().getTranslation()
							.getDistance(referencePose.getTranslation()) <= maxSingleTagPoseDeltaMeters;
		}

		return observation.tagCount() >= minTagCountForOdometry && tagIds.length >= minTagCountForOdometry;
	}

	/** Returns whether the pose is physically plausible and inside the field. */
	public static boolean isPoseInsideField(PoseObservation observation) {
		return Math.abs(observation.pose().getZ()) <= maxZError && observation.pose().getX() >= 0.0
				&& observation.pose().getX() <= aprilTagLayout.getFieldLength() && observation.pose().getY() >= 0.0
				&& observation.pose().getY() <= aprilTagLayout.getFieldWidth();
	}
}
