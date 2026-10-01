package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.FieldConstants.AllianceZone;
import frc.robot.constants.FieldConstants.Hub;
import frc.robot.constants.vision.VisionConstants;
import java.util.Arrays;

/**
 * Pure alliance-, pose-, and tag-based target selection for the driver-held
 * shot command.
 */
public final class ShotTargetSelector {
	private static final int RED_FIRST_TAG_ID = 1;
	private static final int RED_LAST_TAG_ID = 16;
	private static final int BLUE_FIRST_TAG_ID = 17;
	private static final int BLUE_LAST_TAG_ID = 32;
	private static final int RED_FERRY_TAG_A = 6;
	private static final int RED_FERRY_TAG_B = 7;
	private static final int BLUE_FERRY_TAG_A = 22;
	private static final int BLUE_FERRY_TAG_B = 23;
	private static final double FERRY_TARGET_ALLIANCE_ZONE_INSET_METERS = 0.90;
	private static final double FERRY_TARGET_GUARDRAIL_INSET_METERS = 0.40;

	private ShotTargetSelector() {
	}

	/**
	 * Chooses the hub while the robot is in its own alliance zone or when any tag
	 * on its alliance side is visible. Otherwise, chooses a safe ferry landing
	 * point inside the alliance zone.
	 */
	public static ShotTarget select(Alliance alliance, Pose2d robotPose, int[] visibleTagIds) {
		return AllianceZone.contains(alliance, robotPose.getTranslation())
				|| seesAllianceSideTag(alliance, visibleTagIds) ? hub(alliance) : ferry(alliance, robotPose);
	}

	/** Returns a hub target without applying the visibility rule. */
	public static ShotTarget hub(Alliance alliance) {
		return new ShotTarget(ShotTarget.Mode.HUB, Hub.hubPosition(alliance));
	}

	/**
	 * Returns the guardrail-inset landing point on the same field half as the
	 * robot. Tags 6/7 or 22/23 provide one trench-side lane reference; the second
	 * point is its mirror across the field centerline.
	 */
	public static ShotTarget ferry(Alliance alliance, Pose2d robotPose) {
		int firstTag = alliance == Alliance.Blue ? BLUE_FERRY_TAG_A : RED_FERRY_TAG_A;
		int secondTag = alliance == Alliance.Blue ? BLUE_FERRY_TAG_B : RED_FERRY_TAG_B;
		Translation2d trenchReference = tagPosition(firstTag).interpolate(tagPosition(secondTag), 0.5);
		double directionIntoAllianceZone = alliance == Alliance.Blue ? -1.0 : 1.0;
		double directionAwayFromGuardrail = alliance == Alliance.Blue ? -1.0 : 1.0;
		Translation2d referenceSideTarget = new Translation2d(
				AllianceZone.boundaryX(alliance) + directionIntoAllianceZone * FERRY_TARGET_ALLIANCE_ZONE_INSET_METERS,
				trenchReference.getY() + directionAwayFromGuardrail * FERRY_TARGET_GUARDRAIL_INSET_METERS);
		Translation2d oppositeSideTarget = new Translation2d(referenceSideTarget.getX(),
				VisionConstants.aprilTagLayout.getFieldWidth() - referenceSideTarget.getY());
		double fieldCenterY = VisionConstants.aprilTagLayout.getFieldWidth() / 2.0;
		Translation2d lowerFieldTarget = referenceSideTarget.getY() <= fieldCenterY
				? referenceSideTarget
				: oppositeSideTarget;
		Translation2d upperFieldTarget = referenceSideTarget.getY() > fieldCenterY
				? referenceSideTarget
				: oppositeSideTarget;
		Translation2d selectedTarget = robotPose.getY() <= fieldCenterY ? lowerFieldTarget : upperFieldTarget;
		return new ShotTarget(ShotTarget.Mode.FERRY, selectedTarget);
	}

	/** Returns whether at least one visible tag belongs to the alliance side. */
	public static boolean seesAllianceSideTag(Alliance alliance, int[] visibleTagIds) {
		int firstId = alliance == Alliance.Blue ? BLUE_FIRST_TAG_ID : RED_FIRST_TAG_ID;
		int lastId = alliance == Alliance.Blue ? BLUE_LAST_TAG_ID : RED_LAST_TAG_ID;
		return visibleTagIds != null
				&& Arrays.stream(visibleTagIds).anyMatch(tagId -> tagId >= firstId && tagId <= lastId);
	}

	private static Translation2d tagPosition(int tagId) {
		return VisionConstants.aprilTagLayout.getTagPose(tagId)
				.orElseThrow(() -> new IllegalStateException("AprilTag " + tagId + " is missing from the field layout"))
				.getTranslation().toTranslation2d();
	}
}
