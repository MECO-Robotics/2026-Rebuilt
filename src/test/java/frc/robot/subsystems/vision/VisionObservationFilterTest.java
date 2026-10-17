package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.BeforeAll;

class VisionObservationFilterTest {
	private final int firstTag = VisionConstants.aprilTagLayout.getTags().get(0).ID;
	private final int secondTag = VisionConstants.aprilTagLayout.getTags().get(1).ID;

	@BeforeAll
	static void initializeHal() {
		HAL.initialize(500, 0);
	}

	@Test
	void acceptsConstrainedSingleTagMegaTag2() {
		PoseObservation observation = observation(pose(2.0, 2.0, 0.0), 0.1, 1, 3.0, PoseObservationType.MEGATAG_2,
				firstTag);

		assertTrue(VisionObservationFilter.isValid(observation,
				new Pose2d(2.2, 2.1, edu.wpi.first.math.geometry.Rotation2d.kZero)));
	}

	@Test
	void rejectsSingleTagMegaTag1AndUnsafeMegaTag2() {
		Pose2d reference = new Pose2d(2.0, 2.0, edu.wpi.first.math.geometry.Rotation2d.kZero);
		assertFalse(VisionObservationFilter.isValid(
				observation(pose(2.0, 2.0, 0.0), 0.1, 1, 2.0, PoseObservationType.MEGATAG_1, firstTag), reference));
		assertFalse(VisionObservationFilter.isValid(
				observation(pose(2.0, 2.0, 0.0), 0.1, 1, 4.0, PoseObservationType.MEGATAG_2, firstTag), reference));
		assertFalse(VisionObservationFilter.isValid(
				observation(pose(3.2, 2.0, 0.0), 0.1, 1, 2.0, PoseObservationType.MEGATAG_2, firstTag), reference));
	}

	@Test
	void acceptsMultiTagAndRejectsInvalidOrOutOfBoundsObservations() {
		Pose2d reference = new Pose2d(2.0, 2.0, edu.wpi.first.math.geometry.Rotation2d.kZero);
		assertTrue(VisionObservationFilter.isValid(
				observation(pose(2.0, 2.0, 0.0), 0.0, 2, 4.0, PoseObservationType.MEGATAG_1, firstTag, secondTag),
				reference));
		assertFalse(VisionObservationFilter.isValid(
				observation(pose(-0.1, 2.0, 0.0), 0.0, 2, 2.0, PoseObservationType.MEGATAG_1, firstTag, secondTag),
				reference));
		assertFalse(VisionObservationFilter.isValid(
				observation(pose(2.0, 2.0, 0.0), 0.0, 2, 2.0, PoseObservationType.MEGATAG_1, 999, secondTag),
				reference));
	}

	@Test
	void eachObservationUsesOnlyItsOwnTagIds() {
		Pose2d reference = new Pose2d(2.0, 2.0, edu.wpi.first.math.geometry.Rotation2d.kZero);
		PoseObservation firstFrame = observation(pose(2.0, 2.0, 0.0), 0.1, 1, 2.0, PoseObservationType.MEGATAG_2,
				firstTag);
		PoseObservation secondFrame = observation(pose(2.1, 2.0, 0.0), 0.1, 1, 2.0, PoseObservationType.MEGATAG_2,
				secondTag);

		assertTrue(VisionObservationFilter.isValid(firstFrame, reference));
		assertTrue(VisionObservationFilter.isValid(secondFrame, reference));
	}

	@Test
	void visibleTagsPersistAcrossBriefCameraFrameGapsAndThenExpire() {
		IntermittentTagVisionIO io = new IntermittentTagVisionIO(firstTag);
		Vision vision = new Vision((pose, timestamp, standardDeviations) -> {
		}, io);
		SimHooks.pauseTiming();
		try {
			vision.periodic();
			assertArrayEquals(new int[]{firstTag}, vision.getVisibleTagIds());

			io.publishTag = false;
			SimHooks.stepTiming(VisionConstants.visibleTagRetentionSeconds / 2.0);
			vision.periodic();
			assertArrayEquals(new int[]{firstTag}, vision.getVisibleTagIds());

			SimHooks.stepTiming(VisionConstants.visibleTagRetentionSeconds);
			vision.periodic();
			assertArrayEquals(new int[0], vision.getVisibleTagIds());
		} finally {
			CommandScheduler.getInstance().unregisterSubsystem(vision);
			SimHooks.resumeTiming();
		}
	}

	@Test
	void questNavSimSuppressesInertialPoseUntilAbsoluteAlignment() {
		FakeAbsoluteVision absolute = new FakeAbsoluteVision(firstTag, secondTag);
		VisionIOQuestNavSim quest = new VisionIOQuestNavSim(
				() -> new Pose2d(2.0, 2.0, edu.wpi.first.math.geometry.Rotation2d.kZero), absolute);
		VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();

		quest.updateInputs(inputs);
		assertFalse(inputs.poseInitialized);
		assertTrue(inputs.poseObservations.length == 0);

		absolute.publishObservation = true;
		quest.updateInputs(inputs);
		assertTrue(inputs.poseInitialized);
		assertTrue(inputs.poseObservations.length == 1);
		assertTrue(inputs.poseObservations[0].type() == PoseObservationType.QUESTNAV);
	}

	private static PoseObservation observation(Pose3d pose, double ambiguity, int tagCount, double distance,
			PoseObservationType type, int... tagIds) {
		return new PoseObservation(1.0, pose, ambiguity, tagCount, distance, type, tagIds);
	}

	private static Pose3d pose(double x, double y, double z) {
		return new Pose3d(x, y, z, new Rotation3d());
	}

	private static final class FakeAbsoluteVision implements VisionIO {
		private final int firstTag;
		private final int secondTag;
		boolean publishObservation;

		FakeAbsoluteVision(int firstTag, int secondTag) {
			this.firstTag = firstTag;
			this.secondTag = secondTag;
		}

		@Override
		public void updateInputs(VisionIOInputs inputs) {
			inputs.connected = true;
			inputs.tagIds = publishObservation ? new int[]{firstTag, secondTag} : new int[0];
			inputs.poseObservations = publishObservation
					? new PoseObservation[]{observation(pose(2.0, 2.0, 0.0), 0.0, 2, 2.0, PoseObservationType.MEGATAG_1,
							firstTag, secondTag)}
					: new PoseObservation[0];
		}
	}

	private static final class IntermittentTagVisionIO implements VisionIO {
		private final int tagId;
		boolean publishTag = true;

		IntermittentTagVisionIO(int tagId) {
			this.tagId = tagId;
		}

		@Override
		public void updateInputs(VisionIOInputs inputs) {
			inputs.connected = true;
			inputs.tagIds = publishTag ? new int[]{tagId} : new int[0];
			inputs.poseObservations = new PoseObservation[0];
		}
	}
}
