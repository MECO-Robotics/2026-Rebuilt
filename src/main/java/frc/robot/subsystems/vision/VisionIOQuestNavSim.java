package frc.robot.subsystems.vision;

import static frc.robot.constants.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import java.util.Arrays;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Simulation implementation of QuestNav as a drift-prone inertial estimator.
 */
public class VisionIOQuestNavSim implements VisionIO {
	private final Supplier<Pose2d> groundTruthPoseSupplier;
	private final VisionIO absoluteVisionIO;
	private final VisionIOInputsAutoLogged absoluteInputs = new VisionIOInputsAutoLogged();

	private Pose2d inertialPose = null;
	private Pose2d lastGroundTruthPose = null;
	private double lastTimestampSeconds = Timer.getFPGATimestamp();
	private boolean fieldAligned = false;
	private double lastAbsoluteObservationTimestamp = -1.0;

	public VisionIOQuestNavSim(Supplier<Pose2d> groundTruthPoseSupplier, VisionIO absoluteVisionIO) {
		this.groundTruthPoseSupplier = groundTruthPoseSupplier;
		this.absoluteVisionIO = absoluteVisionIO;
	}

	@Override
	public void updateInputs(VisionIOInputs inputs) {
		absoluteVisionIO.updateInputs(absoluteInputs);
		Logger.processInputs("QuestNav/absolute", absoluteInputs);
		PoseObservation[] filteredAbsoluteObservations = filterAbsoluteObservations(absoluteInputs);
		Arrays.stream(filteredAbsoluteObservations).mapToDouble(PoseObservation::timestamp).max()
				.ifPresent(timestamp -> lastAbsoluteObservationTimestamp = timestamp);

		double nowSeconds = Timer.getFPGATimestamp();
		double dtSeconds = Math.max(0.0, nowSeconds - lastTimestampSeconds);
		lastTimestampSeconds = nowSeconds;

		Pose2d groundTruthPose = groundTruthPoseSupplier.get();
		if (inertialPose == null || lastGroundTruthPose == null) {
			inertialPose = groundTruthPose;
			lastGroundTruthPose = groundTruthPose;
		}

		// Apply relative motion to mimic inertial integration instead of using
		// absolute field pose directly.
		Pose2d relativeMotion = groundTruthPose.relativeTo(lastGroundTruthPose);
		inertialPose = inertialPose.plus(new edu.wpi.first.math.geometry.Transform2d(relativeMotion.getTranslation(),
				relativeMotion.getRotation()));

		// Apply tunable random-walk noise and constant bias drift.
		double noiseScale = Math.sqrt(dtSeconds);
		double x = inertialPose.getX() + questNavSimTranslationDriftXMetersPerSec * dtSeconds
				+ (Math.random() * 2.0 - 1.0) * questNavSimTranslationNoiseStdDevMetersPerSqrtSec * noiseScale;
		double y = inertialPose.getY() + questNavSimTranslationDriftYMetersPerSec * dtSeconds
				+ (Math.random() * 2.0 - 1.0) * questNavSimTranslationNoiseStdDevMetersPerSqrtSec * noiseScale;
		Rotation2d yaw = inertialPose.getRotation().plus(new Rotation2d(questNavSimYawDriftRadPerSec * dtSeconds
				+ (Math.random() * 2.0 - 1.0) * questNavSimYawNoiseStdDevRadPerSqrtSec * noiseScale));
		inertialPose = new Pose2d(x, y, yaw);
		lastGroundTruthPose = groundTruthPose;

		if (questNavSimEnableAbsoluteCorrection && filteredAbsoluteObservations.length > 0) {
			Pose2d absolutePose = filteredAbsoluteObservations[0].pose().toPose2d();
			Translation2d correctedTranslation = new Translation2d(
					inertialPose.getX()
							+ (absolutePose.getX() - inertialPose.getX()) * questNavSimTranslationCorrectionAlpha,
					inertialPose.getY()
							+ (absolutePose.getY() - inertialPose.getY()) * questNavSimTranslationCorrectionAlpha);
			Rotation2d correctedYaw = inertialPose.getRotation().interpolate(absolutePose.getRotation(),
					questNavSimYawCorrectionAlpha);
			inertialPose = new Pose2d(correctedTranslation, correctedYaw);
			fieldAligned = true;
		}

		inputs.connected = true;
		inputs.inertialConnected = true;
		inputs.absoluteConnected = absoluteInputs.connected;
		inputs.latestTargetObservation = new TargetObservation(Rotation2d.kZero, Rotation2d.kZero, 0);
		inputs.poseInitialized = fieldAligned;
		inputs.lastAbsoluteObservationTimestamp = lastAbsoluteObservationTimestamp;
		inputs.poseObservations = fieldAligned
				? new PoseObservation[]{new PoseObservation(nowSeconds, new Pose3d(inertialPose), 0.0, -1, 0.0,
						PoseObservationType.QUESTNAV)}
				: filteredAbsoluteObservations;
		inputs.tagIds = absoluteInputs.tagIds.clone();

		Logger.recordOutput("QuestNav/Sim/InertialPose", new Pose3d(inertialPose));
		Logger.recordOutput("QuestNav/Sim/FieldAligned", fieldAligned);
	}

	private PoseObservation[] filterAbsoluteObservations(VisionIOInputs absoluteInputs) {
		Pose2d referencePose = inertialPose != null ? inertialPose : groundTruthPoseSupplier.get();
		return Arrays.stream(absoluteInputs.poseObservations).filter(
				observation -> VisionObservationFilter.isValid(observation, absoluteInputs.tagIds, referencePose))
				.toArray(PoseObservation[]::new);
	}
}
