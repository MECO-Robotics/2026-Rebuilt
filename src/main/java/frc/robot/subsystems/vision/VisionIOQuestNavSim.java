package frc.robot.subsystems.vision;

import static frc.robot.constants.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
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

	public VisionIOQuestNavSim(Supplier<Pose2d> groundTruthPoseSupplier, VisionIO absoluteVisionIO) {
		this.groundTruthPoseSupplier = groundTruthPoseSupplier;
		this.absoluteVisionIO = absoluteVisionIO;
	}

	@Override
	public void updateInputs(VisionIOInputs inputs) {
		absoluteVisionIO.updateInputs(absoluteInputs);
		Logger.processInputs("QuestNav/absolute", absoluteInputs);

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

		if (questNavSimEnableAbsoluteCorrection && absoluteInputs.poseObservations.length > 0) {
			Pose2d absolutePose = absoluteInputs.poseObservations[0].pose().toPose2d();
			Translation2d correctedTranslation = new Translation2d(
					inertialPose.getX()
							+ (absolutePose.getX() - inertialPose.getX()) * questNavSimTranslationCorrectionAlpha,
					inertialPose.getY()
							+ (absolutePose.getY() - inertialPose.getY()) * questNavSimTranslationCorrectionAlpha);
			Rotation2d correctedYaw = inertialPose.getRotation().interpolate(absolutePose.getRotation(),
					questNavSimYawCorrectionAlpha);
			inertialPose = new Pose2d(correctedTranslation, correctedYaw);
		}

		inputs.connected = true;
		inputs.latestTargetObservation = new TargetObservation(Rotation2d.kZero, Rotation2d.kZero, 0);
		inputs.poseObservations = new PoseObservation[]{
				new PoseObservation(nowSeconds, new Pose3d(inertialPose), 0.0, -1, 0.0, PoseObservationType.QUESTNAV)};
		inputs.tagIds = new int[0];

		Logger.recordOutput("QuestNav/Sim/InertialPose", new Pose3d(inertialPose));
	}
}
