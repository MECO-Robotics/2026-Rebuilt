package frc.robot.subsystems.vision;

import frc.robot.constants.Constants;
import frc.robot.subsystems.vision.VisionIOLimelight.LimelightPoseMode;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.AutoLog;

/** Hardware abstraction for vision cameras and pose-estimation pipelines. */
public interface VisionIO {
	/** Logged inputs shared by all vision implementations. */
	@AutoLog
	public static class VisionIOInputs {
		/** True when the camera/pipeline is connected and publishing data. */
		public boolean connected = false;
		/** Latest simple target observation (tx/ty) for servo use cases. */
		public TargetObservation latestTargetObservation = new TargetObservation(Rotation2d.kZero, Rotation2d.kZero, 0);
		/** Pose observations produced this cycle. */
		public PoseObservation[] poseObservations = new PoseObservation[0];
		/** Tag IDs observed this cycle. */
		public int[] tagIds = new int[0];
	}

	/** Represents the angle to a simple target, not used for pose estimation. */
	public static record TargetObservation(Rotation2d tx, Rotation2d ty, int tagID) {
	}

	/** Represents a robot pose sample used for pose estimation. */
	public static record PoseObservation(double timestamp, Pose3d pose, double ambiguity, int tagCount,
			double averageTagDistance, PoseObservationType type) {
	}

	public static enum PoseObservationType {
		MEGATAG_1, MEGATAG_2, PHOTONVISION, PHOTONVISIONTRIG, QUESTNAV
	}

	/** Refreshes all camera and estimation inputs. */
	public default void updateInputs(VisionIOInputs inputs) {
	}

	/** Creates a replay vision IO supplier. */
	public static Supplier<VisionIO> replayFactory() {
		return () -> new VisionIO() {
		};
	}

	/**
	 * Creates a mode-appropriate vision IO.
	 *
	 * <p>
	 * Returns the supplied real implementation on real hardware, supplied sim
	 * implementation in sim, and replay IO during log replay.
	 */
	public static VisionIO fromMode(Supplier<VisionIO> realSupplier, Supplier<VisionIO> simSupplier) {
		return switch (Constants.currentMode) {
			case REAL -> realSupplier.get();
			case SIM -> simSupplier.get();
			default -> replayFactory().get();
		};
	}

	/**
	 * Creates mode-appropriate Limelight IO using MegaTag 1.
	 *
	 * <p>
	 * Real mode uses Limelight hardware. Sim and replay return inert/no-op vision
	 * IO.
	 */
	public static VisionIO limelightMegatag1(String limelightName, Transform3d robotToCamera) {
		return fromMode(() -> new VisionIOLimelight(limelightName, robotToCamera), replayFactory());
	}

	/**
	 * Creates mode-appropriate Limelight IO using MegaTag 2.
	 *
	 * <p>
	 * Real mode uses Limelight hardware. Sim and replay return inert/no-op vision
	 * IO.
	 */
	public static VisionIO limelightMegatag2(String limelightName, Transform3d robotToCamera,
			Supplier<Rotation2d> rotationSupplier) {
		return fromMode(() -> new VisionIOLimelight(limelightName, robotToCamera, rotationSupplier), replayFactory());
	}

	/**
	 * Creates mode-appropriate QuestNav + PhotonVision composition.
	 *
	 * <p>
	 * Real mode uses hardware QuestNav anchored by PhotonVision. Sim mode uses a
	 * QuestNav inertial simulator anchored by PhotonVision simulation.
	 */
	public static VisionIO questNavWithPhoton(String photonCameraName, Transform3d robotToQuest,
			Transform3d robotToPhotonCamera, Supplier<Pose2d> simPoseSupplier) {
		return fromMode(
				() -> new VisionIOQuestNav(robotToQuest,
						new VisionIOPhotonVision(photonCameraName, robotToPhotonCamera)),
				() -> new VisionIOQuestNavSim(simPoseSupplier,
						new VisionIOPhotonVisionSim(photonCameraName, robotToPhotonCamera, simPoseSupplier)));
	}

	/**
	 * Creates mode-appropriate Limelight vision IO.
	 *
	 * <p>
	 * Real mode uses Limelight NetworkTables data. Sim mode uses PhotonVision's
	 * simulator with Limelight-style observations.
	 */
	public static VisionIO limelightWithSim(String limelightName, Supplier<Rotation2d> rotationSupplier,
			Transform3d robotToLimelight, Supplier<Pose2d> simPoseSupplier) {
		return fromMode(
				() -> new VisionIOLimelight(limelightName, robotToLimelight, rotationSupplier, LimelightPoseMode.BOTH),
				() -> new VisionIOLimelightSim(limelightName, robotToLimelight, simPoseSupplier));
	}

	/**
	 * Creates mode-appropriate Limelight MegaTag1-only vision IO.
	 *
	 * <p>
	 * Real mode consumes only Limelight's MegaTag1 stream. Sim mode reuses the
	 * Limelight simulator, which already produces MegaTag1-style observations.
	 */
	public static VisionIO limelightMegaTag1WithSim(String limelightName, Transform3d robotToLimelight,
			Supplier<Pose2d> simPoseSupplier) {
		return fromMode(() -> new VisionIOLimelight(limelightName, robotToLimelight),
				() -> new VisionIOLimelightSim(limelightName, robotToLimelight, simPoseSupplier));
	}

	/**
	 * Creates mode-appropriate Limelight IO that uses MegaTag1 normally and
	 * MegaTag2 only when exactly one tag is visible.
	 *
	 * <p>
	 * Real mode reads both Limelight streams and gates them by tag count. Sim mode
	 * reuses the Limelight simulator.
	 */
	public static VisionIO limelightMegaTag1WithMegaTag2SingleTagWithSim(String limelightName,
			Supplier<Rotation2d> rotationSupplier, Transform3d robotToLimelight, Supplier<Pose2d> simPoseSupplier) {
		return fromMode(
				() -> new VisionIOLimelight(limelightName, robotToLimelight, rotationSupplier,
						LimelightPoseMode.MEGATAG_1_WITH_MEGATAG_2_SINGLE_TAG),
				() -> new VisionIOLimelightSim(limelightName, robotToLimelight, simPoseSupplier));
	}
}
