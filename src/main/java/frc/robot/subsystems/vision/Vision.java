package frc.robot.subsystems.vision;

import static frc.robot.constants.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Aggregates camera inputs, filters observations, and feeds accepted poses to
 * drivetrain odometry.
 */
public class Vision extends SubsystemBase {
	private final VisionConsumer consumer;
	private final VisionIO[] io;
	private final VisionIOInputsAutoLogged[] inputs;
	private final Alert[] disconnectedAlerts;
	private final Supplier<Pose2d> referencePoseSupplier;
	private boolean poseReady;

	/**
	 * Creates the vision subsystem.
	 *
	 * @param consumer
	 *            callback for accepted vision measurements
	 * @param io
	 *            one or more camera IO implementations
	 */
	public Vision(VisionConsumer consumer, VisionIO... io) {
		this(consumer, Pose2d::new, io);
	}

	/**
	 * Creates vision filtering with a current-pose reference for single-tag gates.
	 */
	public Vision(VisionConsumer consumer, Supplier<Pose2d> referencePoseSupplier, VisionIO... io) {
		this.consumer = consumer;
		this.referencePoseSupplier = referencePoseSupplier;
		this.io = io;

		// Initialize inputs
		this.inputs = new VisionIOInputsAutoLogged[io.length];
		for (int i = 0; i < inputs.length; i++) {
			inputs[i] = new VisionIOInputsAutoLogged();
		}

		// Initialize disconnected alerts
		this.disconnectedAlerts = new Alert[io.length];
		for (int i = 0; i < inputs.length; i++) {
			disconnectedAlerts[i] = new Alert("Vision camera " + Integer.toString(i) + " is disconnected.",
					AlertType.kWarning);
		}
	}

	/** Returns true once at least one configured pose source is field aligned. */
	public boolean isPoseReady() {
		return poseReady;
	}

	/**
	 * Returns the X angle to the best target, which can be used for simple servoing
	 * with vision.
	 *
	 * @param cameraIndex
	 *            The index of the camera to use.
	 */
	public Rotation2d getTargetX(int cameraIndex) {
		return inputs[cameraIndex].latestTargetObservation.tx();
	}

	@Override
	public void periodic() {
		for (int i = 0; i < io.length; i++) {
			io[i].updateInputs(inputs[i]);
			Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
		}

		// Initialize logging values
		List<Pose3d> allTagPoses = new LinkedList<>();
		List<Pose3d> allRobotPoses = new LinkedList<>();
		List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
		List<Pose3d> allRobotPosesRejected = new LinkedList<>();
		poseReady = false;

		// Loop over cameras
		for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
			// Update disconnected alert
			disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

			// Initialize logging values
			List<Pose3d> tagPoses = new LinkedList<>();
			List<Pose3d> robotPoses = new LinkedList<>();
			List<Pose3d> robotPosesAccepted = new LinkedList<>();
			List<Pose3d> robotPosesRejected = new LinkedList<>();
			// Add tag poses
			for (int tagId : inputs[cameraIndex].tagIds) {
				var tagPose = aprilTagLayout.getTagPose(tagId);
				if (tagPose.isPresent()) {
					tagPoses.add(tagPose.get());
				}
			}
			poseReady |= inputs[cameraIndex].poseInitialized;

			// Loop over pose observations
			for (var observation : inputs[cameraIndex].poseObservations) {
				boolean isQuestNav = observation.type() == PoseObservationType.QUESTNAV;
				boolean rejectPose = isQuestNav
						? !inputs[cameraIndex].poseInitialized
								|| !VisionObservationFilter.isPoseInsideField(observation)
						: !VisionObservationFilter.isValid(observation, inputs[cameraIndex].tagIds,
								referencePoseSupplier.get());

				// Add pose to log
				robotPoses.add(observation.pose());
				if (rejectPose) {
					robotPosesRejected.add(observation.pose());
				} else {
					robotPosesAccepted.add(observation.pose());
				}

				// Skip if rejected
				if (rejectPose) {
					continue;
				}

				// Calculate standard deviations
				double linearStdDev;
				double angularStdDev;
				if (isQuestNav) {
					linearStdDev = linearStdDevBaseline;
					angularStdDev = angularStdDevBaseline;
				} else {
					double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
					linearStdDev = linearStdDevBaseline * stdDevFactor;
					angularStdDev = angularStdDevBaseline * stdDevFactor;
				}
				if (observation.type() == PoseObservationType.MEGATAG_2) {
					linearStdDev *= linearStdDevMegatag2Factor;
					angularStdDev *= angularStdDevMegatag2Factor;
				}
				if (cameraIndex < cameraStdDevFactors.length) {
					linearStdDev *= cameraStdDevFactors[cameraIndex];
					angularStdDev *= cameraStdDevFactors[cameraIndex];
				}

				// Send vision observation (optionally flip QuestNav about field center)
				Pose2d visionPose2d = observation.pose().toPose2d();
				consumer.accept(visionPose2d, observation.timestamp(),
						VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
			}

			// Log camera metadata
			Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
					tagPoses.toArray(new Pose3d[0]));
			Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
					robotPoses.toArray(new Pose3d[0]));
			Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
					robotPosesAccepted.toArray(new Pose3d[0]));
			Logger.recordOutput("Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
					robotPosesRejected.toArray(new Pose3d[0]));
			allTagPoses.addAll(tagPoses);
			allRobotPoses.addAll(robotPoses);
			allRobotPosesAccepted.addAll(robotPosesAccepted);
			allRobotPosesRejected.addAll(robotPosesRejected);
		}

		// Log summary data
		Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
		Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
		Logger.recordOutput("Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
		Logger.recordOutput("Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
		Logger.recordOutput("Vision/PoseReady", poseReady);
	}

	@FunctionalInterface
	/**
	 * Callback used to hand accepted vision observations to consumers (typically
	 * drivetrain).
	 */
	public static interface VisionConsumer {
		public void accept(Pose2d visionRobotPoseMeters, double timestampSeconds,
				Matrix<N3, N1> visionMeasurementStdDevs);
	}
}
