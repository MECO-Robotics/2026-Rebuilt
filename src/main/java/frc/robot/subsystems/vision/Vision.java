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
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
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
	private final String[] cameraLogKeys;
	private final String[] cameraObservedWhitelistLogKeys;
	private final String[] cameraTagPosesLogKeys;
	private final String[] cameraRobotPosesLogKeys;
	private final String[] cameraRobotPosesAcceptedLogKeys;
	private final String[] cameraRobotPosesRejectedLogKeys;

	/**
	 * Creates the vision subsystem.
	 *
	 * @param consumer
	 *            callback for accepted vision measurements
	 * @param io
	 *            one or more camera IO implementations
	 */
	public Vision(VisionConsumer consumer, VisionIO... io) {
		this.consumer = consumer;
		this.io = io;

		// Initialize inputs
		this.inputs = new VisionIOInputsAutoLogged[io.length];
		for (int i = 0; i < inputs.length; i++) {
			inputs[i] = new VisionIOInputsAutoLogged();
		}

		// Initialize disconnected alerts and per-camera log keys
		this.disconnectedAlerts = new Alert[io.length];
		this.cameraLogKeys = new String[io.length];
		this.cameraObservedWhitelistLogKeys = new String[io.length];
		this.cameraTagPosesLogKeys = new String[io.length];
		this.cameraRobotPosesLogKeys = new String[io.length];
		this.cameraRobotPosesAcceptedLogKeys = new String[io.length];
		this.cameraRobotPosesRejectedLogKeys = new String[io.length];
		for (int i = 0; i < inputs.length; i++) {
			disconnectedAlerts[i] = new Alert("Vision camera " + Integer.toString(i) + " is disconnected.",
					AlertType.kWarning);
			String cameraBaseKey = "Vision/Camera" + i;
			cameraLogKeys[i] = cameraBaseKey;
			cameraObservedWhitelistLogKeys[i] = cameraBaseKey + "/ObservedWhitelistedTagCount";
			cameraTagPosesLogKeys[i] = cameraBaseKey + "/TagPoses";
			cameraRobotPosesLogKeys[i] = cameraBaseKey + "/RobotPoses";
			cameraRobotPosesAcceptedLogKeys[i] = cameraBaseKey + "/RobotPosesAccepted";
			cameraRobotPosesRejectedLogKeys[i] = cameraBaseKey + "/RobotPosesRejected";
		}
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
		Set<Integer> whitelistedTagIds = getOdometryTagWhitelistForCurrentAlliance();

		for (int i = 0; i < io.length; i++) {
			io[i].updateInputs(inputs[i]);
			Logger.processInputs(cameraLogKeys[i], inputs[i]);
		}

		// Initialize logging values
		List<Pose3d> allTagPoses = new ArrayList<>();
		List<Pose3d> allRobotPoses = new ArrayList<>();
		List<Pose3d> allRobotPosesAccepted = new ArrayList<>();
		List<Pose3d> allRobotPosesRejected = new ArrayList<>();

		// Loop over cameras
		for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
			// Update disconnected alert
			disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

			// Initialize logging values
			List<Pose3d> tagPoses = new ArrayList<>(inputs[cameraIndex].tagIds.length);
			List<Pose3d> robotPoses = new ArrayList<>(inputs[cameraIndex].poseObservations.length);
			List<Pose3d> robotPosesAccepted = new ArrayList<>(inputs[cameraIndex].poseObservations.length);
			List<Pose3d> robotPosesRejected = new ArrayList<>(inputs[cameraIndex].poseObservations.length);
			int observedWhitelistedTagCount = 0;

			// Add tag poses
			for (int tagId : inputs[cameraIndex].tagIds) {
				if (whitelistedTagIds.isEmpty() || whitelistedTagIds.contains(tagId)) {
					var tagPose = aprilTagLayout.getTagPose(tagId);
					if (tagPose.isPresent()) {
						tagPoses.add(tagPose.get());
					}
					observedWhitelistedTagCount++;
				}
			}
			boolean hasEnoughWhitelistedTags = observedWhitelistedTagCount >= minWhitelistedTagCountForOdometry;
			Logger.recordOutput(cameraObservedWhitelistLogKeys[cameraIndex], observedWhitelistedTagCount);

			// Loop over pose observations
			for (var observation : inputs[cameraIndex].poseObservations) {
				boolean isQuestNav = observation.type() == PoseObservationType.QUESTNAV;
				boolean enforceWhitelistedTagMinimum = !isQuestNav && !whitelistedTagIds.isEmpty()
						&& minWhitelistedTagCountForOdometry > 0;
				// Check whether to reject pose
				boolean rejectPose = (!isQuestNav && observation.tagCount() < minTagCountForOdometry) // Must have
																										// enough tags
						|| (!isQuestNav && observation.tagCount() == 1) // && observation.ambiguity() > maxAmbiguity)
						// Single-tag solve must not be too ambiguous
						|| (enforceWhitelistedTagMinimum && !hasEnoughWhitelistedTags) // Must include enough
																						// currently-whitelisted tags
						|| Math.abs(observation.pose().getZ()) > maxZError // Must have realistic Z coordinate

						// Must be within the field boundaries
						|| observation.pose().getX() < 0.0
						|| observation.pose().getX() > aprilTagLayout.getFieldLength()
						|| observation.pose().getY() < 0.0
						|| observation.pose().getY() > aprilTagLayout.getFieldWidth();

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
			Logger.recordOutput(cameraTagPosesLogKeys[cameraIndex], tagPoses.toArray(Pose3d[]::new));
			Logger.recordOutput(cameraRobotPosesLogKeys[cameraIndex], robotPoses.toArray(Pose3d[]::new));
			Logger.recordOutput(cameraRobotPosesAcceptedLogKeys[cameraIndex], robotPosesAccepted.toArray(Pose3d[]::new));
			Logger.recordOutput(cameraRobotPosesRejectedLogKeys[cameraIndex], robotPosesRejected.toArray(Pose3d[]::new));
			allTagPoses.addAll(tagPoses);
			allRobotPoses.addAll(robotPoses);
			allRobotPosesAccepted.addAll(robotPosesAccepted);
			allRobotPosesRejected.addAll(robotPosesRejected);
		}

		// Log summary data
		Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(Pose3d[]::new));
		Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(Pose3d[]::new));
		Logger.recordOutput("Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(Pose3d[]::new));
		Logger.recordOutput("Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(Pose3d[]::new));
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
