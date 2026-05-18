package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import java.util.Arrays;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Objects;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** IO implementation for real Limelight hardware. */
public class VisionIOLimelight implements VisionIO {
	public static enum LimelightPoseMode {
		MEGATAG_1_ONLY, MEGATAG_2_ONLY, BOTH, MEGATAG_1_WITH_MEGATAG_2_SINGLE_TAG
	}

	private final String name;
	private final Transform3d robotToCamera;
	private final Supplier<Rotation2d> rotationSupplier;
	private final LimelightPoseMode poseMode;
	private final boolean useMegatag1;
	private final boolean useMegatag2;
	private final DoubleArrayPublisher orientationPublisher;
	private final DoubleArrayPublisher cameraPosePublisher;
	private double[] lastPublishedCameraPose = null;

	private final DoubleSubscriber heartbeatSubscriber;
	private final DoubleSubscriber txSubscriber;
	private final DoubleSubscriber tySubscriber;
	private final DoubleSubscriber tidSubscriber;
	private final DoubleArraySubscriber megatag1Subscriber;
	private final DoubleArraySubscriber megatag2Subscriber;

	/**
	 * Creates a new VisionIOLimelight using MegaTag 1 observations only.
	 *
	 * @param name
	 *            The configured name of the Limelight.
	 * @param robotToCamera
	 *            The 3D position of the camera relative to the robot.
	 */
	public VisionIOLimelight(String name, Transform3d robotToCamera) {
		this(name, robotToCamera, null, LimelightPoseMode.MEGATAG_1_ONLY);
	}

	/**
	 * Creates a new VisionIOLimelight using MegaTag 2 observations only.
	 *
	 * @param name
	 *            The configured name of the Limelight.
	 * @param robotToCamera
	 *            The 3D position of the camera relative to the robot.
	 * @param rotationSupplier
	 *            Supplier for the current estimated rotation, used for MegaTag 2.
	 */
	public VisionIOLimelight(String name, Transform3d robotToCamera, Supplier<Rotation2d> rotationSupplier) {
		this(name, robotToCamera, rotationSupplier, LimelightPoseMode.MEGATAG_2_ONLY);
	}

	/**
	 * Creates a new VisionIOLimelight.
	 *
	 * @param name
	 *            The configured name of the Limelight.
	 * @param robotToCamera
	 *            The 3D position of the camera relative to the robot.
	 * @param rotationSupplier
	 *            Supplier for the current estimated rotation, used for MegaTag 2
	 *            orientation sync.
	 * @param poseMode
	 *            Which Limelight pose stream(s) to consume.
	 */
	public VisionIOLimelight(String name, Transform3d robotToCamera, Supplier<Rotation2d> rotationSupplier,
			LimelightPoseMode poseMode) {
		var table = NetworkTableInstance.getDefault().getTable(name);
		this.name = name;
		this.robotToCamera = robotToCamera;
		this.rotationSupplier = rotationSupplier;
		this.poseMode = poseMode;
		this.useMegatag1 = poseMode != LimelightPoseMode.MEGATAG_2_ONLY;
		this.useMegatag2 = poseMode != LimelightPoseMode.MEGATAG_1_ONLY;
		orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
		cameraPosePublisher = table.getDoubleArrayTopic("camerapose_robotspace_set").publish();
		heartbeatSubscriber = table.getDoubleTopic("hb").subscribe(0.0);
		txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
		tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
		tidSubscriber = table.getDoubleTopic("tid").subscribe(0.0);
		megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[]{});
		megatag2Subscriber = table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[]{});

		if (useMegatag2) {
			Objects.requireNonNull(rotationSupplier, "MegaTag 2 requires a robot rotation supplier.");
		}
	}

	@Override
	public void updateInputs(VisionIOInputs inputs) {
		long nowMicros = RobotController.getFPGATime();
		inputs.connected = ((nowMicros - heartbeatSubscriber.getLastChange()) / 1000) < 250;

		inputs.latestTargetObservation = new TargetObservation(Rotation2d.fromDegrees(txSubscriber.get()),
				Rotation2d.fromDegrees(tySubscriber.get()), (int) tidSubscriber.get());

		double[] limelightRobotSpace = toLimelightRobotSpace(robotToCamera);
		if (lastPublishedCameraPose == null || !Arrays.equals(lastPublishedCameraPose, limelightRobotSpace)) {
			cameraPosePublisher.accept(limelightRobotSpace);
			NetworkTableInstance.getDefault().flush();
			lastPublishedCameraPose = limelightRobotSpace.clone();
		}
		Logger.recordOutput("Vision/" + name + "/RobotToCamera", new Pose3d().transformBy(robotToCamera));
		Logger.recordOutput("Vision/" + name + "/RobotSpaceConfig", limelightRobotSpace);

		if (useMegatag2) {
			orientationPublisher.accept(new double[]{rotationSupplier.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0});
			NetworkTableInstance.getDefault().flush();
		}

		Set<Integer> tagIds = new HashSet<>();
		List<PoseObservation> poseObservations = new LinkedList<>();
		if (useMegatag1) {
			for (var rawSample : megatag1Subscriber.readQueue()) {
				int tagCount = getTagCount(rawSample.value);
				if (poseMode == LimelightPoseMode.MEGATAG_1_WITH_MEGATAG_2_SINGLE_TAG && tagCount == 1) {
					continue;
				}
				PoseObservation observation = parseObservation(rawSample.value, rawSample.timestamp,
						PoseObservationType.MEGATAG_1, tagIds);
				if (observation != null) {
					poseObservations.add(observation);
				}
			}
		}
		if (useMegatag2) {
			for (var rawSample : megatag2Subscriber.readQueue()) {
				int tagCount = getTagCount(rawSample.value);
				if (poseMode == LimelightPoseMode.MEGATAG_1_WITH_MEGATAG_2_SINGLE_TAG && tagCount != 1) {
					continue;
				}
				PoseObservation observation = parseObservation(rawSample.value, rawSample.timestamp,
						PoseObservationType.MEGATAG_2, tagIds);
				if (observation != null) {
					poseObservations.add(observation);
				}
			}
		}

		inputs.poseObservations = new PoseObservation[poseObservations.size()];
		for (int i = 0; i < poseObservations.size(); i++) {
			inputs.poseObservations[i] = poseObservations.get(i);
		}

		inputs.tagIds = new int[tagIds.size()];
		int i = 0;
		for (int id : tagIds) {
			inputs.tagIds[i++] = id;
		}
	}

	private static PoseObservation parseObservation(double[] rawLLArray, long timestampMicros, PoseObservationType type,
			Set<Integer> tagIds) {
		if (rawLLArray.length < 11) {
			return null;
		}

		int tagCount = (int) rawLLArray[7];
		if (tagCount <= 0) {
			return null;
		}

		for (int i = 11; i + 6 < rawLLArray.length; i += 7) {
			tagIds.add((int) rawLLArray[i]);
		}

		double ambiguity = type == PoseObservationType.MEGATAG_1 && rawLLArray.length >= 18 ? rawLLArray[17] : 0.0;
		return new PoseObservation(timestampMicros * 1.0e-6 - rawLLArray[6] * 1.0e-3, parsePose(rawLLArray), ambiguity,
				tagCount, rawLLArray[9], type);
	}

	private static int getTagCount(double[] rawLLArray) {
		if (rawLLArray.length < 8) {
			return 0;
		}
		return (int) rawLLArray[7];
	}

	private static double[] toLimelightRobotSpace(Transform3d robotToCamera) {
		Rotation3d rotation = robotToCamera.getRotation();
		return new double[]{robotToCamera.getX(), -robotToCamera.getY(), robotToCamera.getZ(),
				Units.radiansToDegrees(-rotation.getX()), Units.radiansToDegrees(rotation.getY()),
				Units.radiansToDegrees(-rotation.getZ())};
	}

	/** Parses the 3D pose from a Limelight botpose array. */
	private static Pose3d parsePose(double[] rawLLArray) {
		return new Pose3d(rawLLArray[0], rawLLArray[1], rawLLArray[2],
				new Rotation3d(Units.degreesToRadians(rawLLArray[3]), Units.degreesToRadians(rawLLArray[4]),
						Units.degreesToRadians(rawLLArray[5])));
	}
}
