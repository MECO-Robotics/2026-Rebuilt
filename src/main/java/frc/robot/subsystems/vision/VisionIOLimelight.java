package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Objects;
import java.util.Set;
import java.util.function.Supplier;

/** IO implementation for real Limelight hardware. */
public class VisionIOLimelight implements VisionIO {
	private final Supplier<Rotation2d> rotationSupplier;
	private final boolean useMegatag1;
	private final boolean useMegatag2;
	private final DoubleArrayPublisher orientationPublisher;

	private final DoubleSubscriber heartbeatSubscriber;
	private final DoubleSubscriber txSubscriber;
	private final DoubleSubscriber tySubscriber;
	private final DoubleSubscriber tidSubscriber;
	private final DoubleArraySubscriber megatag1Subscriber;
	private final DoubleArraySubscriber megatag2Subscriber;

	/**
	 * Creates a new VisionIOLimelight.
	 *
	 * @param name
	 *            The configured name of the Limelight. Uses MegaTag 1 observations
	 *            only.
	 */
	public VisionIOLimelight(String name) {
		this(name, null, true, false);
	}

	/**
	 * Creates a new VisionIOLimelight.
	 *
	 * @param name
	 *            The configured name of the Limelight.
	 * @param rotationSupplier
	 *            Supplier for the current estimated rotation, used for MegaTag 2.
	 */
	public VisionIOLimelight(String name, Supplier<Rotation2d> rotationSupplier) {
		this(name, rotationSupplier, false, true);
	}

	private VisionIOLimelight(String name, Supplier<Rotation2d> rotationSupplier, boolean useMegatag1,
			boolean useMegatag2) {
		var table = NetworkTableInstance.getDefault().getTable(name);
		this.rotationSupplier = rotationSupplier;
		this.useMegatag1 = useMegatag1;
		this.useMegatag2 = useMegatag2;
		orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
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
		// Update connection status based on whether an update has been seen in the last
		// 250ms
		long nowMicros = RobotController.getFPGATime();
		inputs.connected = ((nowMicros - heartbeatSubscriber.getLastChange()) / 1000) < 250;

		// Update target observation
		inputs.latestTargetObservation = new TargetObservation(Rotation2d.fromDegrees(txSubscriber.get()),
				Rotation2d.fromDegrees(tySubscriber.get()), (int) tidSubscriber.get());

		// Update orientation for MegaTag 2
		if (useMegatag2) {
			orientationPublisher.accept(new double[]{rotationSupplier.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0});
			NetworkTableInstance.getDefault().flush(); // Recommended by Limelight for MT2 orientation sync
		}

		// Read new pose observations from NetworkTables
		Set<Integer> tagIds = new HashSet<>();
		List<PoseObservation> poseObservations = new LinkedList<>();
		if (useMegatag1) {
			for (var rawSample : megatag1Subscriber.readQueue()) {
				PoseObservation observation = parseObservation(rawSample.value, rawSample.timestamp,
						PoseObservationType.MEGATAG_1, tagIds);
				if (observation != null) {
					poseObservations.add(observation);
				}
			}
		}
		if (useMegatag2) {
			for (var rawSample : megatag2Subscriber.readQueue()) {
				PoseObservation observation = parseObservation(rawSample.value, rawSample.timestamp,
						PoseObservationType.MEGATAG_2, tagIds);
				if (observation != null) {
					poseObservations.add(observation);
				}
			}
		}

		// Save pose observations to inputs object
		inputs.poseObservations = new PoseObservation[poseObservations.size()];
		for (int i = 0; i < poseObservations.size(); i++) {
			inputs.poseObservations[i] = poseObservations.get(i);
		}

		// Save tag IDs to inputs objects
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

	/** Parses the 3D pose from a Limelight botpose array. */
	private static Pose3d parsePose(double[] rawLLArray) {
		return new Pose3d(rawLLArray[0], rawLLArray[1], rawLLArray[2],
				new Rotation3d(Units.degreesToRadians(rawLLArray[3]), Units.degreesToRadians(rawLLArray[4]),
						Units.degreesToRadians(rawLLArray[5])));
	}
}
