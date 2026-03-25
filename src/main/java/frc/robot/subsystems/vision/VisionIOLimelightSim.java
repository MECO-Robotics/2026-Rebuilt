package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.function.Supplier;

/** Simulated Limelight backed by PhotonVision's camera simulator. */
public class VisionIOLimelightSim extends VisionIOPhotonVisionSim {
	/**
	 * Creates a new simulated Limelight IO.
	 *
	 * @param name
	 *            The configured camera name.
	 * @param robotToCamera
	 *            Transform from robot origin to the Limelight.
	 * @param poseSupplier
	 *            Supplier for the robot pose used by the simulation.
	 */
	public VisionIOLimelightSim(String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
		super(name, robotToCamera, poseSupplier);
	}

	@Override
	public void updateInputs(VisionIOInputs inputs) {
		super.updateInputs(inputs);

		PoseObservation[] limelightObservations = new PoseObservation[inputs.poseObservations.length];
		for (int i = 0; i < inputs.poseObservations.length; i++) {
			PoseObservation observation = inputs.poseObservations[i];
			limelightObservations[i] = new PoseObservation(observation.timestamp(), observation.pose(),
					observation.ambiguity(), observation.tagCount(), observation.averageTagDistance(),
					PoseObservationType.MEGATAG_1);
		}
		inputs.poseObservations = limelightObservations;
	}
}
