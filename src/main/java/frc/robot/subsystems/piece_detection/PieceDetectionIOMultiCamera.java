package frc.robot.subsystems.piece_detection;

import edu.wpi.first.math.geometry.Transform3d;

public class PieceDetectionIOMultiCamera implements PieceDetectionIO {
	private final String name;
	private final PieceDetectionIO[] cameras;

	public PieceDetectionIOMultiCamera(String name, PieceDetectionIO... cameras) {
		this.name = name;
		this.cameras = cameras.clone();
	}

	@Override
	public void updateInputs(PieceDetectionIOInputs inputs) {
		clear(inputs);

		PieceDetectionIOInputs best = null;
		int totalGroups = 0;
		boolean anyConnected = false;

		for (PieceDetectionIO camera : cameras) {
			PieceDetectionIOInputs cameraInputs = new PieceDetectionIOInputs();
			camera.updateInputs(cameraInputs);

			anyConnected = anyConnected || cameraInputs.connected;
			totalGroups += cameraInputs.groupCount;

			if (!cameraInputs.seesTarget) {
				continue;
			}

			if (best == null || cameraInputs.selectedGroupScore > best.selectedGroupScore) {
				best = cameraInputs;
			}
		}

		inputs.connected = anyConnected;
		inputs.groupCount = totalGroups;

		if (best != null) {
			copyBestTarget(best, inputs);
			inputs.groupCount = totalGroups;
		}
	}

	private void copyBestTarget(PieceDetectionIOInputs from, PieceDetectionIOInputs to) {
		to.yaw = from.yaw;
		to.pitch = from.pitch;
		to.area = from.area;
		to.distance = from.distance;
		to.robotToPieceTransform = from.robotToPieceTransform;
		to.seesTarget = from.seesTarget;
		to.biggestGroupBallCount = from.biggestGroupBallCount;
		to.closestGroupBallCount = from.closestGroupBallCount;
		to.selectedGroupBallCount = from.selectedGroupBallCount;
		to.biggestGroupDistance = from.biggestGroupDistance;
		to.closestGroupDistance = from.closestGroupDistance;
		to.selectedGroupDistance = from.selectedGroupDistance;
		to.selectedGroupYaw = from.selectedGroupYaw;
		to.selectedGroupKeptBalls = from.selectedGroupKeptBalls;
		to.closestGroupTripsDuringBigTrip = from.closestGroupTripsDuringBigTrip;
		to.selectedGroupScore = from.selectedGroupScore;
		to.selectedGroupShape = from.selectedGroupShape;
		to.selectedGroupReason = from.selectedGroupReason;
	}

	private void clear(PieceDetectionIOInputs inputs) {
		inputs.connected = false;
		inputs.seesTarget = false;
		inputs.yaw = 0.0;
		inputs.pitch = 0.0;
		inputs.area = 0.0;
		inputs.distance = 0.0;
		inputs.robotToPieceTransform = new Transform3d();
		inputs.groupCount = 0;
		inputs.biggestGroupBallCount = 0;
		inputs.closestGroupBallCount = 0;
		inputs.selectedGroupBallCount = 0;
		inputs.biggestGroupDistance = 0.0;
		inputs.closestGroupDistance = 0.0;
		inputs.selectedGroupDistance = 0.0;
		inputs.selectedGroupYaw = 0.0;
		inputs.selectedGroupKeptBalls = 0.0;
		inputs.closestGroupTripsDuringBigTrip = 0.0;
		inputs.selectedGroupScore = 0.0;
		inputs.selectedGroupShape = "";
		inputs.selectedGroupReason = "";
	}

	@Override
	public String getName() {
		return name;
	}
}
