package frc.robot.subsystems.piece_detection;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.function.Supplier;

public class PieceDetectionIOSim implements PieceDetectionIO {
	private static final double KEPT_BALL_RATIO = 60.0 / 70.0;
	private static final double SIM_CRUISE_SPEED_METERS_PER_SECOND = 2.0;
	private static final double SIM_PICKUP_AND_SHOOT_SECONDS = 3.0;

	Supplier<Pose3d[]> notePoses;
	Supplier<Pose3d> drivePose;
	String name;

	public PieceDetectionIOSim(String name, Supplier<Pose3d[]> notePoses, Supplier<Pose3d> drivePose) {
		this.notePoses = notePoses;
		this.drivePose = drivePose;
		this.name = name;
	}

	@Override
	public void updateInputs(PieceDetectionIOInputs inputs) {
		inputs.connected = true;

		Pose3d[] poses = notePoses.get();
		if (poses.length == 0) {
			clearTarget(inputs);
			inputs.connected = true;
			return;
		}

		Pose3d robotPose = drivePose.get();
		Translation2d robotTranslation = robotPose.toPose2d().getTranslation();
		Pose3d closestPose = poses[0];
		double closestDistance = Double.POSITIVE_INFINITY;

		for (Pose3d pose : poses) {
			double distance = pose.toPose2d().getTranslation().getDistance(robotTranslation);
			if (distance < closestDistance) {
				closestDistance = distance;
				closestPose = pose;
			}
		}

		Translation2d fieldRelative = closestPose.toPose2d().getTranslation().minus(robotTranslation);
		Translation2d robotRelative = fieldRelative.rotateBy(robotPose.toPose2d().getRotation().unaryMinus());
		double yawDeg = Math.toDegrees(Math.atan2(robotRelative.getY(), robotRelative.getX()));
		double score = keptBalls(1)
				/ (closestDistance / SIM_CRUISE_SPEED_METERS_PER_SECOND + SIM_PICKUP_AND_SHOOT_SECONDS);

		inputs.seesTarget = true;
		inputs.groupCount = poses.length;
		inputs.yaw = yawDeg;
		inputs.pitch = 0.0;
		inputs.area = Math.max(1.0, 1000.0 / Math.max(closestDistance, 0.1));
		inputs.distance = closestDistance;
		inputs.robotToPieceTransform = new Transform3d(
				new Translation3d(robotRelative.getX(), robotRelative.getY(), 0.0), new Rotation3d());
		inputs.biggestGroupBallCount = 1;
		inputs.closestGroupBallCount = 1;
		inputs.selectedGroupBallCount = 1;
		inputs.biggestGroupDistance = closestDistance;
		inputs.closestGroupDistance = closestDistance;
		inputs.selectedGroupDistance = closestDistance;
		inputs.selectedGroupYaw = yawDeg;
		inputs.selectedGroupKeptBalls = keptBalls(1);
		inputs.closestGroupTripsDuringBigTrip = 1.0;
		inputs.selectedGroupScore = score;
		inputs.selectedGroupShape = "circle";
		inputs.selectedGroupReason = "sim closest fuel";
	}

	private double keptBalls(int rawBalls) {
		return rawBalls * KEPT_BALL_RATIO;
	}

	private void clearTarget(PieceDetectionIOInputs inputs) {
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
