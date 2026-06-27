package frc.robot.subsystems.field_mapping;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.drive.DrivetrainConstants;
import frc.robot.constants.vision.VisionConstants;
import frc.robot.subsystems.obstacle_detection.RobotObstacleTracker;
import frc.robot.subsystems.piece_detection.PieceDetection;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class FieldMap3d extends SubsystemBase {
	private final Supplier<Pose2d> robotPoseSupplier;
	private final PieceDetection pieceDetection;
	private final RobotObstacleTracker robotObstacleTracker;
	private final Pose3d[] fieldBoundaryPoses;
	private final Pose3d[] aprilTagPoses;

	public FieldMap3d(Supplier<Pose2d> robotPoseSupplier, PieceDetection pieceDetection,
			RobotObstacleTracker robotObstacleTracker) {
		this.robotPoseSupplier = robotPoseSupplier;
		this.pieceDetection = pieceDetection;
		this.robotObstacleTracker = robotObstacleTracker;
		fieldBoundaryPoses = createFieldBoundaryPoses();
		aprilTagPoses = VisionConstants.aprilTagLayout.getTags().stream().map((tag) -> tag.pose).toArray(Pose3d[]::new);
	}

	@Override
	public void periodic() {
		Logger.recordOutput("FieldMap3d/RobotPose", new Pose3d(robotPoseSupplier.get()));
		Logger.recordOutput("FieldMap3d/FieldBoundary", fieldBoundaryPoses);
		Logger.recordOutput("FieldMap3d/HubPose", new Pose3d(FieldConstants.Hub.hubPosition().getX(),
				FieldConstants.Hub.hubPosition().getY(), FieldConstants.Hub.hubHeight, new Rotation3d()));
		Logger.recordOutput("FieldMap3d/AprilTagPoses", aprilTagPoses);
		Logger.recordOutput("FieldMap3d/SelectedPiecePose", getSelectedPiecePoses());
		Logger.recordOutput("FieldMap3d/RobotObstacleCenters", robotObstacleTracker.getObstacleCenterPoses());
		Logger.recordOutput("FieldMap3d/RobotObstacleBoxCorners", robotObstacleTracker.getObstacleCornerPoses());
	}

	private Pose3d[] getSelectedPiecePoses() {
		if (!pieceDetection.pieceDetected()) {
			return new Pose3d[0];
		}
		return new Pose3d[]{pieceDetection.getPiecePose()};
	}

	private Pose3d[] createFieldBoundaryPoses() {
		double length = DrivetrainConstants.kSimFieldLengthMeters;
		double width = DrivetrainConstants.kSimFieldWidthMeters;
		return new Pose3d[]{new Pose3d(0.0, 0.0, 0.0, new Rotation3d()), new Pose3d(length, 0.0, 0.0, new Rotation3d()),
				new Pose3d(length, width, 0.0, new Rotation3d()), new Pose3d(0.0, width, 0.0, new Rotation3d())};
	}
}
