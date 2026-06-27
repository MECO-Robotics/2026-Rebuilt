package frc.robot.subsystems.obstacle_detection;

import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.drive.DrivetrainConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class RobotObstacleTracker extends SubsystemBase {
	public static final String ROBOT_OBSTACLES_TOPIC = "Autonomy/RobotObstacles";

	private static final double ROBOT_OBSTACLE_HALF_SIZE_METERS = 0.75;
	private static final double IGNORE_SELF_RADIUS_METERS = 0.8;

	private final Supplier<Pose2d> robotPoseSupplier;
	private final DoubleArraySubscriber obstacleCentersSubscriber;
	private List<Pair<Translation2d, Translation2d>> latestObstacles = List.of();

	public RobotObstacleTracker(Supplier<Pose2d> robotPoseSupplier) {
		this.robotPoseSupplier = robotPoseSupplier;
		obstacleCentersSubscriber = NetworkTableInstance.getDefault().getDoubleArrayTopic(ROBOT_OBSTACLES_TOPIC)
				.subscribe(new double[0]);
	}

	@Override
	public void periodic() {
		Pose2d robotPose = robotPoseSupplier.get();
		List<Pair<Translation2d, Translation2d>> obstacles = buildObstacleBoxes(obstacleCentersSubscriber.get(),
				robotPose.getTranslation());
		latestObstacles = obstacles;

		Pathfinding.setDynamicObstacles(obstacles, robotPose.getTranslation());
		Logger.recordOutput("Autonomy/RobotObstacleCount", obstacles.size());
		Logger.recordOutput("Autonomy/RobotObstacleBoxes", flattenObstacleBoxes(obstacles));
	}

	private List<Pair<Translation2d, Translation2d>> buildObstacleBoxes(double[] obstacleCenters,
			Translation2d robotTranslation) {
		List<Pair<Translation2d, Translation2d>> obstacles = new ArrayList<>();

		for (int i = 0; i + 1 < obstacleCenters.length; i += 2) {
			double x = obstacleCenters[i];
			double y = obstacleCenters[i + 1];

			if (!Double.isFinite(x) || !Double.isFinite(y)) {
				continue;
			}

			Translation2d center = new Translation2d(x, y);
			if (!isInsideField(center) || center.getDistance(robotTranslation) < IGNORE_SELF_RADIUS_METERS) {
				continue;
			}

			Translation2d minCorner = new Translation2d(x - ROBOT_OBSTACLE_HALF_SIZE_METERS,
					y - ROBOT_OBSTACLE_HALF_SIZE_METERS);
			Translation2d maxCorner = new Translation2d(x + ROBOT_OBSTACLE_HALF_SIZE_METERS,
					y + ROBOT_OBSTACLE_HALF_SIZE_METERS);
			obstacles.add(new Pair<>(minCorner, maxCorner));
		}

		return obstacles;
	}

	public Pose3d[] getObstacleCenterPoses() {
		Pose3d[] poses = new Pose3d[latestObstacles.size()];
		for (int i = 0; i < latestObstacles.size(); i++) {
			Pair<Translation2d, Translation2d> obstacle = latestObstacles.get(i);
			Translation2d center = obstacle.getFirst().plus(obstacle.getSecond()).div(2.0);
			poses[i] = new Pose3d(center.getX(), center.getY(), 0.0, new Rotation3d());
		}
		return poses;
	}

	public Pose3d[] getObstacleCornerPoses() {
		Pose3d[] poses = new Pose3d[latestObstacles.size() * 2];
		int idx = 0;
		for (Pair<Translation2d, Translation2d> obstacle : latestObstacles) {
			poses[idx++] = new Pose3d(obstacle.getFirst().getX(), obstacle.getFirst().getY(), 0.0, new Rotation3d());
			poses[idx++] = new Pose3d(obstacle.getSecond().getX(), obstacle.getSecond().getY(), 0.0, new Rotation3d());
		}
		return poses;
	}

	private boolean isInsideField(Translation2d translation) {
		return translation.getX() >= 0.0 && translation.getX() <= DrivetrainConstants.kSimFieldLengthMeters
				&& translation.getY() >= 0.0 && translation.getY() <= DrivetrainConstants.kSimFieldWidthMeters;
	}

	private double[] flattenObstacleBoxes(List<Pair<Translation2d, Translation2d>> obstacles) {
		double[] flattened = new double[obstacles.size() * 4];
		int idx = 0;
		for (Pair<Translation2d, Translation2d> obstacle : obstacles) {
			flattened[idx++] = obstacle.getFirst().getX();
			flattened[idx++] = obstacle.getFirst().getY();
			flattened[idx++] = obstacle.getSecond().getX();
			flattened[idx++] = obstacle.getSecond().getY();
		}
		return flattened;
	}
}
