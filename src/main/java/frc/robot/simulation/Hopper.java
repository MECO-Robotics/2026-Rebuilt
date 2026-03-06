package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.constants.simulation.MapleSimConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Simulated hopper state source used by visualization/logging. */
public class Hopper {
	public static final String BALL_COUNT_LOG_KEY = "Visualization/RobotRemy/Hopper/BallCount";
	public static final String FILL_RATIO_LOG_KEY = "Visualization/RobotRemy/Hopper/FillRatio";
	public static final String GAMEPIECE_POSES_LOG_KEY = "Visualization/RobotRemy/Hopper/GamePiecePoses";

	private final IntSupplier ballCountSupplier;
	private final DoubleSupplier intakeExtensionSupplier;
	private final Supplier<Pose2d> robotPoseSupplier;

	public Hopper(IntSupplier ballCountSupplier, DoubleSupplier intakeExtensionSupplier,
			Supplier<Pose2d> robotPoseSupplier) {
		this.ballCountSupplier = ballCountSupplier;
		this.intakeExtensionSupplier = intakeExtensionSupplier;
		this.robotPoseSupplier = robotPoseSupplier;
	}

	/** Returns the clamped hopper ball count used for visualization. */
	public int getBallCount() {
		return Math.max(0, Math.min(getMaxStorableBalls(), ballCountSupplier.getAsInt()));
	}

	/** Returns hopper fill from 0.0 to 1.0 based on ball count. */
	public double getFillRatio() {
		int maxStorableBalls = getMaxStorableBalls();
		return maxStorableBalls > 0 ? getBallCount() / (double) maxStorableBalls : 0.0;
	}

	/**
	 * Returns field-relative poses for all game pieces currently held inside the
	 * robot hopper.
	 */
	public Pose3d[] getGamePiecePoses() {
		int count = getBallCount();
		if (count == 0) {
			return new Pose3d[0];
		}

		Pose3d robotPose = new Pose3d(robotPoseSupplier.get());
		List<Translation3d> localBallCenters = buildLocalBallCenters();
		List<Pose3d> poses = new ArrayList<>(count);
		for (int i = 0; i < count; i++) {
			Translation3d localCenter = localBallCenters.get(i);

			poses.add(robotPose.transformBy(new Transform3d(localCenter, Rotation3d.kZero)));
		}
		return poses.toArray(Pose3d[]::new);
	}

	private int getMaxStorableBalls() {
		return Math.min(MapleSimConstants.HOPPER_MAX_BALLS, buildLocalBallCenters().size());
	}

	private List<Translation3d> buildLocalBallCenters() {
		double diameter = MapleSimConstants.HOPPER_BALL_DIAMETER_METERS;
		double radius = diameter / 2.0;
		// Triangular layer spacing for sphere stacking (instead of axis-aligned cube
		// packing).
		double verticalPitch = Math.sqrt(3.0) / 2.0 * diameter;
		double intakeFrontProjection = Math.max(0.0, intakeExtensionSupplier.getAsDouble())
				* Math.cos(MapleSimConstants.INTAKE_ANGLE_RADIANS);
		double backLimit = MapleSimConstants.HOPPER_BACK_LIMIT_X_METERS;
		double frontLimit = MapleSimConstants.HOPPER_FRONT_LIMIT_X_METERS + intakeFrontProjection;
		double rightLimit = MapleSimConstants.HOPPER_RIGHT_LIMIT_Y_METERS;
		double leftLimit = MapleSimConstants.HOPPER_LEFT_LIMIT_Y_METERS;
		double bottomLimit = MapleSimConstants.HOPPER_BOTTOM_LIMIT_Z_METERS;
		double topLimit = MapleSimConstants.HOPPER_TOP_LIMIT_Z_METERS;

		List<Translation3d> centers = new ArrayList<>();
		// Fill order requirement: bottom -> top, then back -> front.
		int zIndex = 0;
		for (double z = bottomLimit + radius; z <= topLimit - radius + 1e-9; z += verticalPitch) {
			double xOffset = (zIndex % 2 == 0) ? 0.0 : radius;
			int xIndex = 0;
			for (double x = backLimit + radius + xOffset; x <= frontLimit - radius + 1e-9; x += diameter) {
				// Alternate lateral offset for neighboring stacks to mimic sphere nesting in
				// 3D.
				double yOffset = ((zIndex + xIndex) % 2 == 0) ? 0.0 : radius;
				for (double y = rightLimit + radius + yOffset; y <= leftLimit - radius + 1e-9; y += diameter) {
					centers.add(new Translation3d(x, y, z));
				}
				xIndex++;
			}
			zIndex++;
		}
		return centers;
	}

	/** Publishes hopper telemetry for AdvantageScope. */
	public void periodic() {
		Logger.recordOutput(BALL_COUNT_LOG_KEY, getBallCount());
		Logger.recordOutput(FILL_RATIO_LOG_KEY, getFillRatio());
		Logger.recordOutput(GAMEPIECE_POSES_LOG_KEY, getGamePiecePoses());
	}
}
