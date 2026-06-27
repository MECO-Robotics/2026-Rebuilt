package frc.robot.dashboard;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

public class DashboardAutoDriveController extends SubsystemBase {
	private static final double FIELD_LENGTH_METERS = 16.540988;
	private static final double FIELD_WIDTH_METERS = 8.052;
	private static final double DRIVE_REQUEST_TIMEOUT_SECONDS = 8.0;
	private static final PathConstraints DASHBOARD_PATH_CONSTRAINTS = new PathConstraints(2.0, 1.5, 3.0, 4.0);

	private final CommandSwerveDrivetrain drivetrain;
	private final Runnable emergencyStop;
	private final DoubleArraySubscriber clickedPoseSubscriber;
	private final IntegerSubscriber requestIdSubscriber;
	private final IntegerSubscriber killRequestIdSubscriber;

	private boolean initialized = false;
	private long lastSeenRequestId = -1;
	private long lastSeenKillRequestId = -1;
	private Command activeCommand = null;

	public DashboardAutoDriveController(CommandSwerveDrivetrain drivetrain, Runnable emergencyStop) {
		this.drivetrain = drivetrain;
		this.emergencyStop = emergencyStop;

		NetworkTable autoDriveTable = NetworkTableInstance.getDefault().getTable("AutoDrive");
		clickedPoseSubscriber = autoDriveTable.getDoubleArrayTopic("ClickedPose").subscribe(new double[0]);
		requestIdSubscriber = autoDriveTable.getIntegerTopic("RequestId").subscribe(-1);
		killRequestIdSubscriber = autoDriveTable.getIntegerTopic("KillRequestId").subscribe(-1);

		SmartDashboard.putBoolean("AutoDrive/Active", false);
		SmartDashboard.putNumber("AutoDrive/LastAcceptedRequestId", -1);
		SmartDashboard.putNumber("AutoDrive/LastKillRequestId", -1);
		SmartDashboard.putString("AutoDrive/LastRejectedReason", "No dashboard request received");
	}

	@Override
	public void periodic() {
		long requestId = requestIdSubscriber.get(-1);
		long killRequestId = killRequestIdSubscriber.get(-1);
		if (!initialized) {
			lastSeenRequestId = requestId;
			lastSeenKillRequestId = killRequestId;
			initialized = true;
			return;
		}

		if (killRequestId != lastSeenKillRequestId) {
			lastSeenKillRequestId = killRequestId;
			handleKillRequest(killRequestId);
			return;
		}

		if (requestId == lastSeenRequestId) {
			return;
		}

		lastSeenRequestId = requestId;
		handleRequest(requestId, clickedPoseSubscriber.get(new double[0]));
	}

	private void handleKillRequest(long killRequestId) {
		if (activeCommand != null && activeCommand.isScheduled()) {
			activeCommand.cancel();
		}

		SmartDashboard.putBoolean("AutoDrive/Active", false);
		SmartDashboard.putNumber("AutoDrive/LastKillRequestId", killRequestId);
		SmartDashboard.putString("AutoDrive/LastRejectedReason", "Enter kill requested");
		emergencyStop.run();
	}

	private void handleRequest(long requestId, double[] poseArray) {
		if (!DriverStation.isTeleopEnabled()) {
			reject("Robot must be teleop-enabled for dashboard auto-drive");
			return;
		}

		if (poseArray.length < 3) {
			reject("ClickedPose must be [xMeters, yMeters, headingDegrees]");
			return;
		}

		double xMeters = poseArray[0];
		double yMeters = poseArray[1];
		double headingDegrees = poseArray[2];
		if (!Double.isFinite(xMeters) || !Double.isFinite(yMeters) || !Double.isFinite(headingDegrees)) {
			reject("ClickedPose contains a non-finite value");
			return;
		}

		if (xMeters < 0.0 || xMeters > FIELD_LENGTH_METERS || yMeters < 0.0 || yMeters > FIELD_WIDTH_METERS) {
			reject("ClickedPose is outside the field");
			return;
		}

		Pose2d targetPose = new Pose2d(xMeters, yMeters, Rotation2d.fromDegrees(headingDegrees));
		schedulePathfind(requestId, targetPose);
	}

	private void schedulePathfind(long requestId, Pose2d targetPose) {
		if (activeCommand != null && activeCommand.isScheduled()) {
			activeCommand.cancel();
		}

		SmartDashboard.putBoolean("AutoDrive/Active", true);
		SmartDashboard.putNumber("AutoDrive/LastAcceptedRequestId", requestId);
		SmartDashboard.putString("AutoDrive/LastRejectedReason", "");
		SmartDashboard.putString("AutoDrive/TargetPose", String.format("%.2f, %.2f, %.1f deg", targetPose.getX(),
				targetPose.getY(), targetPose.getRotation().getDegrees()));

		activeCommand = AutoBuilder.pathfindToPose(targetPose, DASHBOARD_PATH_CONSTRAINTS)
				.withTimeout(DRIVE_REQUEST_TIMEOUT_SECONDS).andThen(Commands.runOnce(drivetrain::stop, drivetrain))
				.finallyDo(() -> SmartDashboard.putBoolean("AutoDrive/Active", false))
				.withName("DashboardAutoDriveToPose");
		activeCommand.schedule();
	}

	private void reject(String reason) {
		SmartDashboard.putBoolean("AutoDrive/Active", false);
		SmartDashboard.putString("AutoDrive/LastRejectedReason", reason);
	}
}
