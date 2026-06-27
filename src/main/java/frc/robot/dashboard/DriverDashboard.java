package frc.robot.dashboard;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.vision.PieceDetectionConstants;
import frc.robot.subsystems.piece_detection.PieceDetection;

public class DriverDashboard {
	private static final String TAB_NAME = "8324 Driver";
	private static final String SMARTDASHBOARD_PREFIX = "DriverDashboard/";

	private final PieceDetection pieceDetection;

	public DriverDashboard(PieceDetection pieceDetection) {
		this.pieceDetection = pieceDetection;

		ShuffleboardTab tab = Shuffleboard.getTab(TAB_NAME);
		configureCameraWidgets(tab);
		configureTelemetryWidgets(tab);
		publishStaticLinks();
	}

	public void update() {
		SmartDashboard.putBoolean("AutoDrive/Is Red Alliance",
				DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red);
		SmartDashboard.putBoolean("AutoDrive/Enabled", DriverStation.isEnabled());
		SmartDashboard.putString("AutoDrive/Mode", DriverStation.isAutonomous() ? "Autonomous" : "Teleop");

		SmartDashboard.putBoolean(SMARTDASHBOARD_PREFIX + "Game Piece Camera Connected", pieceDetection.isConnected());
		SmartDashboard.putBoolean(SMARTDASHBOARD_PREFIX + "Sees Balls", pieceDetection.pieceDetected());
		SmartDashboard.putNumber(SMARTDASHBOARD_PREFIX + "Detected Groups", pieceDetection.getGroupCount());
		SmartDashboard.putNumber(SMARTDASHBOARD_PREFIX + "Selected Ball Count",
				pieceDetection.getSelectedGroupBallCount());
		SmartDashboard.putNumber(SMARTDASHBOARD_PREFIX + "Selected Distance Meters",
				pieceDetection.getSelectedGroupDistance());
		SmartDashboard.putNumber(SMARTDASHBOARD_PREFIX + "Selected Yaw Degrees", pieceDetection.getSelectedGroupYaw());
		SmartDashboard.putNumber(SMARTDASHBOARD_PREFIX + "Selected Kept Balls",
				pieceDetection.getSelectedGroupKeptBalls());
		SmartDashboard.putString(SMARTDASHBOARD_PREFIX + "Selected Shape", pieceDetection.getSelectedGroupShape());
		SmartDashboard.putString(SMARTDASHBOARD_PREFIX + "Selected Reason", pieceDetection.getSelectedGroupReason());
		SmartDashboard.putNumber(SMARTDASHBOARD_PREFIX + "Biggest Ball Count",
				pieceDetection.getBiggestGroupBallCount());
		SmartDashboard.putNumber(SMARTDASHBOARD_PREFIX + "Biggest Distance Meters",
				pieceDetection.getBiggestGroupDistance());
		SmartDashboard.putNumber(SMARTDASHBOARD_PREFIX + "Closest Ball Count",
				pieceDetection.getClosestGroupBallCount());
		SmartDashboard.putNumber(SMARTDASHBOARD_PREFIX + "Closest Distance Meters",
				pieceDetection.getClosestGroupDistance());
		SmartDashboard.putNumber(SMARTDASHBOARD_PREFIX + "Closest Trips During Big Trip",
				pieceDetection.getClosestGroupTripsDuringBigTrip());
		SmartDashboard.putNumber(SMARTDASHBOARD_PREFIX + "Selected Score", pieceDetection.getSelectedGroupScore());
	}

	private void configureCameraWidgets(ShuffleboardTab tab) {
		if (PieceDetectionConstants.ENABLE_FRONT_GAME_PIECE_CAMERA) {
			tab.addCamera("Front Game Piece View", PieceDetectionConstants.FRONT_GAME_PIECE_CAMERA_NAME,
					PieceDetectionConstants.FRONT_GAME_PIECE_CAMERA_VIDEO_URI.toString())
					.withWidget(BuiltInWidgets.kCameraStream).withPosition(0, 0).withSize(5, 4);
		}

		if (PieceDetectionConstants.ENABLE_REAR_GAME_PIECE_CAMERA) {
			tab.addCamera("Rear Game Piece View", PieceDetectionConstants.REAR_GAME_PIECE_CAMERA_NAME,
					PieceDetectionConstants.REAR_GAME_PIECE_CAMERA_VIDEO_URI.toString())
					.withWidget(BuiltInWidgets.kCameraStream).withPosition(5, 0).withSize(5, 4);
		}
	}

	private void configureTelemetryWidgets(ShuffleboardTab tab) {
		tab.addBoolean("Camera Connected", pieceDetection::isConnected).withWidget(BuiltInWidgets.kBooleanBox)
				.withPosition(0, 4).withSize(2, 1);
		tab.addBoolean("Sees Balls", pieceDetection::pieceDetected).withWidget(BuiltInWidgets.kBooleanBox)
				.withPosition(2, 4).withSize(2, 1);
		tab.addNumber("Detected Groups", pieceDetection::getGroupCount).withWidget(BuiltInWidgets.kTextView)
				.withPosition(4, 4).withSize(2, 1);
		tab.addNumber("Selected Balls", pieceDetection::getSelectedGroupBallCount).withWidget(BuiltInWidgets.kTextView)
				.withPosition(0, 5).withSize(2, 1);
		tab.addNumber("Selected Distance m", pieceDetection::getSelectedGroupDistance)
				.withWidget(BuiltInWidgets.kTextView).withPosition(2, 5).withSize(2, 1);
		tab.addNumber("Selected Yaw deg", pieceDetection::getSelectedGroupYaw).withWidget(BuiltInWidgets.kTextView)
				.withPosition(4, 5).withSize(2, 1);
		tab.addNumber("Biggest Balls", pieceDetection::getBiggestGroupBallCount).withWidget(BuiltInWidgets.kTextView)
				.withPosition(0, 6).withSize(2, 1);
		tab.addNumber("Biggest Distance m", pieceDetection::getBiggestGroupDistance)
				.withWidget(BuiltInWidgets.kTextView).withPosition(2, 6).withSize(2, 1);
		tab.addNumber("Closest Balls", pieceDetection::getClosestGroupBallCount).withWidget(BuiltInWidgets.kTextView)
				.withPosition(4, 6).withSize(2, 1);
		tab.addNumber("Closest Distance m", pieceDetection::getClosestGroupDistance)
				.withWidget(BuiltInWidgets.kTextView).withPosition(6, 6).withSize(2, 1);
		tab.addString("Selected Reason", this::selectedReasonForDashboard).withWidget(BuiltInWidgets.kTextView)
				.withPosition(0, 7).withSize(8, 1);
	}

	private String selectedReasonForDashboard() {
		if (DriverStation.isDisabled()) {
			return "Disabled: " + pieceDetection.getSelectedGroupReason();
		}

		return pieceDetection.getSelectedGroupReason();
	}

	private void publishStaticLinks() {
		if (PieceDetectionConstants.ENABLE_FRONT_GAME_PIECE_CAMERA) {
			SmartDashboard.putString(SMARTDASHBOARD_PREFIX + "Front Camera Dashboard URL",
					PieceDetectionConstants.FRONT_GAME_PIECE_CAMERA_DASHBOARD_URI.toString());
			SmartDashboard.putString(SMARTDASHBOARD_PREFIX + "Front Camera Video URL",
					PieceDetectionConstants.FRONT_GAME_PIECE_CAMERA_VIDEO_URI.toString());
		}

		if (PieceDetectionConstants.ENABLE_REAR_GAME_PIECE_CAMERA) {
			SmartDashboard.putString(SMARTDASHBOARD_PREFIX + "Rear Camera Dashboard URL",
					PieceDetectionConstants.REAR_GAME_PIECE_CAMERA_DASHBOARD_URI.toString());
			SmartDashboard.putString(SMARTDASHBOARD_PREFIX + "Rear Camera Video URL",
					PieceDetectionConstants.REAR_GAME_PIECE_CAMERA_VIDEO_URI.toString());
		}
	}
}
