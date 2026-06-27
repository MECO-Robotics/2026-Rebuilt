package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.shooter.ShooterCalculator;
import frc.robot.commands.shooter.ShooterCommands;
import frc.robot.constants.FieldConstants.Hub;
import frc.robot.constants.drive.DrivetrainConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.piece_detection.PieceDetection;
import frc.robot.subsystems.position_joint.PositionJoint;
import frc.robot.subsystems.vision.Vision;
import java.util.Set;

public final class GamePieceAutonomyCommands {
	public static final double FULL_MATCH_RUNTIME_SECONDS = 160.0;

	private static final PathConstraints NICE_PATH_CONSTRAINTS = new PathConstraints(2.0, 1.5, 3.0, 4.0);
	private static final double PICKUP_TIMEOUT_SECONDS = 2.0;
	private static final double APRILTAG_LINEUP_TIMEOUT_SECONDS = 0.75;
	private static final double APRILTAG_FRESH_SECONDS = 0.5;
	private static final double SHOOT_TIMEOUT_SECONDS = 3.0;
	private static final double HUB_STANDOFF_METERS = 2.0;

	private GamePieceAutonomyCommands() {
	}

	public static Command collectSelectedGroup(CommandSwerveDrivetrain drivetrain, PieceDetection pieceDetection,
			PositionJoint intakeRack, Flywheel intakeRoller, Flywheel conveyor) {
		return Commands.either(
				Commands.defer(
						() -> Commands
								.deadline(pathfindToSelectedPiece(drivetrain, pieceDetection),
										IntakeCommands.deployAndSpinIntake(intakeRack, intakeRoller))
								.andThen(Commands.deadline(Commands.waitSeconds(PICKUP_TIMEOUT_SECONDS),
										IntakeCommands.stowIntake(intakeRack, intakeRoller, conveyor))),
						Set.of(drivetrain, intakeRack, intakeRoller, conveyor)),
				Commands.none(), pieceDetection::pieceDetected);
	}

	public static Command collectAndShootCycle(CommandSwerveDrivetrain drivetrain, PieceDetection pieceDetection,
			Vision vision, PositionJoint intakeRack, Flywheel intakeRoller, Flywheel conveyor, Flywheel bottomIndexer,
			Flywheel topIndexer, Flywheel shooterFlywheel, PositionJoint hood) {
		return Commands.either(
				Commands.sequence(collectSelectedGroup(drivetrain, pieceDetection, intakeRack, intakeRoller, conveyor),
						pathfindToHubShotPose(drivetrain),
						lineUpWithAprilTags(drivetrain, vision, hood, shooterFlywheel),
						Commands.deadline(Commands.waitSeconds(SHOOT_TIMEOUT_SECONDS),
								DriveCommands.autoAimToHub(drivetrain, DrivetrainConstants.MAX_SPEED),
								ShooterCalculator.calculateAndShoot(drivetrain, hood, shooterFlywheel),
								ShooterCommands.feedRollers(bottomIndexer, topIndexer, conveyor)),
						ShooterCommands.shooterIdle(shooterFlywheel, hood).withTimeout(0.1),
						ShooterCommands.idleRollers(bottomIndexer, topIndexer, conveyor).withTimeout(0.1)),
				Commands.waitSeconds(0.1), pieceDetection::pieceDetected);
	}

	public static Command collectAndShootUntilInterrupted(CommandSwerveDrivetrain drivetrain,
			PieceDetection pieceDetection, Vision vision, PositionJoint intakeRack, Flywheel intakeRoller,
			Flywheel conveyor, Flywheel bottomIndexer, Flywheel topIndexer, Flywheel shooterFlywheel,
			PositionJoint hood) {
		return collectAndShootCycle(drivetrain, pieceDetection, vision, intakeRack, intakeRoller, conveyor,
				bottomIndexer, topIndexer, shooterFlywheel, hood).repeatedly();
	}

	private static Command lineUpWithAprilTags(CommandSwerveDrivetrain drivetrain, Vision vision, PositionJoint hood,
			Flywheel shooterFlywheel) {
		return Commands.deadline(
				Commands.waitUntil(() -> vision.hasRecentAprilTagPose(APRILTAG_FRESH_SECONDS))
						.withTimeout(APRILTAG_LINEUP_TIMEOUT_SECONDS),
				DriveCommands.autoAimToHub(drivetrain, DrivetrainConstants.MAX_SPEED),
				ShooterCalculator.calculateAndShoot(drivetrain, hood, shooterFlywheel));
	}

	private static Command pathfindToSelectedPiece(CommandSwerveDrivetrain drivetrain, PieceDetection pieceDetection) {
		Pose2d piecePose = pieceDetection.getPiecePose().toPose2d();
		Pose2d currentPose = drivetrain.getState().Pose;
		Translation2d targetTranslation = piecePose.getTranslation();
		Rotation2d targetHeading = targetTranslation.minus(currentPose.getTranslation()).getAngle();
		return AutoBuilder.pathfindToPose(new Pose2d(targetTranslation, targetHeading), NICE_PATH_CONSTRAINTS);
	}

	private static Command pathfindToHubShotPose(CommandSwerveDrivetrain drivetrain) {
		Pose2d currentPose = drivetrain.getState().Pose;
		Translation2d hubPosition = Hub.hubPosition();
		Translation2d awayFromHub = currentPose.getTranslation().minus(hubPosition);
		if (awayFromHub.getNorm() < 0.1) {
			awayFromHub = new Translation2d(1.0, 0.0);
		}
		Translation2d shotTranslation = hubPosition
				.plus(awayFromHub.div(awayFromHub.getNorm()).times(HUB_STANDOFF_METERS));
		Rotation2d shotHeading = hubPosition.minus(shotTranslation).getAngle();
		return AutoBuilder.pathfindToPose(new Pose2d(shotTranslation, shotHeading), NICE_PATH_CONSTRAINTS);
	}
}
