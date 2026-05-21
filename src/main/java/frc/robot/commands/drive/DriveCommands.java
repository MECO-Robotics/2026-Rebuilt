package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.FieldConstants.Hub;
import frc.robot.constants.FieldConstants.Obstacles;
import frc.robot.constants.drive.DrivetrainConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.util.mechanical_advantage.LoggedTunableNumber;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;

/** Factory methods for drivetrain teleop and characterization commands. */
public class DriveCommands {
	private static final LoggedTunableNumber DEADBAND = new LoggedTunableNumber("DriveCommands/Deadband", 0.05);
	private static final LoggedTunableNumber ANGLE_KP = new LoggedTunableNumber("DriveCommands/Angle_KP", 15.0);
	private static final LoggedTunableNumber ANGLE_KD = new LoggedTunableNumber("DriveCommands/Angle_KD", 0.01);
	private static final LoggedTunableNumber ANGLE_MAX_VELOCITY = new LoggedTunableNumber(
			"DriveCommands/Angle_Max_Velocity", 25.0);
	private static final LoggedTunableNumber DRIVE_TO_POSE_TRANSLATION_KP = new LoggedTunableNumber(
			"DriveCommands/DriveToPose/Translation_KP", 3.0);
	private static final LoggedTunableNumber DRIVE_TO_POSE_TRANSLATION_KD = new LoggedTunableNumber(
			"DriveCommands/DriveToPose/Translation_KD", 0.0);
	private static final LoggedTunableNumber DRIVE_TO_POSE_MAX_ACCEL = new LoggedTunableNumber(
			"DriveCommands/DriveToPose/MaxAccel", 2.0);
	private static final LoggedTunableNumber DRIVE_TO_POSE_ROTATION_KP = new LoggedTunableNumber(
			"DriveCommands/DriveToPose/Rotation_KP", 5.0);
	private static final LoggedTunableNumber DRIVE_TO_POSE_ROTATION_KD = new LoggedTunableNumber(
			"DriveCommands/DriveToPose/Rotation_KD", 0.0);
	private static final LoggedTunableNumber DRIVE_TO_POSE_MAX_SPEED = new LoggedTunableNumber(
			"DriveCommands/DriveToPose/MaxSpeed", 2.0);
	private static final LoggedTunableNumber DRIVE_TO_POSE_MAX_ANGULAR_RATE = new LoggedTunableNumber(
			"DriveCommands/DriveToPose/MaxAngularRate", 4.0);
	private static final LoggedTunableNumber DRIVE_TO_POSE_MAX_ANGULAR_ACCEL = new LoggedTunableNumber(
			"DriveCommands/DriveToPose/MaxAngularAccel", 8.0);
	private static final LoggedTunableNumber DRIVE_TO_POSE_TRANSLATION_TOLERANCE = new LoggedTunableNumber(
			"DriveCommands/DriveToPose/TranslationTolerance", 0.05);
	private static final LoggedTunableNumber DRIVE_TO_POSE_ROTATION_TOLERANCE = new LoggedTunableNumber(
			"DriveCommands/DriveToPose/RotationTolerance", Math.toRadians(2.0));
	private static final LoggedTunableNumber DRIVE_TO_POSE_PATHFIND_HANDOFF_DISTANCE = new LoggedTunableNumber(
			"DriveCommands/DriveToPose/PathfindHandoffDistance", 3.0);
	private static final LoggedTunableNumber DRIVE_TO_POSE_DIRECT_PATH_DISTANCE = new LoggedTunableNumber(
			"DriveCommands/DriveToPose/DirectPathDistance", 1.0);
	// private static final LoggedTunableNumber FF_START_DELAY = new
	// LoggedTunableNumber("DriveCommands/FF_Start_Delay",
	// 2.0); // Secs
	// private static final LoggedTunableNumber FF_RAMP_RATE = new
	// LoggedTunableNumber("DriveCommands/FF_Ramp_Rate", 0.1); // Volts/Sec
	// private static final LoggedTunableNumber WHEEL_RADIUS_MAX_VELOCITY = new
	// LoggedTunableNumber(
	// "DriveCommands/Wheel_Radius_Max_Velocity", 0.25); // Rad/Sec
	// private static final LoggedTunableNumber WHEEL_RADIUS_RAMP_RATE = new
	// LoggedTunableNumber(
	// "DriveCommands/Wheel_Radius_Ramp_Rate", 0.05); // Rad/Sec^2

	private DriveCommands() {
	}

	/**
	 * Maps joystick x/y inputs into a deadbanded, squared translation command.
	 */
	private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
		// Apply deadband
		double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND.get());
		Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

		// Square magnitude for more precise control
		linearMagnitude = linearMagnitude * linearMagnitude;

		// Return new linear velocity
		return new Pose2d(new Translation2d(), linearDirection)
				.transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d())).getTranslation();
	}

	/**
	 * Field relative drive command using joystick for linear control and PID for
	 * angular control. Possible use cases include snapping to an angle, aiming at a
	 * vision target, or controlling absolute rotation with a joystick.
	 */
	public static Command joystickDriveAtAngle(CommandSwerveDrivetrain drivetrain, DoubleSupplier xSupplier,
			DoubleSupplier ySupplier, Supplier<Rotation2d> rotationSupplier, double maxSpeed) {

		SwerveRequest.FieldCentricFacingAngle driveAtAngleRequest = new SwerveRequest.FieldCentricFacingAngle()
				.withDeadband(maxSpeed * DEADBAND.get())
				.withRotationalDeadband(DrivetrainConstants.MAX_ANGULAR_RATE * DEADBAND.get())
				.withDriveRequestType(DriveRequestType.Velocity);
		driveAtAngleRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

		// Construct command
		return Commands.run(() -> {
			driveAtAngleRequest.HeadingController.setPID(ANGLE_KP.get(), 0.0, ANGLE_KD.get());
			// Get linear velocity
			Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(),
					ySupplier.getAsDouble()).times(maxSpeed);
			double maxAngularRate = Math.min(ANGLE_MAX_VELOCITY.get(), DrivetrainConstants.MAX_ANGULAR_RATE);

			drivetrain.setControl(
					driveAtAngleRequest.withVelocityX(linearVelocity.getX()).withVelocityY(linearVelocity.getY())
							.withTargetDirection(rotationSupplier.get()).withMaxAbsRotationalRate(maxAngularRate));

		}, drivetrain)

				// Reset PID controller when command starts
				.beforeStarting(() -> driveAtAngleRequest.HeadingController.reset());

	}

	/**
	 * Drives to a fixed field-relative pose using the drivetrain pose estimator.
	 */
	public static Command driveToPose(CommandSwerveDrivetrain drive, Pose2d targetPose) {
		return driveToPose(drive, () -> targetPose);
	}

	/**
	 * Drives to a supplied field-relative pose using the drivetrain pose estimator.
	 */
	public static Command driveToPose(CommandSwerveDrivetrain drive, Supplier<Pose2d> targetPoseSupplier) {
		ProfiledPIDController xController = new ProfiledPIDController(DRIVE_TO_POSE_TRANSLATION_KP.get(), 0.0,
				DRIVE_TO_POSE_TRANSLATION_KD.get(),
				new TrapezoidProfile.Constraints(DRIVE_TO_POSE_MAX_SPEED.get(), DRIVE_TO_POSE_MAX_ACCEL.get()));
		ProfiledPIDController yController = new ProfiledPIDController(DRIVE_TO_POSE_TRANSLATION_KP.get(), 0.0,
				DRIVE_TO_POSE_TRANSLATION_KD.get(),
				new TrapezoidProfile.Constraints(DRIVE_TO_POSE_MAX_SPEED.get(), DRIVE_TO_POSE_MAX_ACCEL.get()));
		ProfiledPIDController thetaController = new ProfiledPIDController(DRIVE_TO_POSE_ROTATION_KP.get(), 0.0,
				DRIVE_TO_POSE_ROTATION_KD.get(), new TrapezoidProfile.Constraints(DRIVE_TO_POSE_MAX_ANGULAR_RATE.get(),
						DRIVE_TO_POSE_MAX_ANGULAR_ACCEL.get()));
		SwerveRequest.ApplyRobotSpeeds driveRequest = new SwerveRequest.ApplyRobotSpeeds()
				.withDriveRequestType(DriveRequestType.Velocity);
		Pose2d[] targetPose = new Pose2d[]{targetPoseSupplier.get()};

		thetaController.enableContinuousInput(-Math.PI, Math.PI);

		return Commands.run(() -> {
			Pose2d currentPose = drive.getState().Pose;
			targetPose[0] = targetPoseSupplier.get();

			xController.setP(DRIVE_TO_POSE_TRANSLATION_KP.get());
			xController.setD(DRIVE_TO_POSE_TRANSLATION_KD.get());
			yController.setP(DRIVE_TO_POSE_TRANSLATION_KP.get());
			yController.setD(DRIVE_TO_POSE_TRANSLATION_KD.get());
			xController.setConstraints(
					new TrapezoidProfile.Constraints(DRIVE_TO_POSE_MAX_SPEED.get(), DRIVE_TO_POSE_MAX_ACCEL.get()));
			yController.setConstraints(
					new TrapezoidProfile.Constraints(DRIVE_TO_POSE_MAX_SPEED.get(), DRIVE_TO_POSE_MAX_ACCEL.get()));
			thetaController.setP(DRIVE_TO_POSE_ROTATION_KP.get());
			thetaController.setD(DRIVE_TO_POSE_ROTATION_KD.get());
			thetaController.setConstraints(new TrapezoidProfile.Constraints(DRIVE_TO_POSE_MAX_ANGULAR_RATE.get(),
					DRIVE_TO_POSE_MAX_ANGULAR_ACCEL.get()));

			xController.setTolerance(DRIVE_TO_POSE_TRANSLATION_TOLERANCE.get());
			yController.setTolerance(DRIVE_TO_POSE_TRANSLATION_TOLERANCE.get());
			thetaController.setTolerance(DRIVE_TO_POSE_ROTATION_TOLERANCE.get());

			double maxSpeed = Math.min(DRIVE_TO_POSE_MAX_SPEED.get(), DrivetrainConstants.MAX_SPEED);
			double maxAngularRate = Math.min(DRIVE_TO_POSE_MAX_ANGULAR_RATE.get(),
					DrivetrainConstants.MAX_ANGULAR_RATE);
			double xFeedback = xController.calculate(currentPose.getX(), targetPose[0].getX());
			double yFeedback = yController.calculate(currentPose.getY(), targetPose[0].getY());
			double thetaFeedback = thetaController.calculate(currentPose.getRotation().getRadians(),
					targetPose[0].getRotation().getRadians());
			double xVelocity = MathUtil.clamp(xFeedback + xController.getSetpoint().velocity, -maxSpeed, maxSpeed);
			double yVelocity = MathUtil.clamp(yFeedback + yController.getSetpoint().velocity, -maxSpeed, maxSpeed);
			double thetaVelocity = MathUtil.clamp(thetaFeedback + thetaController.getSetpoint().velocity,
					-maxAngularRate, maxAngularRate);

			if (Math.abs(currentPose.getX() - targetPose[0].getX()) <= DRIVE_TO_POSE_TRANSLATION_TOLERANCE.get()) {
				xVelocity = 0.0;
			}
			if (Math.abs(currentPose.getY() - targetPose[0].getY()) <= DRIVE_TO_POSE_TRANSLATION_TOLERANCE.get()) {
				yVelocity = 0.0;
			}
			if (Math.abs(MathUtil.angleModulus(currentPose.getRotation().minus(targetPose[0].getRotation())
					.getRadians())) <= DRIVE_TO_POSE_ROTATION_TOLERANCE.get()) {
				thetaVelocity = 0.0;
			}

			ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity,
					thetaVelocity, currentPose.getRotation());
			drive.setControl(driveRequest.withSpeeds(ChassisSpeeds.discretize(robotRelativeSpeeds, 0.020)));
		}, drive).beforeStarting(() -> {
			Pose2d currentPose = drive.getState().Pose;
			targetPose[0] = targetPoseSupplier.get();
			xController.setTolerance(DRIVE_TO_POSE_TRANSLATION_TOLERANCE.get());
			yController.setTolerance(DRIVE_TO_POSE_TRANSLATION_TOLERANCE.get());
			thetaController.setTolerance(DRIVE_TO_POSE_ROTATION_TOLERANCE.get());
			ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(drive.getState().Speeds,
					currentPose.getRotation());
			xController.reset(currentPose.getX(), fieldRelativeSpeeds.vxMetersPerSecond);
			yController.reset(currentPose.getY(), fieldRelativeSpeeds.vyMetersPerSecond);
			thetaController.reset(currentPose.getRotation().getRadians(),
					drive.getState().Speeds.omegaRadiansPerSecond);
		}).until(() -> atDriveToPoseGoal(drive.getState().Pose, targetPose[0]))
				.finallyDo((interrupted) -> drive.stop());
	}

	/**
	 * Dynamically plans and follows a path to a supplied field-relative pose.
	 */
	public static Command pathfindToPose(CommandSwerveDrivetrain drive, Supplier<Pose2d> targetPoseSupplier) {
		return Commands.defer(() -> {
			Pathfinding.ensureInitialized();
			Pathfinding.setDynamicObstacles(Obstacles.pathfindingObstacles(), drive.getState().Pose.getTranslation());
			return AutoBuilder.pathfindToPose(targetPoseSupplier.get(), driveToPoseConstraints());
		}, Set.of(drive)).finallyDo((interrupted) -> drive.stop());
	}

	/**
	 * Ranger-style autodrive: pathfind near the target, follow a short final path
	 * from the robot's current motion heading, then run profiled pose alignment.
	 */
	public static Command pathfindAndAlignToPose(CommandSwerveDrivetrain drive, Supplier<Pose2d> targetPoseSupplier) {
		return Commands.defer(() -> {
			Pose2d targetPose = targetPoseSupplier.get();
			double distanceToTarget = drive.getState().Pose.getTranslation().getDistance(targetPose.getTranslation());
			if (distanceToTarget <= DRIVE_TO_POSE_DIRECT_PATH_DISTANCE.get()) {
				return Commands.none();
			}

			Pathfinding.ensureInitialized();
			Pathfinding.setDynamicObstacles(Obstacles.pathfindingObstacles(), drive.getState().Pose.getTranslation());
			return AutoBuilder.pathfindToPose(targetPose, driveToPoseConstraints())
					.until(() -> drive.getState().Pose.getTranslation().getDistance(targetPoseSupplier.get()
							.getTranslation()) <= DRIVE_TO_POSE_PATHFIND_HANDOFF_DISTANCE.get());
		}, Set.of(drive))
				.andThen(Commands.defer(() -> followDirectFinalPath(drive, targetPoseSupplier.get()), Set.of(drive)))
				.andThen(driveToPose(drive, targetPoseSupplier).withTimeout(5.0))
				.finallyDo((interrupted) -> drive.stop());
	}

	private static Command followDirectFinalPath(CommandSwerveDrivetrain drive, Pose2d targetPose) {
		Pose2d currentPose = drive.getState().Pose;
		if (currentPose.getTranslation().getDistance(targetPose.getTranslation()) <= DRIVE_TO_POSE_DIRECT_PATH_DISTANCE
				.get()) {
			return Commands.none();
		}

		Rotation2d heading = finalApproachHeading(drive, currentPose, targetPose);
		ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(drive.getState().Speeds,
				currentPose.getRotation());
		PathPlannerPath path = new PathPlannerPath(
				PathPlannerPath.waypointsFromPoses(new Pose2d(currentPose.getTranslation(), heading), targetPose),
				driveToPoseConstraints(),
				new IdealStartingState(
						Math.hypot(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond),
						heading),
				new GoalEndState(0.0, targetPose.getRotation()));
		path.preventFlipping = true;
		return AutoBuilder.followPath(path);
	}

	private static Rotation2d finalApproachHeading(CommandSwerveDrivetrain drive, Pose2d currentPose,
			Pose2d targetPose) {
		ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(drive.getState().Speeds,
				currentPose.getRotation());
		Translation2d velocity = new Translation2d(fieldRelativeSpeeds.vxMetersPerSecond,
				fieldRelativeSpeeds.vyMetersPerSecond);
		if (velocity.getNorm() > 0.10) {
			return velocity.getAngle();
		}
		return targetPose.getTranslation().minus(currentPose.getTranslation()).getAngle();
	}

	private static PathConstraints driveToPoseConstraints() {
		return new PathConstraints(Math.min(DRIVE_TO_POSE_MAX_SPEED.get(), DrivetrainConstants.MAX_SPEED),
				DRIVE_TO_POSE_MAX_ACCEL.get(),
				Math.min(DRIVE_TO_POSE_MAX_ANGULAR_RATE.get(), DrivetrainConstants.MAX_ANGULAR_RATE),
				DRIVE_TO_POSE_MAX_ANGULAR_ACCEL.get());
	}

	private static boolean atDriveToPoseGoal(Pose2d currentPose, Pose2d targetPose) {
		return currentPose.getTranslation()
				.getDistance(targetPose.getTranslation()) <= DRIVE_TO_POSE_TRANSLATION_TOLERANCE.get()
				&& Math.abs(MathUtil.angleModulus(currentPose.getRotation().minus(targetPose.getRotation())
						.getRadians())) <= DRIVE_TO_POSE_ROTATION_TOLERANCE.get();
	}

	public static Command joystickAimToHub(CommandSwerveDrivetrain drive, DoubleSupplier xSupplier,
			DoubleSupplier ySupplier, double maxSpeed) {

		Supplier<Rotation2d> angleToHub = () -> Hub.hubPosition().minus(drive.getState().Pose.getTranslation())
				.getAngle().plus(getHubAimOffset());

		return joystickDriveAtAngle(drive, xSupplier, ySupplier, angleToHub, maxSpeed);
	}

	/** Auto aim to the hub. */
	public static Command autoAimToHub(CommandSwerveDrivetrain drive, double maxspeed) {
		Supplier<Rotation2d> angleToHub = () -> Hub.hubPosition().minus(drive.getState().Pose.getTranslation())
				.getAngle().plus(getHubAimOffset());

		return joystickDriveAtAngle(drive, () -> 0.0, () -> 0.0, angleToHub, maxspeed);
	}

	private static Rotation2d getHubAimOffset() {
		return DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue
				? Rotation2d.kZero
				: Rotation2d.k180deg;
	}

	// // public static Command azimuthTuning()

	public static Command resetHeading(CommandSwerveDrivetrain drive) {
		return Commands.runOnce(() -> drive.seedFieldCentric(Rotation2d.fromDegrees(0)));
	}
}
