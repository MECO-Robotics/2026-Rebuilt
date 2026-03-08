package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants.Hub;
import frc.robot.constants.drive.DriveConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.mechanical_advantage.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/** Factory methods for drivetrain teleop and characterization commands. */
public class DriveCommands {
	private static final LoggedTunableNumber DEADBAND = new LoggedTunableNumber("DriveCommands/Deadband", 0.05);
	private static final LoggedTunableNumber ANGLE_KP = new LoggedTunableNumber("DriveCommands/Angle_KP", 40.0);
	private static final LoggedTunableNumber ANGLE_KD = new LoggedTunableNumber("DriveCommands/Angle_KD", 0.0);
	private static final LoggedTunableNumber ANGLE_MAX_VELOCITY = new LoggedTunableNumber(
			"DriveCommands/Angle_Max_Velocity", 25.0);
	private static final LoggedTunableNumber ANGLE_MAX_ACCELERATION = new LoggedTunableNumber(
			"DriveCommands/Angle_Max_Acceleration", 20.0);
	// private static final LoggedTunableNumber FF_START_DELAY = new LoggedTunableNumber("DriveCommands/FF_Start_Delay",
	// 		2.0); // Secs
	// private static final LoggedTunableNumber FF_RAMP_RATE = new LoggedTunableNumber("DriveCommands/FF_Ramp_Rate", 0.1); // Volts/Sec
	// private static final LoggedTunableNumber WHEEL_RADIUS_MAX_VELOCITY = new LoggedTunableNumber(
	// 		"DriveCommands/Wheel_Radius_Max_Velocity", 0.25); // Rad/Sec
	// private static final LoggedTunableNumber WHEEL_RADIUS_RAMP_RATE = new LoggedTunableNumber(
	// 		"DriveCommands/Wheel_Radius_Ramp_Rate", 0.05); // Rad/Sec^2

	private DriveCommands() {
	}

	/** Maps joystick x/y inputs into a deadbanded, squared translation command. */
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
	 * Field relative drive command using two joysticks (controlling linear and
	 * angular velocities).
	 */
	public static Command joystickDrive(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier,
			DoubleSupplier omegaSupplier) {
		return Commands.run(() -> {
			// Get linear velocity
			Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(),
					ySupplier.getAsDouble());

			// Apply rotation deadband
			double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND.get());

			// Square rotation value for more precise control
			omega = Math.copySign(omega * omega, omega);

			// Convert to field relative speeds & send command
			ChassisSpeeds speeds = new ChassisSpeeds(linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
					linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
					omega * DriveConstants.spinMultipler * drive.getMaxAngularSpeedRadPerSec());
			boolean isFlipped = Constants.isAllianceRed();
			speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds,
					isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation());
			drive.runVelocity(speeds);
		}, drive);
	}

	/**
	 * Field relative drive command using two joysticks (controlling linear and
	 * angular velocities).
	 *
	 * @param drive
	 *            The drive subsystem.
	 * @param xSupplier
	 *            The supplier for the x-axis value of the joystick.
	 * @param ySupplier
	 *            The supplier for the y-axis value of the joystick.
	 * @param omegaSupplier
	 *            The supplier for the angular velocity.
	 * @return The command.
	 */
	public static Command joystickDriveRobotRelative(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier,
			DoubleSupplier omegaSupplier) {
		return Commands.run(() -> {
			// Get linear velocity
			Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(),
					ySupplier.getAsDouble());

			// Apply rotation deadband
			double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND.get());

			// Square rotation value for more precise control
			omega = Math.copySign(omega * omega, omega);

			// Convert to field relative speeds & send command
			ChassisSpeeds speeds = new ChassisSpeeds(linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
					linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
					omega * drive.getMaxAngularSpeedRadPerSec());
			// boolean isFlipped =
			// DriverStation.getAlliance().isPresent()
			// && DriverStation.getAlliance().get() == Alliance.Red;
			drive.runVelocity(speeds);
		}, drive);
	}

	/**
	 * Field relative drive command using joystick for linear control and PID for
	 * angular control. Possible use cases include snapping to an angle, aiming at a
	 * vision target, or controlling absolute rotation with a joystick.
	 */
	public static Command joystickDriveAtAngle(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier,
			Supplier<Rotation2d> rotationSupplier) {

		// Create PID controller
		ProfiledPIDController angleController = new ProfiledPIDController(ANGLE_KP.get(), 0.0, ANGLE_KD.get(),
				new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY.get(), ANGLE_MAX_ACCELERATION.get()));
		angleController.enableContinuousInput(-Math.PI, Math.PI);

		// Construct command
		return Commands.run(() -> {
			// Get linear velocity
			Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(),
					ySupplier.getAsDouble());

			// Calculate angular speed
			double omega = angleController.calculate(drive.getRotation().getRadians(),
					rotationSupplier.get().getRadians());

			// Convert to field relative speeds & send command
			ChassisSpeeds speeds = new ChassisSpeeds(linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
					linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(), omega);
			boolean isFlipped = Constants.isAllianceRed();
			speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds,
					isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation());
			drive.runVelocity(speeds);
		}, drive)

				// Reset PID controller when command starts
				.beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
	}

	// TODO: verify that distance to angle here is correct
	public static Command joystickAimToHub(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {

		Supplier<Rotation2d> angleToHub = () -> Hub.hubPosition().minus(drive.getPose().getTranslation()).getAngle();

		return joystickDriveAtAngle(drive, xSupplier, ySupplier, angleToHub);
	}

	/** Auto aim to the hub. */
	public static Command autoAimToHub(Drive drive) {
		Supplier<Rotation2d> angleToHub = () -> Hub.hubPosition().minus(drive.getPose().getTranslation()).getAngle();

		return joystickDriveAtAngle(drive, () -> 0.0, () -> 0.0, angleToHub);
	}

	// public static Command azimuthTuning()

}
