package frc.robot.commands.shooter;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FieldConstants.Hub;
import frc.robot.constants.subsystems.ShooterConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.position_joint.PositionJoint;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Shot calculators that derive hood and flywheel setpoints from robot pose. */
public class ShooterCalculator {
	/** Translation from robot origin to the shooter exit point used for range. */
	public static final Translation2d robotToShooter = new Translation2d(-.19, 0);

	/**
	 * Uses the interpolated hood and flywheel maps for continuous range-based
	 * aiming.
	 */
	public static Command calculateAndShoot(CommandSwerveDrivetrain drive, PositionJoint hood, Flywheel shooter) {
		Supplier<Distance> distance = () -> {
			Pose2d shooterPosition = drive.getState().Pose
					.transformBy(new Transform2d(robotToShooter, Rotation2d.kZero));
			Distance distanceToHub = Meters.of(Hub.hubPosition().getDistance(shooterPosition.getTranslation()));
			Logger.recordOutput("Shooter/DistanceToHubMeters", distanceToHub.in(Units.Meters));
			return distanceToHub;
		};

		DoubleSupplier hoodPosition = () -> ShooterConstants.hoodMap.get(distance.get()).in(Units.Rotations);

		DoubleSupplier shooterVelocity = () -> ShooterConstants.shooterVelocityMap.get(distance.get())
				.in(Units.RevolutionsPerSecond);

		return PositionJoint.setPosition(hood, hoodPosition).alongWith(Flywheel.setVelocity(shooter, shooterVelocity));
	}

	/** Uses quadratic regression curves instead of the interpolating maps. */
	public static Command calculateAndShootRegression(CommandSwerveDrivetrain drive, PositionJoint hood,
			Flywheel shooter) {
		Supplier<Distance> distance = () -> {
			Pose2d shooterPosition = drive.getState().Pose
					.transformBy(new Transform2d(robotToShooter, Rotation2d.kZero));
			Distance distanceToHub = Meters.of(Hub.hubPosition().getDistance(shooterPosition.getTranslation()));
			Logger.recordOutput("Shooter/DistanceToHubMeters", distanceToHub.in(Units.Meters));
			return distanceToHub;
		};

		DoubleSupplier hoodPosition = () -> {
			double x = distance.get().in(Units.Inches);
			double[] k = ShooterConstants.kHOOODREGCALC;
			return -k[0] + k[1] * x + k[2] * x * x;
		};

		DoubleSupplier shooterVelocity = () -> {
			double x = distance.get().in(Units.Inches);
			double[] k = ShooterConstants.kSHOOTERVELREGCALC;
			return k[0] + k[1] * x + k[2] * x * x;
		};

		return PositionJoint.setPosition(hood, hoodPosition).alongWith(Flywheel.setVelocity(shooter, shooterVelocity));
	}
}
