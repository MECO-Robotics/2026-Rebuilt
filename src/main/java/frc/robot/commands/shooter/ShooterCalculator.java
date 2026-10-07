package frc.robot.commands.shooter;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FieldConstants.Hub;
import frc.robot.constants.subsystems.ShooterConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.position_joint.PositionJoint;
import org.littletonrobotics.junction.Logger;

/** Shot calculators that derive hood and flywheel setpoints from robot pose. */
public class ShooterCalculator {
	/** Calculates a stationary shot solution for an explicit target. */
	public static ShotSolution calculate(Pose2d robotPose, Translation2d targetPosition) {
		return calculate(robotPose, targetPosition, frc.robot.constants.Constants.currentMode);
	}

	/**
	 * Explicit mode allows real aiming to be regression-tested independently of CAD
	 * simulation.
	 */
	public static ShotSolution calculate(Pose2d robotPose, Translation2d targetPosition,
			frc.robot.constants.Constants.Mode mode) {
		Pose2d shooterPose = robotPose
				.transformBy(new Transform2d(ShooterConstants.SHOOTER_EXIT_TRANSLATION, Rotation2d.kZero));
		Translation2d shooterToTarget = targetPosition.minus(shooterPose.getTranslation());
		double distanceMeters = shooterToTarget.getNorm();
		boolean calibrated = distanceMeters >= ShooterConstants.MIN_CALIBRATED_DISTANCE_METERS
				&& distanceMeters <= ShooterConstants.MAX_CALIBRATED_DISTANCE_METERS;
		var distance = Meters.of(distanceMeters);

		return new ShotSolution(distanceMeters,
				shooterToTarget.getAngle().minus(ShooterConstants.aimingYawForMode(mode)),
				ShooterConstants.hoodMap.get(distance).in(Units.Rotations),
				ShooterConstants.shooterVelocityMap.get(distance).in(Units.RevolutionsPerSecond), calibrated);
	}

	/**
	 * Calculates either a calibrated range-based hub shot or a carpet-landing ferry
	 * shot while preserving pose-based aiming to the selected field target.
	 */
	public static ShotSolution calculate(Pose2d robotPose, ShotTarget target) {
		ShotSolution aimedSolution = calculate(robotPose, target.position());
		if (target.mode() == ShotTarget.Mode.FERRY) {
			FerryShotCalculator.FerryShotPlan plan = FerryShotCalculator.calculate(robotPose, target);
			return new ShotSolution(aimedSolution.distanceMeters(), aimedSolution.targetHeading(), plan.hoodRotations(),
					plan.flywheelRps(), plan.safe());
		}
		return aimedSolution;
	}

	/**
	 * Calculates the ideal flywheel speed required for a ferry shot to land on the
	 * carpet at the selected horizontal distance.
	 */
	static double calculateFerryFlywheelRps(double distanceMeters, double hoodRotations) {
		return FerryShotCalculator.calculateFlywheelRps(distanceMeters, hoodRotations);
	}

	/** Calculates a shot solution for the current alliance hub. */
	public static ShotSolution calculate(Pose2d robotPose) {
		return calculate(robotPose, Hub.hubPosition());
	}

	/**
	 * Uses the interpolated hood and flywheel maps for continuous range-based
	 * aiming.
	 */
	public static Command calculateAndShoot(CommandSwerveDrivetrain drive, PositionJoint hood, Flywheel shooter) {
		return PositionJoint.holdPosition(hood, () -> currentSolution(drive).hoodRotations())
				.alongWith(Flywheel.holdVelocity(shooter, () -> currentSolution(drive).flywheelRotationsPerSecond()));
	}

	private static ShotSolution currentSolution(CommandSwerveDrivetrain drive) {
		ShotSolution solution = calculate(drive.getState().Pose);
		Logger.recordOutput("Shooter/DistanceToHubMeters", solution.distanceMeters());
		Logger.recordOutput("Shooter/SolutionCalibrated", solution.calibrated());
		return solution;
	}
}
