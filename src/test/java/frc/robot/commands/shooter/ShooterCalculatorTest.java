package frc.robot.commands.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import org.junit.jupiter.api.Test;

class ShooterCalculatorTest {
	private static final double EPSILON = 1e-9;

	@Test
	void calculatesExactCloseCalibrationPointFromShooterExit() {
		double distance = Units.inchesToMeters(58.0);
		ShotSolution solution = ShooterCalculator.calculate(new Pose2d(0.19, 0.0, Rotation2d.kZero),
				new Translation2d(distance, 0.0));

		assertEquals(distance, solution.distanceMeters(), EPSILON);
		assertEquals(0.005, solution.hoodRotations(), EPSILON);
		assertEquals(28.5, solution.flywheelRotationsPerSecond(), EPSILON);
		assertEquals(0.0, solution.targetHeading().getRadians(), EPSILON);
		assertTrue(solution.calibrated());
	}

	@Test
	void interpolatesBetweenCalibrationPoints() {
		double distance = Units.inchesToMeters((58.0 + 114.25) / 2.0);
		ShotSolution solution = ShooterCalculator.calculate(new Pose2d(0.19, 0.0, Rotation2d.kZero),
				new Translation2d(distance, 0.0));

		assertEquals(0.015, solution.hoodRotations(), EPSILON);
		assertEquals(31.25, solution.flywheelRotationsPerSecond(), EPSILON);
		assertTrue(solution.calibrated());
	}

	@Test
	void flagsDistancesOutsideCalibrationRange() {
		ShotSolution tooClose = ShooterCalculator.calculate(new Pose2d(0.19, 0.0, Rotation2d.kZero),
				new Translation2d(Units.inchesToMeters(40.0), 0.0));
		ShotSolution tooFar = ShooterCalculator.calculate(new Pose2d(0.19, 0.0, Rotation2d.kZero),
				new Translation2d(Units.inchesToMeters(250.0), 0.0));

		assertFalse(tooClose.calibrated());
		assertFalse(tooFar.calibrated());
	}

	@Test
	void targetDirectionIsGeometricAndNotAllianceDependent() {
		Pose2d robotPose = new Pose2d(0.19, 0.0, Rotation2d.kZero);
		ShotSolution forward = ShooterCalculator.calculate(robotPose, new Translation2d(3.0, 0.0));
		ShotSolution backward = ShooterCalculator.calculate(robotPose, new Translation2d(-3.0, 0.0));

		assertEquals(0.0, forward.targetHeading().getRadians(), EPSILON);
		assertEquals(Math.PI, Math.abs(backward.targetHeading().getRadians()), EPSILON);
	}

	@Test
	void rotatesShooterExitTransformWithRobot() {
		Pose2d robotPose = new Pose2d(2.0, 2.0, Rotation2d.kCCW_90deg);
		ShotSolution solution = ShooterCalculator.calculate(robotPose, new Translation2d(2.0, 3.0));

		assertEquals(Math.PI / 2.0, solution.targetHeading().getRadians(), EPSILON);
		assertEquals(1.19, solution.distanceMeters(), EPSILON);
	}
}
