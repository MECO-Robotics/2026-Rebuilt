package frc.robot.commands.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.subsystems.ShooterConstants;
import frc.robot.constants.vision.VisionConstants;
import org.junit.jupiter.api.Test;

class ShooterCalculatorTest {
	private static final double EPSILON = 1e-9;

	@Test
	void calculatesExactCloseCalibrationPointFromShooterExit() {
		double distance = Units.inchesToMeters(46.003);
		ShotSolution solution = ShooterCalculator.calculate(
				new Pose2d(distance - ShooterConstants.SHOOTER_EXIT_TRANSLATION.getX(), 0.0, Rotation2d.kZero),
				Translation2d.kZero);

		assertEquals(distance, solution.distanceMeters(), EPSILON);
		assertEquals(0.000, solution.hoodRotations(), EPSILON);
		assertEquals(30.0, solution.flywheelRotationsPerSecond(), EPSILON);
		assertEquals(0.0, solution.targetHeading().getRadians(), EPSILON);
		assertTrue(solution.calibrated());
	}

	@Test
	void calculatesExactTrenchEdgeCalibrationPoint() {
		double distance = Units.inchesToMeters(153.64);
		ShotSolution solution = ShooterCalculator.calculate(
				new Pose2d(distance - ShooterConstants.SHOOTER_EXIT_TRANSLATION.getX(), 0.0, Rotation2d.kZero),
				Translation2d.kZero);

		assertEquals(distance, solution.distanceMeters(), EPSILON);
		assertEquals(0.020, solution.hoodRotations(), EPSILON);
		assertEquals(41.0, solution.flywheelRotationsPerSecond(), EPSILON);
		assertTrue(solution.calibrated());
	}

	@Test
	void calculatesExactIntermediateCalibrationPoint() {
		double distance = Units.inchesToMeters(123.24);
		ShotSolution solution = ShooterCalculator.calculate(
				new Pose2d(distance - ShooterConstants.SHOOTER_EXIT_TRANSLATION.getX(), 0.0, Rotation2d.kZero),
				Translation2d.kZero);

		assertEquals(distance, solution.distanceMeters(), EPSILON);
		assertEquals(0.019, solution.hoodRotations(), EPSILON);
		assertEquals(37.9, solution.flywheelRotationsPerSecond(), EPSILON);
		assertTrue(solution.calibrated());
	}

	@Test
	void calculatesExactFarCalibrationPoint() {
		double distance = Units.inchesToMeters(191.86);
		ShotSolution solution = ShooterCalculator.calculate(
				new Pose2d(distance - ShooterConstants.SHOOTER_EXIT_TRANSLATION.getX(), 0.0, Rotation2d.kZero),
				Translation2d.kZero);

		assertEquals(distance, solution.distanceMeters(), EPSILON);
		assertEquals(0.020, solution.hoodRotations(), EPSILON);
		assertEquals(46.0, solution.flywheelRotationsPerSecond(), EPSILON);
		assertTrue(solution.calibrated());
	}

	@Test
	void calculatesExact202InchCalibrationPoint() {
		double distance = Units.inchesToMeters(202.78);
		ShotSolution solution = ShooterCalculator.calculate(
				new Pose2d(distance - ShooterConstants.SHOOTER_EXIT_TRANSLATION.getX(), 0.0, Rotation2d.kZero),
				Translation2d.kZero);

		assertEquals(distance, solution.distanceMeters(), EPSILON);
		assertEquals(0.020, solution.hoodRotations(), EPSILON);
		assertEquals(47.0, solution.flywheelRotationsPerSecond(), EPSILON);
		assertTrue(solution.calibrated());
	}

	@Test
	void calculatesExact144InchCalibrationPoint() {
		double distance = Units.inchesToMeters(144.60);
		ShotSolution solution = ShooterCalculator.calculate(
				new Pose2d(distance - ShooterConstants.SHOOTER_EXIT_TRANSLATION.getX(), 0.0, Rotation2d.kZero),
				Translation2d.kZero);

		assertEquals(distance, solution.distanceMeters(), EPSILON);
		assertEquals(0.020, solution.hoodRotations(), EPSILON);
		assertEquals(40.0, solution.flywheelRotationsPerSecond(), EPSILON);
		assertTrue(solution.calibrated());
	}

	@Test
	void calculatesExact210InchCalibrationPoint() {
		double distance = Units.inchesToMeters(210.47);
		ShotSolution solution = ShooterCalculator.calculate(
				new Pose2d(distance - ShooterConstants.SHOOTER_EXIT_TRANSLATION.getX(), 0.0, Rotation2d.kZero),
				Translation2d.kZero);

		assertEquals(distance, solution.distanceMeters(), EPSILON);
		assertEquals(0.020, solution.hoodRotations(), EPSILON);
		assertEquals(47.0, solution.flywheelRotationsPerSecond(), EPSILON);
		assertTrue(solution.calibrated());
	}

	@Test
	void interpolatesBetweenCalibrationPoints() {
		double distance = Units.inchesToMeters((46.003 + 123.24) / 2.0);
		ShotSolution solution = ShooterCalculator.calculate(
				new Pose2d(distance - ShooterConstants.SHOOTER_EXIT_TRANSLATION.getX(), 0.0, Rotation2d.kZero),
				Translation2d.kZero);

		assertEquals(0.0095, solution.hoodRotations(), EPSILON);
		assertEquals(33.95, solution.flywheelRotationsPerSecond(), EPSILON);
		assertTrue(solution.calibrated());
	}

	@Test
	void flagsDistancesOutsideCalibrationRange() {
		ShotSolution tooClose = ShooterCalculator
				.calculate(new Pose2d(Units.inchesToMeters(40.0) - ShooterConstants.SHOOTER_EXIT_TRANSLATION.getX(),
						0.0, Rotation2d.kZero), Translation2d.kZero);
		ShotSolution tooFar = ShooterCalculator
				.calculate(new Pose2d(Units.inchesToMeters(250.0) - ShooterConstants.SHOOTER_EXIT_TRANSLATION.getX(),
						0.0, Rotation2d.kZero), Translation2d.kZero);

		assertFalse(tooClose.calibrated());
		assertFalse(tooFar.calibrated());
	}

	@Test
	void autoAimCompensatesForDrivetrainHeadingBeingOppositeModelForward() {
		Pose2d robotPose = new Pose2d(0.0, 0.0, Rotation2d.kZero);
		ShotSolution aimForward = ShooterCalculator.calculate(robotPose, new Translation2d(-3.0, 0.0));
		ShotSolution modelForward = ShooterCalculator.calculate(robotPose, new Translation2d(3.0, 0.0));

		assertEquals(0.0, aimForward.targetHeading().getRadians(), EPSILON);
		assertEquals(Math.PI, Math.abs(modelForward.targetHeading().getRadians()), EPSILON);
	}

	@Test
	void rotatesShooterExitTransformWithRobot() {
		Pose2d robotPose = new Pose2d(2.0, 2.0, Rotation2d.kCCW_90deg);
		ShotSolution solution = ShooterCalculator.calculate(robotPose, new Translation2d(2.0, 0.0));

		assertEquals(Math.PI / 2.0, solution.targetHeading().getRadians(), EPSILON);
		assertEquals(1.81, solution.distanceMeters(), EPSILON);
	}

	@Test
	void limelightFacesTheSameRobotSideAsShooter() {
		assertEquals(ShooterConstants.SHOOTER_YAW_OFFSET.getRadians(),
				VisionConstants.robotToLimelight.getRotation().getZ(), EPSILON);
	}

	@Test
	void wornTapeSimulationScalePreservesPreviousShotVelocity() {
		double unscaledMetersPerSecondPerRps = ShooterConstants.MAIN_WHEEL_METERS_PER_SECOND_PER_RPS
				* (1.0 + ShooterConstants.COUNTER_TO_MAIN_SHOOTER_WHEEL_SPEED_RATIO) / 2.0;

		assertEquals(28.5 * unscaledMetersPerSecondPerRps,
				30.0 * ShooterConstants.PROJECTILE_METERS_PER_SECOND_PER_FLYWHEEL_RPS, EPSILON);
	}
}
