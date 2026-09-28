package frc.robot.commands.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.Test;

class ShotReadinessTest {
	private static final ShotSolution VALID_SOLUTION = new ShotSolution(3.0, Rotation2d.kZero, 0.025, 35.0, true);

	@Test
	void requiresPoseAndCalibratedRange() {
		assertEquals(ShotReadiness.POSE_NOT_READY, evaluate(false, VALID_SOLUTION, new ChassisSpeeds(), 0.025, 35.0));
		ShotSolution invalidRange = new ShotSolution(8.0, Rotation2d.kZero, 0.049, 51.0, false);
		assertEquals(ShotReadiness.OUT_OF_RANGE, evaluate(true, invalidRange, new ChassisSpeeds(), 0.049, 51.0));
	}

	@Test
	void rejectsMotionAndMechanismErrorsInPriorityOrder() {
		assertEquals(ShotReadiness.MOVING_TOO_FAST,
				evaluate(true, VALID_SOLUTION, new ChassisSpeeds(0.3, 0.0, 0.0), 0.025, 35.0));
		assertEquals(ShotReadiness.AIMING, ShooterCommands.evaluateReadiness(true, VALID_SOLUTION, new ChassisSpeeds(),
				Rotation2d.fromDegrees(3.0), 0.025, 35.0));
		assertEquals(ShotReadiness.HOOD_MOVING, evaluate(true, VALID_SOLUTION, new ChassisSpeeds(), 0.03, 35.0));
		assertEquals(ShotReadiness.FLYWHEEL_SPINNING_UP,
				evaluate(true, VALID_SOLUTION, new ChassisSpeeds(), 0.025, 33.0));
	}

	@Test
	void reportsReadyOnlyInsideEveryThreshold() {
		assertEquals(ShotReadiness.READY,
				evaluate(true, VALID_SOLUTION, new ChassisSpeeds(0.1, 0.1, 0.05), 0.025, 35.0));
	}

	private static ShotReadiness evaluate(boolean poseReady, ShotSolution solution, ChassisSpeeds speeds,
			double hoodPosition, double flywheelVelocity) {
		return ShooterCommands.evaluateReadiness(poseReady, solution, speeds, Rotation2d.kZero, hoodPosition,
				flywheelVelocity);
	}
}
