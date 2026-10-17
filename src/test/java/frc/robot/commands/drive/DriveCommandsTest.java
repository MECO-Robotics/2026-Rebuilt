package frc.robot.commands.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.junit.jupiter.api.Test;

class DriveCommandsTest {
	private static final double EPSILON = 1e-9;

	@Test
	void blueAbsoluteAimHeadingIsNotFlipped() {
		Rotation2d fieldTarget = Rotation2d.fromDegrees(37.0);

		Rotation2d requestTarget = DriveCommands.toOperatorPerspectiveTarget(fieldTarget, Alliance.Blue);

		assertEquals(0.0, requestTarget.minus(fieldTarget).getRadians(), EPSILON);
	}

	@Test
	void redOperatorPerspectiveDoesNotFlipAbsoluteAimHeading() {
		Rotation2d fieldTarget = Rotation2d.fromDegrees(37.0);

		Rotation2d requestTarget = DriveCommands.toOperatorPerspectiveTarget(fieldTarget, Alliance.Red);
		Rotation2d ctreAppliedFieldTarget = requestTarget.plus(Rotation2d.kPi);

		assertEquals(0.0, ctreAppliedFieldTarget.minus(fieldTarget).getRadians(), EPSILON);
	}
}
