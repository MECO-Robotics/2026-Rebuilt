package frc.robot.simulation;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.Arrays;
import org.junit.jupiter.api.Test;

class HopperTest {
	private static final double EPSILON = 1e-9;

	@Test
	void storedFuelStaysOnAndExtendsTowardIntakeAtRobotNegativeX() {
		Hopper stowed = new Hopper(() -> 100, () -> 0.0, Pose2d::new);
		Hopper deployed = new Hopper(() -> 100, () -> 0.30, Pose2d::new);
		Pose3d[] stowedPoses = stowed.getGamePiecePoses();
		Pose3d[] deployedPoses = deployed.getGamePiecePoses();

		assertTrue(stowedPoses.length > 0);
		assertTrue(Arrays.stream(stowedPoses).allMatch(pose -> pose.getX() <= EPSILON));
		assertTrue(minX(deployedPoses) < minX(stowedPoses));
	}

	private static double minX(Pose3d[] poses) {
		return Arrays.stream(poses).mapToDouble(Pose3d::getX).min().orElseThrow();
	}
}
