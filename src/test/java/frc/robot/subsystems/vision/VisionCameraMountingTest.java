package frc.robot.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.constants.vision.VisionConstants;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class VisionCameraMountingTest {
	@BeforeAll
	static void initializeHal() {
		HAL.initialize(500, 0);
	}

	@Test
	void limelightLooksForwardAndThirtyDegreesAboveTheHorizon() {
		Translation3d viewDirection = new Translation3d(1.0, 0.0, 0.0)
				.rotateBy(VisionConstants.robotToLimelight.getRotation());
		assertEquals(Math.cos(Math.toRadians(30)), viewDirection.getX(), 1e-9);
		assertEquals(0.0, viewDirection.getY(), 1e-9);
		assertEquals(Math.sin(Math.toRadians(30)), viewDirection.getZ(), 1e-9);
	}

	@Test
	void questLooksRearwardWithItsTopPointingDown() {
		var mountingRotation = VisionConstants.robotToQuest.getRotation();
		Translation3d viewDirection = new Translation3d(1.0, 0.0, 0.0).rotateBy(mountingRotation);
		Translation3d headsetUp = new Translation3d(0.0, 0.0, 1.0).rotateBy(mountingRotation);
		assertEquals(-1.0, viewDirection.getX(), 1e-9);
		assertEquals(0.0, viewDirection.getY(), 1e-9);
		assertEquals(0.0, viewDirection.getZ(), 1e-9);
		assertEquals(0.0, headsetUp.getX(), 1e-9);
		assertEquals(0.0, headsetUp.getY(), 1e-9);
		assertEquals(-1.0, headsetUp.getZ(), 1e-9);
	}

	@Test
	void publishesTheForwardUpwardMountInLimelightRobotSpace() {
		String cameraName = "limelight-mounting-test";
		try (var cameraPose = NetworkTableInstance.getDefault().getTable(cameraName)
				.getDoubleArrayTopic("camerapose_robotspace_set").subscribe(new double[0])) {
			var io = new VisionIOLimelight(cameraName, VisionConstants.robotToLimelight);
			io.updateInputs(new VisionIO.VisionIOInputs());
			assertArrayEquals(new double[]{0.0, 0.0, Units.inchesToMeters(21.25), 0.0, 30.0, 0.0}, cameraPose.get(),
					1e-9);
		}
	}
}
