package frc.robot.simulation;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.math.geometry.*;
import frc.robot.constants.Constants.Mode;
import frc.robot.constants.subsystems.ShooterConstants;
import frc.robot.util.visualization.RobotRemyVisualizer;
import org.junit.jupiter.api.Test;

class RemyGeometryTest {
	@Test
	void rackExtendsOutOfCadRollerEndAndShootsSameWayAtEveryHeading() {
		// model_2.glb Roller Hub / Tube: source Z=+0.315283384266377.
		// config.json Rx(+90), then Rz(+90): (x,y,z) -> (z,x,y).
		var cadRoller = new Translation3d(-.00993905327510855, .203262641882958, .315283384266377)
				.rotateBy(new Rotation3d(Math.PI / 2, 0, 0)).rotateBy(new Rotation3d(0, 0, Math.PI / 2));
		assertTrue(cadRoller.getX() > 0);
		assertEquals(org.ironmaple.simulation.IntakeSimulation.IntakeSide.FRONT, IntakeSim.INTAKE_SIDE);
		for (double extension : new double[]{0, .13, .30}) {
			var poses = RobotRemyVisualizer.componentPoses(extension, 0, 0, 0);
			assertEquals(6, poses.length);
			assertEquals(extension * Math.cos(Math.toRadians(7.5)), poses[2].getX(), 1e-9);
			assertEquals(-extension * Math.sin(Math.toRadians(7.5)), poses[2].getZ(), 1e-9);
			assertTrue(cadRoller.plus(poses[2].getTranslation()).getZ() > 0);
			assertEquals(poses[2].getX(), poses[3].getX());
			assertEquals(poses[2].getX(), poses[4].getX());
			for (double degrees : new double[]{0, 90, 180, 270}) {
				var yaw = Rotation2d.fromDegrees(degrees);
				var delta = poses[2].getTranslation().toTranslation2d().rotateBy(yaw);
				var shot = LaunchedFuelSim.getShooterFacing(yaw);
				assertEquals(extension * Math.cos(Math.toRadians(7.5)),
						delta.getX() * shot.getCos() + delta.getY() * shot.getSin(), 1e-9);
				var exit = frc.robot.constants.simulation.MapleSimConstants.SHOOTER_TRANSLATION_ON_ROBOT.rotateBy(yaw);
				// Projectile frame rotation must not rotate the exit twice.
				int middle = 0;
				var offset = LaunchedFuelSim.getMapleShooterTranslationForBurstIndex(middle).rotateBy(shot);
				assertEquals(exit.getX() * shot.getCos() + exit.getY() * shot.getSin(),
						offset.getX() * shot.getCos() + offset.getY() * shot.getSin(), 1e-9);
			}
		}
	}

	@Test
	void simulationCorrectionDoesNotReverseRealOrReplayAim() {
		assertEquals(Math.PI, ShooterConstants.aimingYawForMode(Mode.REAL).getRadians(), 1e-9);
		assertEquals(Math.PI, ShooterConstants.aimingYawForMode(Mode.REPLAY).getRadians(), 1e-9);
		assertEquals(0, ShooterConstants.aimingYawForMode(Mode.SIM).getRadians(), 1e-9);
	}
}
