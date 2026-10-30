package frc.robot.simulation;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import org.dyn4j.dynamics.Body;
import org.dyn4j.geometry.Geometry;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Vector2;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class PracticeRebuiltArenaTest {
	static class Arena extends PracticeRebuiltArena {
		org.dyn4j.world.World<Body> world() {
			return physicsWorld;
		}
	}

	@BeforeAll
	static void initialize() {
		HAL.initialize(500, 0);
	}

	@Test
	void allFourTrenchWallsExistExactlyOnce() {
		Arena arena = new Arena();
		for (int x : new int[]{-1, 1}) {
			for (int y : new int[]{-1, 1}) {
				double px = 8.27 + x * PracticeRebuiltArena.TRENCH_X_OFFSET;
				double py = 4.035 + y * PracticeRebuiltArena.TRENCH_Y_OFFSET;
				assertEquals(1,
						arena.world().getBodies().stream().filter(
								body -> body.getTransform().getTranslation().distance(new Vector2(px, py)) < 1e-6)
								.count());
			}
		}
	}

	@Test
	void panelsAllowSlidingButStillBlockPenetration() {
		// Same 53 kg bumper body, force and time for free motion and wall contact.
		double free = slide(0.7, false, false);
		for (boolean trench : new boolean[]{false, true}) {
			double surfaceY = trench ? 4.035 - PracticeRebuiltArena.TRENCH_Y_OFFSET - 0.1524 : 0;
			double y = trench ? surfaceY - 0.42545 : 0.42545;
			double corrected = slide(y, trench, false);
			double original = slide(y, trench, true);
			System.out.printf("Contact trench=%s free=%.3f corrected=%.3f original=%.3f m/s%n", trench, free, corrected,
					original);
			assertTrue(corrected > free * 0.80, "Light wall contact must retain most along-wall speed");
			assertTrue(corrected > original, "Smooth panels must reduce the previous contact drag");
		}
	}

	private static double slide(double y, boolean trench, boolean originalFriction) {
		Arena arena = new Arena();
		if (originalFriction) {
			arena.world().getBodies().forEach(body -> body.getFixtures().forEach(f -> f.setFriction(0.6)));
		}
		Body robot = new Body();
		robot.addFixture(Geometry.createRectangle(0.8509, 0.8509), 53.0 / (0.8509 * 0.8509), 0.65, 0.0);
		robot.setMass(MassType.FIXED_ANGULAR_VELOCITY);
		robot.translate(trench ? 4.3 : 2.0, y);
		robot.setLinearVelocity(1.0, 0);
		arena.world().addBody(robot);
		for (int step = 0; step < 60; step++) {
			robot.applyForce(new Vector2(200, trench ? 100 : -100));
			arena.world().step(1, 0.005);
		}
		if (trench) {
			assertTrue(robot.getTransform().getTranslationY() <= y + 0.02, "Trench wall remains solid");
		} else if (y < 0.5) {
			assertTrue(robot.getTransform().getTranslationY() >= y - 0.02, "Field wall remains solid");
		}
		return robot.getLinearVelocity().x;
	}
}
