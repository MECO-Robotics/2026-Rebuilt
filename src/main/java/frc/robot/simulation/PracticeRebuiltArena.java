package frc.robot.simulation;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;

/**
 * Practice layout with exactly 400 neutral-zone fuel, plus the normal depot
 * fuel.
 */
public class PracticeRebuiltArena extends Arena2026Rebuilt {
	/** Approximate smooth field panels, independently of tire/carpet traction. */
	static final double FIELD_CONTACT_FRICTION = 0.08;
	static final double TRENCH_X_OFFSET = Units.inchesToMeters(143.5);
	static final double TRENCH_Y_OFFSET = Units.inchesToMeters(102.5);

	public PracticeRebuiltArena() {
		super(false);
		boolean firstLowerLeftTrench = true;
		for (var body : physicsWorld.getBodies()) {
			if (!body.getMass().isInfinite()) {
				continue;
			}
			for (var fixture : body.getFixtures()) {
				fixture.setFriction(FIELD_CONTACT_FRICTION);
			}
			// MapleSim 0.4.0-beta duplicates the lower-left trench wall and
			// omits the upper-right one. Move only the duplicate, preserving shape.
			var center = body.getTransform().getTranslation();
			if (Math.abs(center.x - (8.27 - TRENCH_X_OFFSET)) < 1e-6
					&& Math.abs(center.y - (4.035 - TRENCH_Y_OFFSET)) < 1e-6) {
				if (firstLowerLeftTrench) {
					firstLowerLeftTrench = false;
				} else {
					body.translate(2.0 * TRENCH_X_OFFSET, 2.0 * TRENCH_Y_OFFSET);
				}
			}
		}
		setEfficiencyMode(false);
	}

	@Override
	public void placeGamePiecesOnField() {
		// The full MapleSim layout has 12 x 30 = 360 neutral-zone fuel.
		setEfficiencyMode(false);
		super.placeGamePiecesOnField();
		// Add 20 on each side of that grid without overlapping its existing fuel.
		for (int x : new int[]{-1, 12}) {
			for (int y = 5; y < 25; y++) {
				addGamePiece(new RebuiltFuelOnField(centerPieceBottomRightCorner
						.plus(new Translation2d(Units.inchesToMeters(5.991 * x), Units.inchesToMeters(5.95 * y)))));
			}
		}
	}
}
