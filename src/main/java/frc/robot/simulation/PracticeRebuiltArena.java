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
	public PracticeRebuiltArena() {
		super(false);
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
