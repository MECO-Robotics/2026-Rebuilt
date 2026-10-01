package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * A field-relative target and the mechanism profile used to shoot toward it.
 */
public record ShotTarget(Mode mode, Translation2d position) {
	/** Supported coordinated-shot target modes. */
	public enum Mode {
		HUB, FERRY
	}
}
