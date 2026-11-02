package frc.robot.commands.shooter;

import frc.robot.constants.subsystems.ShooterConstants.INDEXER_PRESET;

/**
 * Existing motor requests, shared by real commands and simulated transport
 * checks.
 */
public enum IndexerFeedDirection {
	MANUAL(1.0), COORDINATED(-1.0);

	private final double sign;

	IndexerFeedDirection(double sign) {
		this.sign = sign;
	}

	public double bottomVolts() {
		return sign * INDEXER_PRESET.FEED_BOTTOM.get();
	}

	public double topVolts() {
		return sign * INDEXER_PRESET.FEED_TOP.get();
	}
}
