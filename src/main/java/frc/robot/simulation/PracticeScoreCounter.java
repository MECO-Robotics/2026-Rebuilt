package frc.robot.simulation;

/** Successful practice fuel, attributed to the enabled mode at launch. */
final class PracticeScoreCounter {
	private int auto;
	private int teleop;
	private int generation;

	Runnable onLaunch(boolean autonomousEnabled, boolean teleopEnabled) {
		int launchGeneration = generation;
		return new Runnable() {
			private boolean counted;

			@Override
			public void run() {
				if (counted || launchGeneration != generation) {
					return;
				}
				counted = true;
				if (autonomousEnabled) {
					auto++;
				} else if (teleopEnabled) {
					teleop++;
				}
			}
		};
	}

	int autoCount() {
		return auto;
	}

	int teleopCount() {
		return teleop;
	}

	int totalCount() {
		return auto + teleop;
	}

	void reset() {
		auto = 0;
		teleop = 0;
		generation++;
	}
}
