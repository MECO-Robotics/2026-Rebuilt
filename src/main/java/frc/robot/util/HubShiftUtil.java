package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class HubShiftUtil {
	public enum ShiftState {
		BLUE_ACTIVE, RED_ACTIVE, TRANSITION
	}

	public record ShiftInfo(double matchTime, double remainingTime, boolean active, ShiftState currentShift,
			String matchTimeColor, String shiftTimeColor) {
	}

	private static final double PERIOD_LENGTH_SECS = 25.0;
	private static final double TRANSITION_LENGTH_SECS = 10.0;
	private static final double TELEOP_DURATION = 135.0;

	/** Returns the current state of the hub shift. */
	public static ShiftInfo getShiftedShiftInfo() {
		double matchTime = DriverStation.getMatchTime();
		// Default to 0 if not in match
		if (matchTime < 0) {
			matchTime = 0.0;
		}

		double remainingTime;
		ShiftState state;
		boolean isActive = false;

		if (DriverStation.isTeleop()) {
			if (matchTime > TELEOP_DURATION - TRANSITION_LENGTH_SECS) {
				state = ShiftState.TRANSITION;
				remainingTime = matchTime - (TELEOP_DURATION - TRANSITION_LENGTH_SECS);
				isActive = false;
			} else {
				double shiftTime = (TELEOP_DURATION - TRANSITION_LENGTH_SECS) - matchTime;
				remainingTime = PERIOD_LENGTH_SECS - (shiftTime % PERIOD_LENGTH_SECS);
				int periodCount = (int) (shiftTime / PERIOD_LENGTH_SECS);
				Alliance activeAlliance = (periodCount % 2 == 0)
						? getFirstActiveAlliance()
						: (getFirstActiveAlliance() == Alliance.Blue ? Alliance.Red : Alliance.Blue);
				state = (activeAlliance == Alliance.Blue) ? ShiftState.BLUE_ACTIVE : ShiftState.RED_ACTIVE;
				isActive = DriverStation.getAlliance().isPresent()
						&& DriverStation.getAlliance().get() == activeAlliance;
			}
		} else {
			remainingTime = matchTime % PERIOD_LENGTH_SECS;
			int periodCount = (int) (matchTime / PERIOD_LENGTH_SECS);
			Alliance activeAlliance = (periodCount % 2 == 0) ? Alliance.Blue : Alliance.Red;
			state = (activeAlliance == Alliance.Blue) ? ShiftState.BLUE_ACTIVE : ShiftState.RED_ACTIVE;
			isActive = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == activeAlliance;
		}

		String matchTimeColor;
		if (matchTime > 50) {
			matchTimeColor = "#0000FF";
		} else if (matchTime > 20) {
			matchTimeColor = "#FFFF00";
		} else {
			matchTimeColor = "#FF0000";
		}

		String shiftTimeColor = remainingTime > 10 ? "#00FF00" : "#FF0000";

		return new ShiftInfo(matchTime, remainingTime, isActive, state, matchTimeColor, shiftTimeColor);
	}

	public static Alliance getFirstActiveAlliance() {
		String fmsMessage = DriverStation.getGameSpecificMessage();
		if (fmsMessage != null && !fmsMessage.isEmpty()) {
			if (fmsMessage.toUpperCase().startsWith("R")) {
				return Alliance.Red;
			} else if (fmsMessage.toUpperCase().startsWith("B")) {
				return Alliance.Blue;
			}
		}
		return Alliance.Blue;
	}
}
