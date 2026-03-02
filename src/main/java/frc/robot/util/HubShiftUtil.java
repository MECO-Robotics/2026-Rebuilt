package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class HubShiftUtil {
  public enum ShiftState {
    BLUE_ACTIVE,
    RED_ACTIVE
  }

  public record ShiftInfo(
      double matchTime,
      double remainingTime,
      boolean active,
      ShiftState currentShift,
      String matchTimeColor,
      String shiftTimeColor) {}

  private static final double PERIOD_LENGTH_SECS = 20.0;

  /** Returns the current state of the hub shift. */
  public static ShiftInfo getShiftedShiftInfo() {
    double matchTime = DriverStation.getMatchTime();
    // Default to 0 if not in match
    if (matchTime < 0) {
      matchTime = 0.0;
    }

    // Calculate remaining time in current period
    // Match time counts down, so modulo gives us the remainder of the current block
    double remainingTime = matchTime % PERIOD_LENGTH_SECS;

    // Determine active alliance based on period count
    // (matchTime / periodLength) roughly gives the number of periods remaining
    int periodCount = (int) (matchTime / PERIOD_LENGTH_SECS);

    // Alternate based on period count.
    Alliance activeAlliance = (periodCount % 2 == 0) ? Alliance.Blue : Alliance.Red;

    boolean isActive =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == activeAlliance;
    ShiftState state =
        (activeAlliance == Alliance.Blue) ? ShiftState.BLUE_ACTIVE : ShiftState.RED_ACTIVE;

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
    return Alliance.Blue;
  }
}
