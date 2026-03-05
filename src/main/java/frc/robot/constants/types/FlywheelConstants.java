package frc.robot.constants.types;

/** Shared constants and configuration records for flywheel/roller mechanisms. */
public class FlywheelConstants {
  /** Closed-loop and feedforward gains for one flywheel mechanism. */
  public record FlywheelGains(
      double kP,
      double kI,
      double kD,
      double kS,
      double kV,
      double kA,
      double kMaxAccel,
      double kTolerance) {}

  /**
   * Hardware mapping and mechanical constants for one flywheel instance.
   *
   * @param canIds CAN IDs of the motors in the flywheel mechanism, in order from closest to
   *     furthest from the shooter.
   * @param reversed Whether each motor is reversed. First boolean corresponds to clockwise
   *     positive, / rest of the booleans correspond to whether each subsequent motor is reversed
   *     relative to the first motor.
   * @param gearRatio The gear ratio between the motor and the flywheel (output speed / motor
   *     speed).
   * @param momentOfInertiaKgMetersSquared Flywheel mechanism moment of inertia in kg*m^2 for sim
   *     modeling.
   * @param currentLimit The current limit for the motors in amps.
   * @param canBus The CAN bus the motors are on, or an empty string for the rio bus.
   */
  public record FlywheelHardwareConfig(
      int[] canIds,
      boolean[] reversed,
      double gearRatio,
      double momentOfInertiaKgMetersSquared,
      int currentLimit,
      String canBus) {}

  /** Reference hardware config used as a template when adding new flywheels. */
  public static final FlywheelHardwareConfig EXAMPLE_CONFIG =
      new FlywheelHardwareConfig(new int[] {1}, new boolean[] {true}, 2.0, 0.025, 40, "");

  /** Reference gains used as a template when adding new flywheels. */
  public static final FlywheelGains EXAMPLE_GAINS =
      new FlywheelGains(0.2, 0.0, 0.0, 0.35, 0.065, 0.0, 1.0, 1.0);
}
