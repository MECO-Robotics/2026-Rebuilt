package frc.robot.subsystems.flywheel;

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

  /** Hardware mapping and mechanical constants for one flywheel instance. */
  public record FlywheelHardwareConfig(
      int[] canIds, boolean[] reversed, double gearRatio, int currentLimit, String canBus) {}

  /** Reference hardware config used as a template when adding new flywheels. */
  public static final FlywheelHardwareConfig EXAMPLE_CONFIG =
      new FlywheelHardwareConfig(new int[] {1}, new boolean[] {true}, 2.0, 40, "");

  /** Reference gains used as a template when adding new flywheels. */
  public static final FlywheelGains EXAMPLE_GAINS =
      new FlywheelGains(0.2, 0.0, 0.0, 0.0, 0.065, 0.0, 1.0, 1.0);

  // -----------
  // Conveyor Constants
  // -----------
  public static final FlywheelHardwareConfig CONVEYOR_CONFIG =
      new FlywheelHardwareConfig(new int[] {23}, new boolean[] {false}, 1, 40, "");
  public static final FlywheelGains CONVEYOR_GAINS =
      new FlywheelGains(0.2, 0.0, 0.0, 0.0, 0.065, 0.0, 1.0, 1.0);

  // -----------
  // Intake Constants
  // -----------
  public static final FlywheelHardwareConfig INTAKE_ROLLER_CONFIG =
      new FlywheelHardwareConfig(new int[] {22}, new boolean[] {false}, 1, 40, "");
  public static final FlywheelGains INTAKE_ROLLER_GAINS =
      new FlywheelGains(0.2, 0.0, 0.0, 0.0, 0.065, 0.0, 1.0, 1.0);

  // ------------
  // Indexer Constants
  // ------------
  public static final FlywheelHardwareConfig TOP_INDEXER_ROLLER_CONFIG =
      new FlywheelHardwareConfig(new int[] {32}, new boolean[] {false}, 1, 40, "");
  public static final FlywheelHardwareConfig BOTTOM_INDEXER_ROLLER_CONFIG =
      new FlywheelHardwareConfig(new int[] {31}, new boolean[] {false}, 1, 40, "");
  public static final FlywheelGains INDEXER_ROLLER_GAINS =
      new FlywheelGains(0, 0, 0, 0, 0, 0, 0, 0);

  // ------------
  // Shooter Constants
  // ------------
  public static final FlywheelHardwareConfig FLYWHEEL_ROLLER_CONFIG =
      new FlywheelHardwareConfig(
          new int[] {35, 34}, new boolean[] {false, true}, 1, 40, "MECO CANIvore");
  public static final FlywheelGains FLYWHEEL_ROLLER_GAINS =
      new FlywheelGains(0.2, 0.0, 0.0, 0.0, 0.065, 0.0, 1.0, 1.0);
}
