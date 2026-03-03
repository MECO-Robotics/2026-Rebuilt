package frc.robot.subsystems.drive.drive_motor;

import frc.robot.Constants;
import frc.robot.subsystems.drive.drive_motor.DriveMotorConstants.DriveMotorGains;
import frc.robot.subsystems.drive.drive_motor.DriveMotorConstants.DriveMotorHardwareConfig;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;

/** Hardware abstraction for a swerve module drive motor. */
public interface DriveMotorIO {
  /** Logged inputs shared by all drive-motor implementations. */
  @AutoLog
  public static class DriveMotorIOInputs {
    /** Measured wheel velocity in mechanism rotations/sec. */
    public double velocityRotationsPerSecond = 0.0;
    /** Last requested wheel velocity in mechanism rotations/sec. */
    public double desiredVelocityRotationsPerSecond = 0.0;

    /** Measured wheel position in mechanism rotations. */
    public double positionRotations = 0.0;

    /** Connectivity state per motor controller. */
    public boolean[] motorsConnected = {false};

    // In phoenix native units
    /** Per-motor position telemetry. */
    public double[] motorPositions = {0.0};
    /** Per-motor velocity telemetry. */
    public double[] motorVelocities = {0.0};
    /** Per-motor acceleration telemetry (optional, impl-dependent). */
    public double[] motorAccelerations = {0.0};

    /** Per-motor applied voltage telemetry. */
    public double[] motorVoltages = {0.0};
    /** Per-motor current draw telemetry. */
    public double[] motorCurrents = {0.0};

    /** High-rate sample timestamps for odometry updates. */
    public double[] odometryTimestamps = new double[] {};
    /** High-rate drive positions in radians for odometry updates. */
    public double[] odometryDrivePositionsRad = new double[] {};
  }

  /** Refreshes all sensor and diagnostic inputs. */
  public default void updateInputs(DriveMotorIOInputs inputs) {}

  /** Commands closed-loop drive velocity. */
  public default void setVelocity(double velocity) {}

  /** Commands open-loop drive voltage. */
  public default void setVoltage(double voltage) {}

  /** Applies controller/feedforward gains. */
  public default void setGains(DriveMotorGains gains) {}

  /** Creates a TalonFX-backed drive motor IO supplier. */
  public static Supplier<DriveMotorIO> talonFXFactory(
      String name, DriveMotorHardwareConfig config) {
    return () -> new DriveMotorIOTalonFX(name, config);
  }

  /** Creates a SparkMax-backed drive motor IO supplier. */
  public static Supplier<DriveMotorIO> sparkMaxFactory(
      String name, DriveMotorHardwareConfig config) {
    return () -> new DriveMotorIOSparkMax(name, config);
  }

  /** Creates a default simulation drive motor IO supplier. */
  public static Supplier<DriveMotorIO> simFactory(String name, DriveMotorHardwareConfig config) {
    return () -> new DriveMotorIOSim(name, config);
  }

  /** Creates a replay drive motor IO supplier. */
  public static Supplier<DriveMotorIO> replayFactory(String name) {
    return () ->
        new DriveMotorIO() {
          @Override
          public String getName() {
            return name;
          }
        };
  }

  /**
   * Creates a mode-appropriate drive motor IO.
   *
   * <p>Returns the supplied real implementation on real hardware, simulated IO in sim, and replay
   * IO during log replay.
   */
  public static DriveMotorIO fromMode(
      String name, DriveMotorHardwareConfig config, Supplier<DriveMotorIO> realFactory) {
    return fromMode(name, config, realFactory, simFactory(name, config));
  }

  /**
   * Creates a mode-appropriate drive motor IO.
   *
   * <p>Allows callers to provide a custom sim factory (for example, MapleSim-backed module IO
   * endpoints that share state with azimuth IO).
   */
  public static DriveMotorIO fromMode(
      String name,
      DriveMotorHardwareConfig config,
      Supplier<DriveMotorIO> realFactory,
      Supplier<DriveMotorIO> simFactory) {
    return switch (Constants.currentMode) {
      case REAL -> realFactory.get();
      case SIM -> simFactory.get();
      default -> replayFactory(name).get();
    };
  }

  /** Creates mode-appropriate drive motor IO using TalonFX for real hardware. */
  public static DriveMotorIO fromTalonFX(String name, DriveMotorHardwareConfig config) {
    return fromMode(name, config, talonFXFactory(name, config));
  }

  /**
   * Creates mode-appropriate drive motor IO using TalonFX for real hardware and a custom sim
   * source.
   */
  public static DriveMotorIO fromTalonFX(
      String name, DriveMotorHardwareConfig config, Supplier<DriveMotorIO> simFactory) {
    return fromMode(name, config, talonFXFactory(name, config), simFactory);
  }

  /** Returns a unique telemetry/logging name for this drive motor. */
  public String getName();
}
