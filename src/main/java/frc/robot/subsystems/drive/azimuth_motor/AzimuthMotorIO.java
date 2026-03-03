package frc.robot.subsystems.drive.azimuth_motor;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.subsystems.drive.azimuth_motor.AzimuthMotorConstants.AzimuthMotorGains;
import frc.robot.subsystems.drive.azimuth_motor.AzimuthMotorConstants.AzimuthMotorHardwareConfig;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;

/** Hardware abstraction for a swerve module azimuth/steering motor. */
public interface AzimuthMotorIO {
  /** Logged inputs shared by all azimuth-motor implementations. */
  @AutoLog
  public static class AzimuthMotorIOInputs {
    /** Measured mechanism output position in rotations. */
    public double outputPositionRotations = 0.0;
    /** Measured rotor position in rotations. */
    public double rotorPositionRotations = 0.0;
    /** Last requested mechanism position setpoint in rotations. */
    public double desiredPositionRotations = 0.0;

    /** Measured mechanism velocity in rotations/sec. */
    public double velocityRotationsPerSecond = 0.0;
    /** Last requested mechanism velocity setpoint in rotations/sec. */
    public double desiredVelocityRotationsPerSecond = 0.0;

    /** Connectivity state per motor controller. */
    public boolean[] motorsConnected = {false};
    /** True when an external encoder is available and healthy. */
    public boolean encoderConnected = false;

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
    /** High-rate azimuth positions for odometry updates. */
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
  }

  /** Refreshes all sensor and diagnostic inputs. */
  public default void updateInputs(AzimuthMotorIOInputs inputs) {}

  /** Commands closed-loop position and velocity setpoints. */
  public default void setPosition(double position, double velocity) {}

  /** Commands open-loop voltage output. */
  public default void setVoltage(double voltage) {}

  /** Applies controller/feedforward gains. */
  public default void setGains(AzimuthMotorGains gains) {}

  /** Creates a TalonFX-backed azimuth motor IO supplier. */
  public static Supplier<AzimuthMotorIO> talonFXFactory(
      String name, AzimuthMotorHardwareConfig config) {
    return () -> new AzimuthMotorIOTalonFX(name, config);
  }

  /** Creates a SparkMax-backed azimuth motor IO supplier. */
  public static Supplier<AzimuthMotorIO> sparkMaxFactory(
      String name, AzimuthMotorHardwareConfig config) {
    return () -> new AzimuthMotorIOSparkMax(name, config);
  }

  /** Creates a default simulation azimuth motor IO supplier. */
  public static Supplier<AzimuthMotorIO> simFactory(
      String name, AzimuthMotorHardwareConfig config) {
    return () -> new AzimuthMotorIOSim(name, config);
  }

  /** Creates a replay azimuth motor IO supplier. */
  public static Supplier<AzimuthMotorIO> replayFactory(String name) {
    return () -> new AzimuthMotorIOReplay(name);
  }

  /**
   * Creates a mode-appropriate azimuth motor IO.
   *
   * <p>Returns the supplied real implementation on real hardware, simulated IO in sim, and replay
   * IO during log replay.
   */
  public static AzimuthMotorIO fromMode(
      String name, AzimuthMotorHardwareConfig config, Supplier<AzimuthMotorIO> realFactory) {
    return fromMode(name, config, realFactory, simFactory(name, config));
  }

  /**
   * Creates a mode-appropriate azimuth motor IO.
   *
   * <p>Allows callers to provide a custom sim factory (for example, MapleSim-backed module IO
   * endpoints that share state with drive IO).
   */
  public static AzimuthMotorIO fromMode(
      String name,
      AzimuthMotorHardwareConfig config,
      Supplier<AzimuthMotorIO> realFactory,
      Supplier<AzimuthMotorIO> simFactory) {
    return switch (Constants.currentMode) {
      case REAL -> realFactory.get();
      case SIM -> simFactory.get();
      default -> replayFactory(name).get();
    };
  }

  /** Creates mode-appropriate azimuth motor IO using TalonFX for real hardware. */
  public static AzimuthMotorIO fromTalonFX(String name, AzimuthMotorHardwareConfig config) {
    return fromMode(name, config, talonFXFactory(name, config));
  }

  /**
   * Creates mode-appropriate azimuth motor IO using TalonFX for real hardware and a custom sim
   * source.
   */
  public static AzimuthMotorIO fromTalonFX(
      String name, AzimuthMotorHardwareConfig config, Supplier<AzimuthMotorIO> simFactory) {
    return fromMode(name, config, talonFXFactory(name, config), simFactory);
  }

  /** Returns a unique telemetry/logging name for this azimuth motor. */
  public String getName();
}
