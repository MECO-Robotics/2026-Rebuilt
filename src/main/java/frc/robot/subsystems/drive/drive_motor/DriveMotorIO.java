package frc.robot.subsystems.drive.drive_motor;

import frc.robot.subsystems.drive.drive_motor.DriveMotorConstants.DriveMotorGains;
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

  /** Returns a unique telemetry/logging name for this drive motor. */
  public String getName();
}
