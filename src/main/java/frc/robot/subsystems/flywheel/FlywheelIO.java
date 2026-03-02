package frc.robot.subsystems.flywheel;

import frc.robot.subsystems.flywheel.FlywheelConstants.FlywheelGains;
import org.littletonrobotics.junction.AutoLog;

/** Hardware abstraction for a velocity-controlled flywheel-style mechanism. */
public interface FlywheelIO {
  /** Logged inputs shared by all flywheel hardware implementations. */
  @AutoLog
  public static class FlywheelIOInputs {
    /** Measured mechanism velocity. */
    public double velocity = 0.0;
    /** Last requested velocity setpoint. */
    public double desiredVelocity = 0.0;

    /** Measured mechanism position. */
    public double position = 0.0;

    /** Connectivity state per motor controller. */
    public boolean[] motorsConnected = {false};

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
  }

  /** Refreshes all sensor and diagnostic inputs. */
  public default void updateInputs(FlywheelIOInputs inputs) {}

  /** Commands a closed-loop velocity setpoint. */
  public default void setVelocity(double velocity) {}

  /** Commands open-loop voltage output. */
  public default void setVoltage(double voltage) {}

  /** Applies controller/feedforward gains. */
  public default void setGains(FlywheelGains gains) {}

  /** Returns a unique telemetry/logging name for this flywheel. */
  public String getName();
}
