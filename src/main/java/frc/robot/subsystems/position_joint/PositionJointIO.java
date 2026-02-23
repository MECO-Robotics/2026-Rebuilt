package frc.robot.subsystems.position_joint;

import frc.robot.subsystems.position_joint.PositionJointConstants.PositionJointGains;
import org.littletonrobotics.junction.AutoLog;

/** Hardware abstraction for a closed-loop position-controlled joint. */
public interface PositionJointIO {
  /**
   * Logged inputs shared by all joint implementations.
   *
   * <p>Units are rotations and rotations/sec unless otherwise documented by a specific
   * implementation.
   */
  @AutoLog
  public static class PositionJointIOInputs {
    /** Joint output position after gearing (mechanism position). */
    public double outputPosition = 0.0;
    /** Motor rotor position before gearing (motor shaft position). */
    public double rotorPosition = 0.0;
    /** Last requested joint position setpoint. */
    public double desiredPosition = 0.0;

    /** Measured mechanism velocity. */
    public double velocity = 0.0;
    /** Last requested mechanism velocity setpoint. */
    public double desiredVelocity = 0.0;

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
  }

  /** Refreshes all sensor and diagnostic inputs. */
  public default void updateInputs(PositionJointIOInputs inputs) {}

  /** Commands joint position/velocity setpoints for closed-loop control. */
  public default void setPosition(double position, double velocity) {}

  /** Commands an open-loop voltage output. */
  public default void setVoltage(double voltage) {}

  /** Applies controller/feedforward gains. */
  public default void setGains(PositionJointGains gains) {}

  /** Resets mechanism position to the implementation-defined zero. */
  public default void resetPosition() {}

  /** Returns a unique telemetry/logging name for this joint. */
  public String getName();
}
