// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Hardware abstraction for drivetrain yaw sensing. */
public interface GyroIO {
  /** Logged inputs shared by all gyro implementations. */
  @AutoLog
  public static class GyroIOInputs {
    /** True when gyro data is currently valid. */
    public boolean connected = false;
    /** Current yaw estimate. */
    public Rotation2d yawPosition = Rotation2d.kZero;
    /** Current yaw rate in rad/s. */
    public double yawVelocityRadPerSec = 0.0;
    /** High-rate sample timestamps for odometry updates. */
    public double[] odometryYawTimestamps = new double[] {};
    /** High-rate yaw samples for odometry updates. */
    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
  }

  /** Refreshes all sensor inputs. */
  public default void updateInputs(GyroIOInputs inputs) {}
}
