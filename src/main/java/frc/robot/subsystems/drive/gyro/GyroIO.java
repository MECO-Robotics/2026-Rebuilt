// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.Constants;
import java.util.function.Supplier;
import org.ironmaple.simulation.drivesims.GyroSimulation;
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

  /** Creates a replay gyro IO supplier. */
  public static Supplier<GyroIO> replayFactory() {
    return () -> new GyroIO() {};
  }

  /**
   * Creates a mode-appropriate gyro IO.
   *
   * <p>Returns the supplied real implementation on real hardware, default MapleSim IO in sim, and
   * no-op IO for replay.
   */
  public static GyroIO fromMode(Supplier<GyroIO> subsystemSupplier) {
    return switch (Constants.currentMode) {
      case REAL -> subsystemSupplier.get();
      case SIM -> new GyroIOSim();
      default -> replayFactory().get();
    };
  }

  /**
   * Creates a mode-appropriate gyro IO.
   *
   * <p>Allows callers to provide a custom sim factory for shared MapleSim state.
   */
  public static GyroIO fromMode(Supplier<GyroIO> subsystemSupplier, Supplier<GyroIO> simSupplier) {
    return switch (Constants.currentMode) {
      case REAL -> subsystemSupplier.get();
      case SIM -> simSupplier.get();
      default -> replayFactory().get();
    };
  }

  /** Creates a MapleSim-backed gyro IO supplier from a provided simulation model. */
  public static Supplier<GyroIO> simFactory(GyroSimulation gyroSimulation) {
    return () -> new GyroIOSim(gyroSimulation);
  }

  /** Creates mode-appropriate gyro IO using a Pigeon2 for real hardware. */
  public static GyroIO fromPigeon2(int canID, String canBusName) {
    return fromMode(() -> new GyroIOPigeon2(canID, canBusName));
  }

  /** Creates mode-appropriate gyro IO using a NavX for real hardware. */
  public static GyroIO fromNavX() {
    return fromMode(GyroIONavX::new);
  }
}
