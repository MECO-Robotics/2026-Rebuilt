// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Hardware abstraction for vision cameras and pose-estimation pipelines. */
public interface VisionIO {
  /** Logged inputs shared by all vision implementations. */
  @AutoLog
  public static class VisionIOInputs {
    /** True when the camera/pipeline is connected and publishing data. */
    public boolean connected = false;
    /** Latest simple target observation (tx/ty) for servo use cases. */
    public TargetObservation latestTargetObservation =
        new TargetObservation(Rotation2d.kZero, Rotation2d.kZero, 0);
    /** Pose observations produced this cycle. */
    public PoseObservation[] poseObservations = new PoseObservation[0];
    /** Tag IDs observed this cycle. */
    public int[] tagIds = new int[0];
  }

  /** Represents the angle to a simple target, not used for pose estimation. */
  public static record TargetObservation(Rotation2d tx, Rotation2d ty, int tagID) {}

  /** Represents a robot pose sample used for pose estimation. */
  public static record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      PoseObservationType type) {}

  public static enum PoseObservationType {
    MEGATAG_1,
    MEGATAG_2,
    PHOTONVISION,
    PHOTONVISIONTRIG,
    QUESTNAV
  }

  /** Refreshes all camera and estimation inputs. */
  public default void updateInputs(VisionIOInputs inputs) {}
}
