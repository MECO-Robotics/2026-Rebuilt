package frc.robot.util.visualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Logs robot and component poses for the Robot_Remy custom AdvantageScope asset. */
public class RobotRemyVisualizer {
  public static final String ROBOT_POSE_LOG_KEY = "Visualization/RobotRemy/RobotPose";
  public static final String COMPONENT_POSES_LOG_KEY = "Visualization/RobotRemy/ComponentPoses";

  private final Supplier<Pose2d> robotPoseSupplier;
  private final DoubleSupplier intakeRackRotations;
  private final DoubleSupplier hoodRotations;
  private final DoubleSupplier conveyorRotations;
  private final DoubleSupplier flywheelRotations;

  public RobotRemyVisualizer(
      Supplier<Pose2d> robotPoseSupplier,
      DoubleSupplier intakeRackRotations,
      DoubleSupplier hoodRotations,
      DoubleSupplier conveyorRotations,
      DoubleSupplier flywheelRotations) {
    this.robotPoseSupplier = robotPoseSupplier;
    this.intakeRackRotations = intakeRackRotations;
    this.hoodRotations = hoodRotations;
    this.conveyorRotations = conveyorRotations;
    this.flywheelRotations = flywheelRotations;
  }

  /** Pushes latest robot + component transforms for custom-asset visualization. */
  public void periodic() {
    Logger.recordOutput(ROBOT_POSE_LOG_KEY, new Pose3d(robotPoseSupplier.get()));
    // Logger.recordOutput(
    //     COMPONENT_POSES_LOG_KEY,
    //     new Pose3d[] {
    //       new Pose3d(
    //           0.0,
    //           0.0,
    //           0.0,
    //           new Rotation3d(0.0, toRadians(intakeRackRotations.getAsDouble()), 0.0)),
    //       new Pose3d(
    //           0.0, 0.0, 0.0, new Rotation3d(0.0, toRadians(hoodRotations.getAsDouble()), 0.0)),
    //       new Pose3d(
    //           0.0, 0.0, 0.0, new Rotation3d(toRadians(conveyorRotations.getAsDouble()), 0.0, 0.0)),
    //       new Pose3d(
    //           0.0, 0.0, 0.0, new Rotation3d(toRadians(flywheelRotations.getAsDouble()), 0.0, 0.0)),
    //       new Pose3d()
    //     });
  }

  private static double toRadians(double rotations) {
    return rotations * 2.0 * Math.PI;
  }
}
