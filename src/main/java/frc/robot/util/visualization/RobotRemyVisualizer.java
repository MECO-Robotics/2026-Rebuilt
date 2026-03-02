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
  private final DoubleSupplier hoodRotations;
  private final DoubleSupplier flywheelRotations;
  private final DoubleSupplier intakeRackRotations;
  private final DoubleSupplier intakeKickerRotations;
  private final DoubleSupplier conveyorRotations;
  private final DoubleSupplier climberRotations;

  public RobotRemyVisualizer(
      Supplier<Pose2d> robotPoseSupplier,
      DoubleSupplier hoodRotations,
      DoubleSupplier flywheelRotations,
      DoubleSupplier intakeRackRotations,
      DoubleSupplier intakeKickerRotations,
      DoubleSupplier conveyorRotations,
      DoubleSupplier climberRotations) {
    this.robotPoseSupplier = robotPoseSupplier;
    this.hoodRotations = hoodRotations;
    this.flywheelRotations = flywheelRotations;
    this.intakeRackRotations = intakeRackRotations;
    this.intakeKickerRotations = intakeKickerRotations;
    this.conveyorRotations = conveyorRotations;
    this.climberRotations = climberRotations;
  }

  /** Pushes latest robot + component transforms for custom-asset visualization. */
  public void periodic() {
    Logger.recordOutput(ROBOT_POSE_LOG_KEY, new Pose3d(robotPoseSupplier.get()));
    Logger.recordOutput(
        COMPONENT_POSES_LOG_KEY,
        new Pose3d[] {
          // 0: flywheel & hood
          new Pose3d(
              0.0,
              0.0,
              0.0,
              new Rotation3d(
                  toRadians(flywheelRotations.getAsDouble()), toRadians(hoodRotations.getAsDouble()), 0.0)),
          // 1: intake rack
          new Pose3d(
              0.0,
              0.0,
              0.0,
              new Rotation3d(0.0, toRadians(intakeRackRotations.getAsDouble()), 0.0)),
          // 2: intake kicker bar
          new Pose3d(
              0.0,
              0.0,
              0.0,
              new Rotation3d(toRadians(intakeKickerRotations.getAsDouble()), 0.0, 0.0)),
          // 3: hopper
          new Pose3d(
              0.0,
              0.0,
              0.0,
              new Rotation3d(toRadians(conveyorRotations.getAsDouble()), 0.0, 0.0)),
          // 4: climber
          new Pose3d(0.0, 0.0, 0.0, new Rotation3d(toRadians(climberRotations.getAsDouble()), 0.0, 0.0))
        });
  }

  private static double toRadians(double rotations) {
    return rotations * 2.0 * Math.PI;
  }
}
