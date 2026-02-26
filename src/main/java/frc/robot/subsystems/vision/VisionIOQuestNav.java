package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import org.littletonrobotics.junction.Logger;

public class VisionIOQuestNav implements VisionIO {
  public record QuestNavData(
      Pose3d pose,
      double batteryPercent,
      double timestamp,
      float[] translation,
      float[] rotation) {}

  private QuestNav questNav;

  private final Transform3d robotToCamera;

  private final VisionIO absoluteVisionIO;
  private final VisionIOInputsAutoLogged absoluteInputs = new VisionIOInputsAutoLogged();

  private Translation3d[] questNavRawToFieldCoordinateSystemQueue = new Translation3d[15];
  private Translation3d questNavRawToFieldCoordinateSystem = new Translation3d();

  protected Rotation3d gyroResetAngle = new Rotation3d();
  protected Pose3d lastPose3d = new Pose3d();

  int count = 0;
  int idx = 0;

  public VisionIOQuestNav(Transform3d robotToCamera, VisionIO absoluteVisionIO) {
    // Initialize the camera to robot transform
    questNav = new QuestNav();
    this.robotToCamera = robotToCamera;
    this.absoluteVisionIO = absoluteVisionIO;

    // gyroResetAngle = Constants.isAllianceRed() ? Rotation3d.kZero : new Rotation3d(0, 0,
    // Math.PI);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    questNav.commandPeriodic();

    QuestNavData[] questNavData = getQuestNavData();

    absoluteVisionIO.updateInputs(absoluteInputs);
    Logger.processInputs("QuestNav/absolute", absoluteInputs);

    inputs.connected = connected();
    inputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d(), 0);
    inputs.poseObservations = new PoseObservation[questNavData.length];

    if (absoluteInputs.poseObservations.length > 0 && questNavData.length > 0) {
      questNavRawToFieldCoordinateSystemQueue[idx] =
          absoluteInputs
              .poseObservations[0]
              .pose()
              .getTranslation()
              .minus(questNavData[0].pose.getTranslation().rotateBy(gyroResetAngle));
      count += 1;
      idx += 1;
      if (idx == questNavRawToFieldCoordinateSystemQueue.length) {
        idx = 0;
      }
      questNavRawToFieldCoordinateSystem = new Translation3d();
      for (int i = 0; i < Math.min(count, questNavRawToFieldCoordinateSystemQueue.length); i++) {
        questNavRawToFieldCoordinateSystem =
            questNavRawToFieldCoordinateSystem.plus(questNavRawToFieldCoordinateSystemQueue[i]);
      }
      questNavRawToFieldCoordinateSystem =
          questNavRawToFieldCoordinateSystem.div(
              Math.min(count, questNavRawToFieldCoordinateSystemQueue.length));
      Logger.recordOutput("QuestNav/RawToField", questNavRawToFieldCoordinateSystem);
    }

    for (int i = 0; i < questNavData.length; i++) {
      inputs.poseObservations[i] =
          new PoseObservation(
              questNavData[i].timestamp(),
              new Pose3d(
                  questNavData[i]
                      .pose()
                      .getTranslation()
                      .rotateBy(gyroResetAngle)
                      .plus(questNavRawToFieldCoordinateSystem),
                  questNavData[i].pose().getRotation().plus(gyroResetAngle)),
              0.0,
              -1,
              0.0,
              PoseObservationType.QUESTNAV);

      lastPose3d = inputs.poseObservations[i].pose();
    }
    inputs.tagIds = new int[0];

    Logger.recordOutput("QuestNav/battery", getBatteryPercent());
  }

  private boolean connected() {
    return questNav.isConnected();
  }

  private double getBatteryPercent() {
    return questNav.getBatteryPercent().orElse(0);
  }

  private QuestNavData[] getQuestNavData() {

    PoseFrame[] newFrame = questNav.getAllUnreadPoseFrames();
    double battery = getBatteryPercent();
    int length = newFrame.length;
    QuestNavData[] data = new QuestNavData[length];

    for (int i = 0; i < length; i++) {
      data[i] =
          new QuestNavData(
              newFrame[i]
                  .questPose3d()
                  .rotateBy(robotToCamera.getRotation())
                  .plus(robotToCamera.inverse()),
              battery,
              newFrame[i].dataTimestamp(),
              getQuestTranslation(newFrame[i].questPose3d()),
              getQuestRotation(newFrame[i].questPose3d().getRotation()));
    }

    return data;
  }

  private float[] getQuestTranslation(Pose3d pose) {
    float xPosition = (float) pose.getX();
    float yPosition = (float) pose.getY();
    float zPosition = (float) pose.getZ();

    return new float[] {-yPosition, zPosition, xPosition};
  }

  private float[] getQuestRotation(Rotation3d angle) {
    float yaw = (float) -Units.radiansToDegrees(angle.getZ());
    float pitch = (float) Units.radiansToDegrees(angle.getY());
    float roll = (float) -Units.radiansToDegrees(angle.getX());

    return new float[] {pitch, yaw, roll};
  }

  public void resetPose(Pose3d pose) {

    questNavRawToFieldCoordinateSystem =
        pose.getTranslation()
            .minus(lastPose3d.getTranslation().minus(questNavRawToFieldCoordinateSystem));

    // TODO: clarify if we need to have the robot to camera
    questNav.setPose(pose.transformBy(robotToCamera));

    count = 0;
    idx = 0;
  }

  public void resetHeading(Rotation2d heading) {

    gyroResetAngle =
        (lastPose3d.getRotation().minus(gyroResetAngle).minus(new Rotation3d(heading)))
            .unaryMinus();

    questNav.setPose(new Pose3d(lastPose3d.getTranslation(), new Rotation3d(heading)));
  }

  public void resetHeading() {
    resetHeading(
        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
            ? Rotation2d.kPi
            : Rotation2d.kZero);
  }

  public void resetBlue() {
    resetHeading(Rotation2d.kZero);
  }
}
