package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

public class Telemetry {
	private final double MaxSpeed;

	/**
	 * Construct a telemetry object, with the specified max speed of the robot
	 *
	 * @param maxSpeed
	 *            Maximum speed in meters per second
	 */
	public Telemetry(double maxSpeed) {
		MaxSpeed = maxSpeed;

		/* Set up the module state Mechanism2d telemetry */
		for (int i = 0; i < 4; ++i) {
			SmartDashboard.putData("Module " + i, m_moduleMechanisms[i]);
		}
	}

	/* What to publish over networktables for telemetry */
	private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

	/* Robot swerve drive state */
	private final NetworkTable driveStateTable = inst.getTable("DriveState");
	private final StructPublisher<Pose2d> drivePose = driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();
	private final StructPublisher<Pose2d> drivePhysicsPose = driveStateTable
			.getStructTopic("PhysicsPose", Pose2d.struct).publish();
	private final StructPublisher<ChassisSpeeds> driveSpeeds = driveStateTable
			.getStructTopic("Speeds", ChassisSpeeds.struct).publish();
	private final StructPublisher<ChassisSpeeds> driveAutoCommandedSpeeds = driveStateTable
			.getStructTopic("AutoCommandedSpeeds", ChassisSpeeds.struct).publish();
	private final StructArrayPublisher<SwerveModuleState> driveModuleStates = driveStateTable
			.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
	private final StructArrayPublisher<SwerveModuleState> driveModuleTargets = driveStateTable
			.getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish();
	private final StructArrayPublisher<SwerveModulePosition> driveModulePositions = driveStateTable
			.getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();
	private final DoublePublisher driveTimestamp = driveStateTable.getDoubleTopic("Timestamp").publish();
	private final DoublePublisher driveOdometryFrequency = driveStateTable.getDoubleTopic("OdometryFrequency")
			.publish();

	/* Robot pose for field positioning */
	private final NetworkTable table = inst.getTable("Pose");
	private final DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("robotPose").publish();
	private final DoubleArrayPublisher physicsFieldPub = table.getDoubleArrayTopic("robotPoseActual").publish();
	private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

	/* Mechanisms to represent the swerve module states */
	private final Mechanism2d[] m_moduleMechanisms = new Mechanism2d[]{new Mechanism2d(1, 1), new Mechanism2d(1, 1),
			new Mechanism2d(1, 1), new Mechanism2d(1, 1),};
	/* A direction and length changing ligament for speed representation */
	private final MechanismLigament2d[] m_moduleSpeeds = new MechanismLigament2d[]{
			m_moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
			m_moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
			m_moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
			m_moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),};
	/* A direction changing and length constant ligament for module direction */
	private final MechanismLigament2d[] m_moduleDirections = new MechanismLigament2d[]{
			m_moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
					.append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
			m_moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
					.append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
			m_moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
					.append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
			m_moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
					.append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),};

	private final double[] m_poseArray = new double[3];
	private final double[] m_physicsPoseArray = new double[3];

	/**
	 * Accept the swerve drive state and telemeterize it to dashboards and
	 * AdvantageKit.
	 */
	public void telemeterize(SwerveDriveState state, Pose2d physicsPose, ChassisSpeeds physicsSpeeds,
			Rotation2d rawGyroHeading) {
		/* Telemeterize the swerve drive state */
		drivePose.set(state.Pose);
		drivePhysicsPose.set(physicsPose);
		driveSpeeds.set(state.Speeds);
		driveModuleStates.set(state.ModuleStates);
		driveModuleTargets.set(state.ModuleTargets);
		driveModulePositions.set(state.ModulePositions);
		driveTimestamp.set(state.Timestamp);
		driveOdometryFrequency.set(1.0 / state.OdometryPeriod);

		/* Mirror drivetrain state into AdvantageKit for WPILOG capture/replay. */
		Logger.recordOutput("DriveState/Pose", state.Pose);
		Logger.recordOutput("DriveState/PhysicsPose", physicsPose);
		Transform2d poseError = physicsPose.minus(state.Pose);
		Logger.recordOutput("DriveState/PoseErrorX", poseError.getX());
		Logger.recordOutput("DriveState/PoseErrorY", poseError.getY());
		Logger.recordOutput("DriveState/PoseErrorThetaDeg", poseError.getRotation().getDegrees());
		Logger.recordOutput("DriveState/Speeds", state.Speeds);
		Logger.recordOutput("DriveState/PhysicsSpeeds", physicsSpeeds);
		Logger.recordOutput("DriveState/ModuleStates", state.ModuleStates);
		Logger.recordOutput("DriveState/ModuleTargets", state.ModuleTargets);
		Logger.recordOutput("DriveState/ModulePositions", state.ModulePositions);
		Logger.recordOutput("DriveState/Timestamp", state.Timestamp);
		Logger.recordOutput("DriveState/OdometryPeriod", state.OdometryPeriod);
		Logger.recordOutput("DriveState/OdometryFrequency", 1.0 / state.OdometryPeriod);
		Logger.recordOutput("DriveState/RawGyroHeading", rawGyroHeading);
		Logger.recordOutput("DriveState/EstimatedHeadingErrorDeg",
				rawGyroHeading.minus(state.Pose.getRotation()).getDegrees());

		/* Telemeterize the pose to a Field2d */
		fieldTypePub.set("Field2d");

		m_poseArray[0] = state.Pose.getX();
		m_poseArray[1] = state.Pose.getY();
		m_poseArray[2] = state.Pose.getRotation().getDegrees();
		fieldPub.set(m_poseArray);
		m_physicsPoseArray[0] = physicsPose.getX();
		m_physicsPoseArray[1] = physicsPose.getY();
		m_physicsPoseArray[2] = physicsPose.getRotation().getDegrees();
		physicsFieldPub.set(m_physicsPoseArray);

		/* Telemeterize each module state to a Mechanism2d */
		for (int i = 0; i < 4; ++i) {
			m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
			m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
			m_moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * MaxSpeed));
		}
	}

	public void logAutoCommandedSpeeds(ChassisSpeeds speeds) {
		driveAutoCommandedSpeeds.set(speeds);
		Logger.recordOutput("DriveState/AutoCommandedSpeeds", speeds);
	}
}
