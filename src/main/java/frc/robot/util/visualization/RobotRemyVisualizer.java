package frc.robot.util.visualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.simulation.MapleSimConstants;
import frc.robot.util.mechanical_advantage.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Logs robot and component poses for the Robot_Remy custom AdvantageScope
 * asset.
 */
public class RobotRemyVisualizer {
	public static final String ROBOT_POSE_LOG_KEY = "Visualization/RobotRemy/RobotPose";
	public static final String COMPONENT_POSES_LOG_KEY = "Visualization/RobotRemy/ComponentPoses";

	private final Supplier<Pose2d> robotPoseSupplier;
	private final DoubleSupplier hoodRotations;
	private final DoubleSupplier flywheelRotations;
	private final DoubleSupplier intakeExtensionMeters;
	private final DoubleSupplier climberRotations;

	public static final LoggedTunableNumber hood = new LoggedTunableNumber("HoodPosition/Sim", 0);

	private static final Translation3d SHOOTER_OFFSET = new Translation3d(-0.103310, 0, 0.423); // from center of robot
																								// to center of flywheel

	private static final Translation3d CLIMBER_OFFSET = new Translation3d(-0.290, 0, 0.512); // from center of robot to
																								// center of climber

	public RobotRemyVisualizer(Supplier<Pose2d> robotPoseSupplier, DoubleSupplier hoodRotations,
			DoubleSupplier flywheelRotations, DoubleSupplier intakeExtensionMeters, DoubleSupplier climberRotations) {
		this.robotPoseSupplier = robotPoseSupplier;
		this.hoodRotations = hoodRotations;
		this.flywheelRotations = flywheelRotations;
		this.intakeExtensionMeters = intakeExtensionMeters;
		this.climberRotations = climberRotations;
	}

	/**
	 * Pushes latest robot + component transforms for custom-asset visualization.
	 */
	public void periodic() {
		Logger.recordOutput(ROBOT_POSE_LOG_KEY, new Pose3d(robotPoseSupplier.get()));
		Logger.recordOutput(COMPONENT_POSES_LOG_KEY, componentPoses(intakeExtensionMeters.getAsDouble(),
				hoodRotations.getAsDouble(), flywheelRotations.getAsDouble(), climberRotations.getAsDouble()));
	}

	/**
	 * Component transforms in the existing Remy CAD frame; rack extension is
	 * metres.
	 */
	public static Pose3d[] componentPoses(double extensionMeters, double hoodRotations, double flywheelRotations,
			double climberRotations) {
		double intakeExtensionX = extensionMeters * Math.cos(MapleSimConstants.INTAKE_ANGLE_RADIANS)
				* MapleSimConstants.INTAKE_EXTENSION_X_SIGN;
		return new Pose3d[]{
				Pose3d.kZero.rotateAround(SHOOTER_OFFSET,
						new Rotation3d(0.0, Units.rotationsToRadians(flywheelRotations), 0.0)),
				Pose3d.kZero.rotateAround(SHOOTER_OFFSET,
						new Rotation3d(0.0, Units.rotationsToRadians(hoodRotations), 0.0)),
				new Pose3d(intakeExtensionX, 0, -extensionMeters * Math.sin(MapleSimConstants.INTAKE_ANGLE_RADIANS),
						Rotation3d.kZero),
				new Pose3d(intakeExtensionX, 0, 0, Rotation3d.kZero),
				new Pose3d(intakeExtensionX, 0, 0, Rotation3d.kZero), Pose3d.kZero.rotateAround(CLIMBER_OFFSET,
						new Rotation3d(0.0, -Units.rotationsToRadians(climberRotations), 0.0))};
	}
}
