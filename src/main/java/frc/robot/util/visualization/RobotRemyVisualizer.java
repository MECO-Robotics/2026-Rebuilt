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
	private final DoubleSupplier intakeRackRotations;
	private final DoubleSupplier climberRotations;

	public static final LoggedTunableNumber hood = new LoggedTunableNumber("HoodPosition/Sim", 0);

	private static final Translation3d SHOOTER_OFFSET = new Translation3d(-0.103310, 0, 0.423); // from center of robot
																								// to center of flywheel

	private static final Translation3d CLIMBER_OFFSET = new Translation3d(-0.290, 0, 0.512); // from center of robot to
																								// center of climber

	public RobotRemyVisualizer(Supplier<Pose2d> robotPoseSupplier, DoubleSupplier hoodRotations,
			DoubleSupplier flywheelRotations, DoubleSupplier intakeRackRotations, DoubleSupplier climberRotations) {
		this.robotPoseSupplier = robotPoseSupplier;
		this.hoodRotations = hoodRotations;
		this.flywheelRotations = flywheelRotations;
		this.intakeRackRotations = intakeRackRotations;
		this.climberRotations = climberRotations;
	}

	/**
	 * Pushes latest robot + component transforms for custom-asset visualization.
	 */
	public void periodic() {
		Logger.recordOutput(ROBOT_POSE_LOG_KEY, new Pose3d(robotPoseSupplier.get()));
		Logger.recordOutput(COMPONENT_POSES_LOG_KEY, new Pose3d[]{
				// 0: flywheel
				Pose3d.kZero.rotateAround(SHOOTER_OFFSET,
						new Rotation3d(0.0, Units.rotationsToRadians(flywheelRotations.getAsDouble()), 0.0)),
				// 1: hood
				Pose3d.kZero.rotateAround(SHOOTER_OFFSET,
						new Rotation3d(0.0, Units.rotationsToRadians(hoodRotations.getAsDouble()), 0.0)),
				// 2: intake rack
				new Pose3d(intakeRackRotations.getAsDouble() * Math.cos(MapleSimConstants.INTAKE_ANGLE_RADIANS), 0,
						-intakeRackRotations.getAsDouble() * Math.sin(MapleSimConstants.INTAKE_ANGLE_RADIANS),
						Rotation3d.kZero),
				// 3: intake kicker bar
				new Pose3d(intakeRackRotations.getAsDouble() * Math.cos(MapleSimConstants.INTAKE_ANGLE_RADIANS), 0.0,
						0.0, Rotation3d.kZero),
				// 4: hopper
				new Pose3d(intakeRackRotations.getAsDouble() * Math.cos(MapleSimConstants.INTAKE_ANGLE_RADIANS), 0.0,
						0.0, Rotation3d.kZero),
				// 5: climber
				Pose3d.kZero.rotateAround(CLIMBER_OFFSET,
						new Rotation3d(0.0, -Units.rotationsToRadians(climberRotations.getAsDouble()), 0.0))});
	}
}
