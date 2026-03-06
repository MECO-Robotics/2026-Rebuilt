package frc.robot.constants.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.types.FlywheelConstants.FlywheelGains;
import frc.robot.constants.types.FlywheelConstants.FlywheelHardwareConfig;
import frc.robot.constants.types.PositionJointConstants.EncoderType;
import frc.robot.constants.types.PositionJointConstants.GravityType;
import frc.robot.constants.types.PositionJointConstants.PositionJointGains;
import frc.robot.constants.types.PositionJointConstants.PositionJointHardwareConfig;

/** Constants for intake-specific flywheel and position-joint mechanisms. */
public final class IntakeConstants {
	private IntakeConstants() {
	}

	public static final FlywheelHardwareConfig INTAKE_ROLLER_CONFIG = new FlywheelHardwareConfig(new int[]{22},
			new boolean[]{false}, 1.5, 0.025, 40, "");
	public static final FlywheelGains INTAKE_ROLLER_GAINS = new FlywheelGains(0.2, 0.0, 0.0, 0.0, 0.065, 0.0, 0.0, 0.0);

	public static final PositionJointGains INTAKE_RACK_GAINS = new PositionJointGains(10, 0.0, 0.0, 0.5, 1.0, 2.0, 0.0,
			10.0, 20.0, 0.0, 0.21, 0.2, 0.0);
	public static final PositionJointHardwareConfig INTAKE_RACK_CONFIG = new PositionJointHardwareConfig(new int[]{21},
			new boolean[]{true}, ((48 / 16) * (26 / 16)) / (((Math.PI * 10) / 10) * 0.0254), 0.01, 40,
			GravityType.COSINE, EncoderType.INTERNAL, 0, Rotation2d.fromRotations(0), "");
}
