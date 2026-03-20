package frc.robot.constants.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.types.FlywheelConstants.FlywheelGains;
import frc.robot.constants.types.FlywheelConstants.FlywheelHardwareConfig;
import frc.robot.constants.types.PositionJointConstants.EncoderType;
import frc.robot.constants.types.PositionJointConstants.GravityType;
import frc.robot.constants.types.PositionJointConstants.PositionJointGains;
import frc.robot.constants.types.PositionJointConstants.PositionJointHardwareConfig;
import frc.robot.util.mechanical_advantage.LoggedTunableNumber;

/** Constants for intake-specific flywheel and position-joint mechanisms. */
public final class IntakeConstants {
	private IntakeConstants() {
	}

	public static final FlywheelHardwareConfig INTAKE_ROLLER_CONFIG = new FlywheelHardwareConfig(new int[]{22},
			new boolean[]{false}, 1.5, 0.025, 40, "");
	public static final FlywheelGains INTAKE_ROLLER_GAINS = new FlywheelGains(5, 0.0, 0.0, 0.25, 0.065, 0.0, 1, 0.1);

	// public static final PositionJointGains INTAKE_RACK_GAINS = new
	// PositionJointGains(0, 0.0, 0.0, 0, 0.0, 0.0, 0.0,
	// 10.0, 10.0, 0.0, 0.21, 0.01, 0.0);
	public static final PositionJointGains INTAKE_RACK_GAINS = new PositionJointGains(15, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
			1.0, 5.0, -1, 1, 0.02, 0.0);
	public static final PositionJointHardwareConfig INTAKE_RACK_CONFIG = new PositionJointHardwareConfig(new int[]{21},

			new boolean[]{false}, (5 * (48 / 16) * (22 / 12)) / (((Math.PI * 10) / 10) * 0.0254), 0.01, 30,
			GravityType.COSINE, EncoderType.INTERNAL, 0, Rotation2d.fromRotations(0), "");

	/** Intake rotation preset positions. */
	public static final class RACK_PRESETS {
		public static final LoggedTunableNumber STOW = new LoggedTunableNumber("Presets/IntakePosition/Stow", -0.39);
		public static final LoggedTunableNumber DEPLOY = new LoggedTunableNumber("Presets/IntakePosition/Deploy", .39);
		public static final LoggedTunableNumber SAFE = new LoggedTunableNumber("Presets/IntakePosition/Safe", 0.19);
	}

	/** Intake roller preset voltages. */
	public final class ROLLER_PRESETS {
		public static final LoggedTunableNumber INTAKE = new LoggedTunableNumber("Presets/IntakeVolts/IntakeSpeed", 10);
		public static final LoggedTunableNumber SLOW = new LoggedTunableNumber("Presets/IntakeVolts/Slow", 7);
		public static final LoggedTunableNumber EJECT = new LoggedTunableNumber("Presets/IntakeVolts/Eject", -10);
		public static final LoggedTunableNumber IDLE = new LoggedTunableNumber("Presets/IntakeVolts/Stop", 0);
		public static final LoggedTunableNumber TIMEOUT = new LoggedTunableNumber("Presets/IntakeVolts/Timeout", 0);
	}
}
