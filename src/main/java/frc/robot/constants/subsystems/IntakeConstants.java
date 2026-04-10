package frc.robot.constants.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.types.FlywheelConstants.FlywheelGains;
import frc.robot.constants.types.FlywheelConstants.FlywheelHardwareConfig;
import frc.robot.constants.types.PositionJointConstants.EncoderType;
import frc.robot.constants.types.PositionJointConstants.MechanismType;
import frc.robot.constants.types.PositionJointConstants.PositionJointGains;
import frc.robot.constants.types.PositionJointConstants.PositionJointHardwareConfig;
import frc.robot.util.mechanical_advantage.LoggedTunableNumber;

/** Constants for intake-specific flywheel and position-joint mechanisms. */
public final class IntakeConstants {
	private static final double INTAKE_MASS_KG = Units.lbsToKilograms(15.0);
	private static final double INTAKE_RACK_DRUM_RADIUS_METERS = Units.inchesToMeters(0.5);
	private static final double INTAKE_RACK_GEAR_RATIO = (3 * (48.0 / 16.0) * (22.0 / 18.0))
			/ (((Math.PI * 10.0) / 10.0) * 0.0254);
	private static final double INTAKE_RACK_LOAD_MOI_KG_METERS_SQUARED = INTAKE_MASS_KG * INTAKE_RACK_DRUM_RADIUS_METERS
			* INTAKE_RACK_DRUM_RADIUS_METERS;
	private static final double INTAKE_RACK_INPUT_SHAFT_MOI_KG_METERS_SQUARED = INTAKE_RACK_LOAD_MOI_KG_METERS_SQUARED
			/ (INTAKE_RACK_GEAR_RATIO * INTAKE_RACK_GEAR_RATIO);

	private IntakeConstants() {
	}

	public static final FlywheelHardwareConfig INTAKE_ROLLER_CONFIG = new FlywheelHardwareConfig(new int[]{22},
			new boolean[]{false}, 1.5, 0.025, 40, "");
	// public static final FlywheelGains INTAKE_ROLLER_GAINS = new FlywheelGains(5,
	// 0.0, 0.0, 0.25, 0.065, 0.0, 1, 0.1);
	public static final FlywheelGains INTAKE_ROLLER_GAINS = new FlywheelGains(0, 0.0, 0.0, 0, 0, 0.0, 1, 0.1);

	// public static final PositionJointGains INTAKE_RACK_GAINS = new
	// PositionJointGains(0, 0.0, 0.0, 0, 0.0, 0.0, 0.0,
	// 10.0, 10.0, 0.0, 0.21, 0.01, 0.0);

	public static final PositionJointGains INTAKE_RACK_GAINS = new PositionJointGains(2616.7, 0.0, 1.25, 0.0, 0.03,
			34.37, 0.02, 13.0, 3.5, 0, 0.35, 0.02, 0.0);
	public static final PositionJointHardwareConfig INTAKE_RACK_CONFIG = new PositionJointHardwareConfig(new int[]{21},
			new boolean[]{false}, INTAKE_RACK_GEAR_RATIO, INTAKE_RACK_INPUT_SHAFT_MOI_KG_METERS_SQUARED, 40,
			EncoderType.INTERNAL, 0, MechanismType.LINEAR, INTAKE_RACK_DRUM_RADIUS_METERS, Rotation2d.fromRotations(0),
			"");

	/** Intake rotation preset positions. */
	public static final class RACK_PRESETS {
		public static final LoggedTunableNumber STOW = new LoggedTunableNumber("Presets/IntakePosition/Stow", 0);
		public static final LoggedTunableNumber DEPLOY = new LoggedTunableNumber("Presets/IntakePosition/Deploy", .31);
		public static final LoggedTunableNumber SAFE = new LoggedTunableNumber("Presets/IntakePosition/Safe", 0.13);
	}

	/** Intake roller preset voltages. */
	public final class ROLLER_PRESETS {
		public static final LoggedTunableNumber INTAKE = new LoggedTunableNumber("Presets/IntakeVolts/IntakeSpeed", 11);
		public static final LoggedTunableNumber SLOW = new LoggedTunableNumber("Presets/IntakeVolts/Slow", 7);
		public static final LoggedTunableNumber EJECT = new LoggedTunableNumber("Presets/IntakeVolts/Eject", -10);
		public static final LoggedTunableNumber IDLE = new LoggedTunableNumber("Presets/IntakeVolts/Stop", 0);
		public static final LoggedTunableNumber TIMEOUT = new LoggedTunableNumber("Presets/IntakeVolts/Timeout", 0);
	}
}
