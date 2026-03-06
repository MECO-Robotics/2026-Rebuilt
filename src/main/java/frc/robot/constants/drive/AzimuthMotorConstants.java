package frc.robot.constants.drive;

import edu.wpi.first.math.geometry.Rotation2d;

/** Constants and hardware mappings for drivetrain azimuth/steering motors. */
public class AzimuthMotorConstants {
	// by default, the drive is set to the RoboRio's CANBus (you can also make it
	// the rio it by doing
	// "")
	// change this value if using CANivore to CANivore's Bus name, set in Phoenix
	// Tuner X
	// (if necessary, do this in DriveMotorConstants.java if drive motors are
	// connected
	// to CANivore as well)
	public static final String canBusName = "MECO CANIvore";

	/** Closed-loop and feedforward gains for one azimuth motor. */
	public record AzimuthMotorGains(double kP, double kI, double kD, double kS, double kV, double kA) {
	}

	/** Supported sensor sources for azimuth position. */
	public enum EncoderType {
		INTERNAL, EXTERNAL_CANCODER, EXTERNAL_CANCODER_PRO, EXTERNAL_DIO, EXTERNAL_SPARK
	}

	/** Hardware mapping and mechanism constants for one azimuth motor instance. */
	public record AzimuthMotorHardwareConfig(int[] canIds, boolean[] reversed, double gearRatio, double currentLimit,
			EncoderType encoderType, int encoderID, Rotation2d encoderOffset, String canBus) {
	}

	public static final AzimuthMotorHardwareConfig FRONT_LEFT_CONFIG = new AzimuthMotorHardwareConfig(new int[]{1},
			new boolean[]{false}, DriveConstants.steerMotorGearRatio, 40, EncoderType.EXTERNAL_CANCODER_PRO, 9,
			Rotation2d.fromRotations(-0.180420), canBusName);

	public static final AzimuthMotorHardwareConfig FRONT_RIGHT_CONFIG = new AzimuthMotorHardwareConfig(new int[]{3},
			new boolean[]{false}, DriveConstants.steerMotorGearRatio, 40, EncoderType.EXTERNAL_CANCODER_PRO, 10,
			Rotation2d.fromRotations(0.303711), canBusName);

	public static final AzimuthMotorHardwareConfig BACK_LEFT_CONFIG = new AzimuthMotorHardwareConfig(new int[]{5},
			new boolean[]{false}, DriveConstants.steerMotorGearRatio, 40, EncoderType.EXTERNAL_CANCODER_PRO, 11,
			Rotation2d.fromRotations(0.404297), canBusName);

	public static final AzimuthMotorHardwareConfig BACK_RIGHT_CONFIG = new AzimuthMotorHardwareConfig(new int[]{7},
			new boolean[]{false}, DriveConstants.steerMotorGearRatio, 40, EncoderType.EXTERNAL_CANCODER_PRO, 12,
			Rotation2d.fromRotations(-0.350342), canBusName);

	public static final AzimuthMotorGains AZIMUTH_MOTOR_GAINS = new AzimuthMotorGains(25, 0, 0, 0.25, 2, 0);
}
