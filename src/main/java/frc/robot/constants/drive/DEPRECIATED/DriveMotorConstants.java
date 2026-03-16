package frc.robot.constants.drive.DEPRECIATED;

/** Constants and hardware mappings for drivetrain wheel motors. */
public class DriveMotorConstants {
	// by default, the drive is set to the RoboRio's CANBus (you can also make it
	// the rio it by doing
	// "")
	// change this value if using CANivore to CANivore's Bus name, set in Phoenix
	// Tuner X
	// (if necessary, do this in AzimuthMotorConstants.java if drive motors are
	// connected
	// to CANivore as well)
	public static final String canBusName = "MECO CANIvore";

	/** Closed-loop and feedforward gains for one drive motor. */
	public record DriveMotorGains(double kP, double kI, double kD, double kS, double kV, double kA) {
	};

	/** Hardware mapping and mechanism constants for one drive motor instance. */
	public record DriveMotorHardwareConfig(int[] canIds, boolean[] reversed, double gearRatio, double currentLimit,
			String canBus) {
	}

	public static final DriveMotorHardwareConfig FRONT_LEFT_CONFIG = new DriveMotorHardwareConfig(new int[]{2},
			new boolean[]{false}, DriveConstants.driveMotorGearRatio, 60, canBusName);

	public static final DriveMotorHardwareConfig FRONT_RIGHT_CONFIG = new DriveMotorHardwareConfig(new int[]{4},
			new boolean[]{false}, DriveConstants.driveMotorGearRatio, 60, canBusName);

	public static final DriveMotorHardwareConfig BACK_LEFT_CONFIG = new DriveMotorHardwareConfig(new int[]{6},
			new boolean[]{false}, DriveConstants.driveMotorGearRatio, 60, canBusName);

	public static final DriveMotorHardwareConfig BACK_RIGHT_CONFIG = new DriveMotorHardwareConfig(new int[]{8},
			new boolean[]{false}, DriveConstants.driveMotorGearRatio, 60, canBusName);

	public static final DriveMotorGains DRIVE_MOTOR_GAINS = new DriveMotorGains(1, 0, 0, 0.25, 0.5, 0);
}
