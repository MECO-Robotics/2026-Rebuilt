package frc.robot.systemcheck;

/** One low-rate status sample for a robot CAN network. */
public record CanBusSnapshot(String name, boolean available, String status, double utilization, int busOffCount,
		int txFullCount, int receiveErrorCount, int transmitErrorCount) {
}
