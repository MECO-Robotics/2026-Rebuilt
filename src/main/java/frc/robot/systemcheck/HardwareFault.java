package frc.robot.systemcheck;

/** Hardware-specific context captured when a system-check stage fails. */
public record HardwareFault(String hardware, String faultType, String details, String canBus, int canId,
		int chainPosition, String upstreamNeighbor, String downstreamNeighbor, boolean physicalOrderVerified) {
}
