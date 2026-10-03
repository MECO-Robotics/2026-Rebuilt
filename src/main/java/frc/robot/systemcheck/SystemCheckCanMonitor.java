package frc.robot.systemcheck;

import com.ctre.phoenix6.CANBus;
import frc.robot.constants.drive.DrivetrainConstants;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/** Low-rate status polling for every CAN network used by the robot. */
public final class SystemCheckCanMonitor {
	private final List<CANBus> buses = List.of(CANBus.roboRIO(), new CANBus(SystemCheckCanTopology.MECO_2_BUS),
			new CANBus(DrivetrainConstants.CAN_BUS.getName()));

	/** Polls all CAN networks. This should be called at the 2 Hz health rate. */
	public List<CanBusSnapshot> poll() {
		List<CanBusSnapshot> snapshots = new ArrayList<>();
		for (CANBus bus : buses) {
			String displayName = bus.getName().isBlank() ? SystemCheckCanTopology.RIO_BUS : bus.getName();
			try {
				CANBus.CANBusStatus status = bus.getStatus();
				snapshots.add(new CanBusSnapshot(displayName, status.Status.isOK(), status.Status.toString(),
						status.BusUtilization, status.BusOffCount, status.TxFullCount, status.REC, status.TEC));
			} catch (RuntimeException exception) {
				snapshots.add(new CanBusSnapshot(displayName, false,
						exception.getClass().getSimpleName() + ": " + String.valueOf(exception.getMessage()),
						Double.NaN, 0, 0, 0, 0));
			}
		}
		return List.copyOf(snapshots);
	}

	/** Captures bus-off counters for comparison during an active run. */
	public static Map<String, Integer> busOffCounts(List<CanBusSnapshot> snapshots) {
		Map<String, Integer> counts = new LinkedHashMap<>();
		for (CanBusSnapshot snapshot : snapshots) {
			counts.put(snapshot.name(), snapshot.busOffCount());
		}
		return Map.copyOf(counts);
	}
}
