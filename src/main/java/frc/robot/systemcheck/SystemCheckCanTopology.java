package frc.robot.systemcheck;

import frc.robot.constants.drive.DrivetrainConstants;
import frc.robot.constants.subsystems.IntakeConstants;
import frc.robot.constants.subsystems.ShooterConstants;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

/**
 * CAN inventory and diagnostic wiring order used by downloadable system-check
 * reports.
 *
 * <p>
 * The device assignments come from the robot constants. The order below is an
 * initial diagnostic order only; it must be changed to match the physical
 * yellow/green wire route before the report's upstream/downstream suggestions
 * are treated as authoritative.
 */
public final class SystemCheckCanTopology {
	public static final String RIO_BUS = "rio";
	public static final String MECO_2_BUS = ShooterConstants.BOTTOM_INDEXER_ROLLER_CONFIG.canBus();
	public static final String DRIVETRAIN_BUS = DrivetrainConstants.CAN_BUS_NAME;

	/** False until the team traces and records the physical CAN wire order. */
	public static final boolean PHYSICAL_ORDER_VERIFIED = false;

	private static final List<CanChain> CHAINS = List.of(
			new CanChain(RIO_BUS, "roboRIO CAN controller", PHYSICAL_ORDER_VERIFIED, List.of(
					motor(RIO_BUS, "IntakeRack motor", "IntakeRack", 0, IntakeConstants.INTAKE_RACK_CONFIG.canIds()[0]),
					motor(RIO_BUS, "IntakeRoller leader", "IntakeRoller", 0,
							IntakeConstants.INTAKE_ROLLER_CONFIG.canIds()[0]),
					motor(RIO_BUS, "IntakeRoller follower", "IntakeRoller", 1,
							IntakeConstants.INTAKE_ROLLER_CONFIG.canIds()[1]),
					motor(RIO_BUS, "Conveyor motor", "Conveyor", 0, ShooterConstants.CONVEYOR_CONFIG.canIds()[0]),
					motor(RIO_BUS, "TopIndexer motor", "TopIndexer", 0,
							ShooterConstants.TOP_INDEXER_ROLLER_CONFIG.canIds()[0]),
					motor(RIO_BUS, "Hood motor", "Hood", 0, ShooterConstants.HOOD_CONFIG.canIds()[0]))),
			new CanChain(MECO_2_BUS, "MECO 2 CANivore", PHYSICAL_ORDER_VERIFIED,
					List.of(motor(MECO_2_BUS, "BottomIndexer motor", "BottomIndexer", 0,
							ShooterConstants.BOTTOM_INDEXER_ROLLER_CONFIG.canIds()[0]),
							motor(MECO_2_BUS, "ShooterFlywheel leader", "ShooterFlywheel", 0,
									ShooterConstants.FLYWHEEL_ROLLER_CONFIG.canIds()[0]),
							motor(MECO_2_BUS, "ShooterFlywheel follower", "ShooterFlywheel", 1,
									ShooterConstants.FLYWHEEL_ROLLER_CONFIG.canIds()[1]))),
			new CanChain(DRIVETRAIN_BUS, "MECO CANIvore", PHYSICAL_ORDER_VERIFIED,
					List.of(motor(DRIVETRAIN_BUS, "Front-left drive motor", "SwerveModule0", 0,
							DrivetrainConstants.FrontLeft.DriveMotorId),
							motor(DRIVETRAIN_BUS, "Front-left steer motor", "SwerveModule0", 1,
									DrivetrainConstants.FrontLeft.SteerMotorId),
							encoder(DRIVETRAIN_BUS, "Front-left CANcoder", "SwerveModule0",
									DrivetrainConstants.FrontLeft.EncoderId),
							motor(DRIVETRAIN_BUS, "Front-right drive motor", "SwerveModule1", 0,
									DrivetrainConstants.FrontRight.DriveMotorId),
							motor(DRIVETRAIN_BUS, "Front-right steer motor", "SwerveModule1", 1,
									DrivetrainConstants.FrontRight.SteerMotorId),
							encoder(DRIVETRAIN_BUS, "Front-right CANcoder", "SwerveModule1",
									DrivetrainConstants.FrontRight.EncoderId),
							motor(DRIVETRAIN_BUS, "Back-left drive motor", "SwerveModule2", 0,
									DrivetrainConstants.BackLeft.DriveMotorId),
							motor(DRIVETRAIN_BUS, "Back-left steer motor", "SwerveModule2", 1,
									DrivetrainConstants.BackLeft.SteerMotorId),
							encoder(DRIVETRAIN_BUS, "Back-left CANcoder", "SwerveModule2",
									DrivetrainConstants.BackLeft.EncoderId),
							motor(DRIVETRAIN_BUS, "Back-right drive motor", "SwerveModule3", 0,
									DrivetrainConstants.BackRight.DriveMotorId),
							motor(DRIVETRAIN_BUS, "Back-right steer motor", "SwerveModule3", 1,
									DrivetrainConstants.BackRight.SteerMotorId),
							encoder(DRIVETRAIN_BUS, "Back-right CANcoder", "SwerveModule3",
									DrivetrainConstants.BackRight.EncoderId),
							motor(DRIVETRAIN_BUS, "Pigeon2", "Pigeon2", 0, DrivetrainConstants.PIGEON_ID))));

	private SystemCheckCanTopology() {
	}

	/** Returns the configured CAN chains in diagnostic order. */
	public static List<CanChain> chains() {
		return CHAINS;
	}

	/** Finds configured CAN context for one health-snapshot entry. */
	public static Optional<CanDevice> findDevice(String healthGroup, int connectionIndex, boolean encoder) {
		return CHAINS.stream().flatMap(chain -> chain.devices().stream())
				.filter(device -> device.healthGroup().equals(healthGroup)
						&& device.healthConnectionIndex() == connectionIndex && device.encoder() == encoder)
				.findFirst();
	}

	/**
	 * Converts a failed device into a report entry with its configured neighbors.
	 */
	public static HardwareFault hardwareFault(CanDevice device, String faultType, String details) {
		CanChain chain = CHAINS.stream().filter(candidate -> candidate.busName().equals(device.busName())).findFirst()
				.orElseThrow();
		int index = chain.devices().indexOf(device);
		String upstream = index == 0 ? chain.controllerName() : describe(chain.devices().get(index - 1));
		String downstream = index + 1 >= chain.devices().size()
				? "CAN terminator / end of configured chain"
				: describe(chain.devices().get(index + 1));
		return new HardwareFault(device.hardwareName(), faultType, details, device.busName(), device.deviceId(),
				index + 1, upstream, downstream, chain.physicalOrderVerified());
	}

	/** Diagnoses each configured bus from a complete health snapshot. */
	public static List<CanChainDiagnosis> diagnoseChains(List<DeviceHealth> healthSnapshots) {
		Map<String, DeviceHealth> healthByName = new LinkedHashMap<>();
		for (DeviceHealth health : healthSnapshots) {
			healthByName.put(health.name(), health);
		}
		List<CanChainDiagnosis> diagnoses = new ArrayList<>();
		for (CanChain chain : CHAINS) {
			List<CanDevice> disconnected = chain.devices().stream()
					.filter(device -> !deviceConnected(device, healthByName.get(device.healthGroup()))).toList();
			String likelyBreak = "No configured device reported disconnected";
			if (!disconnected.isEmpty()) {
				CanDevice first = disconnected.get(0);
				int firstIndex = chain.devices().indexOf(first);
				String upstream = firstIndex == 0
						? chain.controllerName()
						: describe(chain.devices().get(firstIndex - 1));
				boolean laterDeviceConnected = chain.devices().subList(firstIndex + 1, chain.devices().size()).stream()
						.anyMatch(device -> deviceConnected(device, healthByName.get(device.healthGroup())));
				likelyBreak = laterDeviceConnected
						? describe(first)
								+ " is offline while a later configured device responds; inspect its power and local CAN connectors first"
						: "Inspect between " + upstream + " and " + describe(first) + " first";
				if (!chain.physicalOrderVerified()) {
					likelyBreak += "; configured order is not yet physically verified";
				}
			}
			diagnoses.add(new CanChainDiagnosis(chain.busName(), chain.physicalOrderVerified(),
					disconnected.stream().map(SystemCheckCanTopology::describe).toList(), likelyBreak));
		}
		return diagnoses;
	}

	private static boolean deviceConnected(CanDevice device, DeviceHealth health) {
		if (health == null) {
			return false;
		}
		if (device.encoder()) {
			return !health.encoderExpected() || health.encoderConnected();
		}
		boolean[] connections = health.connections();
		return device.healthConnectionIndex() >= 0 && device.healthConnectionIndex() < connections.length
				&& connections[device.healthConnectionIndex()];
	}

	private static String describe(CanDevice device) {
		return device.hardwareName() + " (CAN " + device.deviceId() + ")";
	}

	private static CanDevice motor(String busName, String hardwareName, String healthGroup, int connectionIndex,
			int deviceId) {
		return new CanDevice(busName, deviceId, hardwareName, healthGroup, connectionIndex, false);
	}

	private static CanDevice encoder(String busName, String hardwareName, String healthGroup, int deviceId) {
		return new CanDevice(busName, deviceId, hardwareName, healthGroup, -1, true);
	}

	/** One configured physical/logical CAN chain. */
	public record CanChain(String busName, String controllerName, boolean physicalOrderVerified,
			List<CanDevice> devices) {
		public CanChain {
			devices = List.copyOf(devices);
		}
	}

	/** One CAN-connected device and its source in {@link DeviceHealth}. */
	public record CanDevice(String busName, int deviceId, String hardwareName, String healthGroup,
			int healthConnectionIndex, boolean encoder) {
	}

	/** Human-readable result of checking one CAN chain. */
	public record CanChainDiagnosis(String busName, boolean physicalOrderVerified, List<String> disconnectedDevices,
			String likelyBreak) {
		public CanChainDiagnosis {
			disconnectedDevices = List.copyOf(disconnectedDevices);
		}
	}
}
