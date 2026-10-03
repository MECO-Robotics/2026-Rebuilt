package frc.robot.systemcheck;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * Dashboard-controlled simulation-only fault injection for diagnostic testing.
 */
final class SystemCheckFaultInjection {
	private final BooleanEntry disconnectedMotor = booleanEntry("DisconnectedMotor");
	private final BooleanEntry frozenEncoder = booleanEntry("FrozenEncoder");
	private final BooleanEntry excessiveCurrent = booleanEntry("ExcessiveCurrent");
	private final BooleanEntry highTemperature = booleanEntry("HighTemperature");
	private final BooleanEntry missingVision = booleanEntry("MissingVision");
	private final BooleanEntry lowBattery = booleanEntry("LowBattery");

	List<DeviceHealth> apply(List<DeviceHealth> source) {
		if (!RobotBase.isSimulation()) {
			return source;
		}
		List<DeviceHealth> injected = new ArrayList<>();
		for (DeviceHealth health : source) {
			DeviceHealth changed = health;
			if (disconnectedMotor.get() && health.name().equals("IntakeRoller")) {
				boolean[] connections = health.connections();
				if (connections.length > 0) {
					connections[0] = false;
				}
				changed = copy(changed, connections, changed.normalizedVelocities(), changed.currentsAmps(),
						changed.temperaturesCelsius(), changed.encoderExpected(), changed.encoderConnected(),
						append(changed.activeFaults(), "Injected disconnected motor"));
			}
			if (excessiveCurrent.get() && health.name().equals("IntakeRoller")) {
				double[] currents = changed.currentLimitsAmps();
				double[] velocities = new double[changed.normalizedVelocities().length];
				changed = copy(changed, changed.connections(), velocities, currents, changed.temperaturesCelsius(),
						changed.encoderExpected(), changed.encoderConnected(),
						append(changed.activeFaults(), "Injected excessive current"));
			}
			if (highTemperature.get() && health.name().equals("ShooterFlywheel")) {
				changed = copy(changed, changed.connections(), changed.normalizedVelocities(), changed.currentsAmps(),
						new double[]{90.0}, changed.encoderExpected(), changed.encoderConnected(),
						append(changed.activeFaults(), "Injected high temperature"));
			}
			if (missingVision.get() && health.name().equals("Vision")) {
				boolean[] connections = health.connections();
				java.util.Arrays.fill(connections, false);
				changed = copy(changed, connections, changed.normalizedVelocities(), changed.currentsAmps(),
						changed.temperaturesCelsius(), changed.encoderExpected(), changed.encoderConnected(),
						append(changed.activeFaults(), "Injected missing vision"));
			}
			injected.add(changed);
		}
		return List.copyOf(injected);
	}

	boolean frozenEncoder() {
		return RobotBase.isSimulation() && frozenEncoder.get();
	}

	double batteryVoltage(double measuredVoltage) {
		return RobotBase.isSimulation() && lowBattery.get() ? 9.0 : measuredVoltage;
	}

	void log() {
		Logger.recordOutput("SystemCheck/SimulationFaults/DisconnectedMotor", disconnectedMotor.get());
		Logger.recordOutput("SystemCheck/SimulationFaults/FrozenEncoder", frozenEncoder.get());
		Logger.recordOutput("SystemCheck/SimulationFaults/ExcessiveCurrent", excessiveCurrent.get());
		Logger.recordOutput("SystemCheck/SimulationFaults/HighTemperature", highTemperature.get());
		Logger.recordOutput("SystemCheck/SimulationFaults/MissingVision", missingVision.get());
		Logger.recordOutput("SystemCheck/SimulationFaults/LowBattery", lowBattery.get());
	}

	private static DeviceHealth copy(DeviceHealth health, boolean[] connections, double[] velocities, double[] currents,
			double[] temperatures, boolean encoderExpected, boolean encoderConnected, String[] faults) {
		return new DeviceHealth(health.name(), connections, velocities, currents, health.currentLimitsAmps(),
				temperatures, encoderExpected, encoderConnected, faults, health.deviceIds());
	}

	private static String[] append(String[] values, String added) {
		String[] result = java.util.Arrays.copyOf(values, values.length + 1);
		result[result.length - 1] = added;
		return result;
	}

	private static BooleanEntry booleanEntry(String key) {
		BooleanEntry entry = NetworkTableInstance.getDefault().getTable("SystemCheck/SimulationFaults")
				.getBooleanTopic(key).getEntry(false);
		entry.setDefault(false);
		return entry;
	}
}
