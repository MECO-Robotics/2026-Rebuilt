package frc.robot.systemcheck;

/** Low-rate vendor telemetry populated by mechanism IO implementations. */
public final class MotorHealthData {
	public double[] temperaturesCelsius = new double[0];
	public String[] activeFaults = new String[0];
}
