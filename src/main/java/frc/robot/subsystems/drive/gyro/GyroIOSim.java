package frc.robot.subsystems.drive.gyro;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import frc.robot.util.PhoenixUtil;
import org.ironmaple.simulation.drivesims.GyroSimulation;

/** IO implementation backed by MapleSim gyro simulation. */
public class GyroIOSim implements GyroIO {
  private final GyroSimulation gyroSimulation;

  /** Creates a sim gyro IO with a provided MapleSim gyro model. */
  public GyroIOSim(GyroSimulation gyroSimulation) {
    this.gyroSimulation = gyroSimulation;
  }

  /** Creates a sim gyro IO with a default MapleSim gyro model. */
  public GyroIOSim() {
    this(new GyroSimulation(0.0, 0.0));
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;
    inputs.yawPosition = gyroSimulation.getGyroReading();
    inputs.yawVelocityRadPerSec = gyroSimulation.getMeasuredAngularVelocity().in(RadiansPerSecond);

    inputs.odometryYawPositions = gyroSimulation.getCachedGyroReadings();
    double[] timestamps = PhoenixUtil.getSimulationOdometryTimeStamps();
    if (timestamps.length == inputs.odometryYawPositions.length) {
      inputs.odometryYawTimestamps = timestamps;
    } else {
      int sampleCount = Math.min(timestamps.length, inputs.odometryYawPositions.length);
      inputs.odometryYawTimestamps = new double[sampleCount];
      System.arraycopy(timestamps, 0, inputs.odometryYawTimestamps, 0, sampleCount);
    }
  }
}
