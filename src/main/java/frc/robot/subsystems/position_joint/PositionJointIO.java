package frc.robot.subsystems.position_joint;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.constants.Constants;
import frc.robot.constants.types.PositionJointConstants.PositionJointGains;
import frc.robot.constants.types.PositionJointConstants.PositionJointHardwareConfig;
import frc.robot.sim.position_joint.PositionJointIOSimSparkMax;
import frc.robot.sim.position_joint.PositionJointIOSimTalonFX;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;

/** Hardware abstraction for a closed-loop position-controlled joint. */
public interface PositionJointIO {
	/**
	 * Logged inputs shared by all joint implementations.
	 *
	 * <p>
	 * Units are rotations and rotations/sec unless otherwise documented by a
	 * specific implementation.
	 */
	@AutoLog
	public static class PositionJointIOInputs {
		/** Joint output position after gearing (mechanism position). */
		public double outputPosition = 0.0;
		/** Motor rotor position before gearing (motor shaft position). */
		public double rotorPosition = 0.0;
		/** Last requested joint position setpoint. */
		public double desiredPosition = 0.0;

		/** Measured mechanism velocity. */
		public double velocity = 0.0;
		/** Last requested mechanism velocity setpoint. */
		public double desiredVelocity = 0.0;

		/** Connectivity state per motor controller. */
		public boolean[] motorsConnected = {false};
		/** True when an external encoder is available and healthy. */
		public boolean encoderConnected = false;

		/** Per-motor position telemetry. */
		public double[] motorPositions = {0.0};
		/** Per-motor velocity telemetry. */
		public double[] motorVelocities = {0.0};
		/** Per-motor acceleration telemetry (optional, impl-dependent). */
		public double[] motorAccelerations = {0.0};

		/** Per-motor applied voltage telemetry. */
		public double[] motorVoltages = {0.0};
		/** Per-motor current draw telemetry. */
		public double[] motorCurrents = {0.0};
	}

	/** Refreshes all sensor and diagnostic inputs. */
	public default void updateInputs(PositionJointIOInputs inputs) {
	}

	/** Commands joint position/velocity setpoints for closed-loop control. */
	public default void setPosition(double position, double velocity) {
	}

	/**
	 * Commands a dynamic closed-loop position request with runtime profile
	 * constraints.
	 *
	 * @return true if the IO implementation handled the request directly
	 */
	public default boolean setPositionDynamic(double position, double maxVelocity, double maxAcceleration) {
		return false;
	}

	/** Commands an open-loop voltage output. */
	public default void setVoltage(double voltage) {
	}

	/** Applies controller/feedforward gains. */
	public default void setGains(PositionJointGains gains) {
	}

	/** Resets mechanism position to the implementation-defined zero. */
	public default void resetPosition() {
	}

	/** Creates a replay position-joint IO supplier. */
	public static Supplier<PositionJointIO> replayFactory(String name) {
		return () -> new PositionJointIO() {
			@Override
			public String getName() {
				return name;
			}
		};
	}

	/**
	 * Creates a mode-appropriate position joint IO.
	 *
	 * <p>
	 * Returns the supplied real implementation on real hardware, simulated IO in
	 * sim, and replay IO during log replay.
	 */
	public static PositionJointIO fromMode(String name, PositionJointHardwareConfig config,
			Supplier<PositionJointIO> subsystemSupplier, DCMotor simMotorModel) {
		return switch (Constants.currentMode) {
			case REAL -> subsystemSupplier.get();
			case SIM -> replayFactory(name).get();
			default -> replayFactory(name).get();
		};
	}

	/**
	 * Creates mode-appropriate position-joint IO using SparkMax for real hardware.
	 */
	public static PositionJointIO fromSparkMax(String name, PositionJointHardwareConfig config) {
		return switch (Constants.currentMode) {
			case REAL -> new PositionJointIOSparkMax(name, config);
			case SIM -> new PositionJointIOSimSparkMax(name, config, DCMotor.getNEO(config.canIds().length));
			default -> replayFactory(name).get();
		};
	}

	/**
	 * Creates mode-appropriate position-joint IO using TalonFX for real hardware.
	 */
	public static PositionJointIO fromTalonFX(String name, PositionJointHardwareConfig config) {
		return switch (Constants.currentMode) {
			case REAL -> new PositionJointIOTalonFX(name, config);
			case SIM -> new PositionJointIOSimTalonFX(name, config, DCMotor.getKrakenX60Foc(config.canIds().length));
			default -> replayFactory(name).get();
		};
	}

	/** Returns a unique telemetry/logging name for this joint. */
	public String getName();
}
