package frc.robot.simulation;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.subsystems.IntakeConstants;
import frc.robot.constants.subsystems.ShooterConstants;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.position_joint.PositionJoint;

/**
 * Gates simulated fuel transfer on the same actuator requests used on hardware.
 */
public record FuelTransportSim(PositionJoint rack, Flywheel intake, Flywheel conveyor, Flywheel bottomIndexer,
		Flywheel topIndexer) {
	public boolean canCollect() {
		return DriverStation.isEnabled() && deployed() && moving(intake, IntakeConstants.ROLLER_PRESETS.INTAKE.get())
				&& moving(conveyor, -IntakeConstants.ROLLER_PRESETS.INTAKE.get() / 2.0)
				&& moving(bottomIndexer, -ShooterConstants.INDEXER_PRESET.FEED_BOTTOM.get() / 2.0)
				&& moving(topIndexer, -ShooterConstants.INDEXER_PRESET.FEED_TOP.get() / 2.0);
	}

	public boolean canFeed() {
		return DriverStation.isEnabled() && moving(conveyor, -ShooterConstants.CONVEYOR_PRESET.FEED.get())
				&& moving(bottomIndexer, ShooterConstants.INDEXER_PRESET.FEED_BOTTOM.get())
				&& moving(topIndexer, ShooterConstants.INDEXER_PRESET.FEED_TOP.get());
	}

	public boolean canEject() {
		return DriverStation.isEnabled() && deployed() && moving(intake, IntakeConstants.ROLLER_PRESETS.EJECT.get())
				&& moving(conveyor, IntakeConstants.ROLLER_PRESETS.INTAKE.get() / 2.0)
				&& moving(bottomIndexer, -ShooterConstants.INDEXER_PRESET.FEED_BOTTOM.get())
				&& moving(topIndexer, -ShooterConstants.INDEXER_PRESET.FEED_TOP.get());
	}

	private boolean deployed() {
		return rack.getPosition() >= IntakeConstants.RACK_PRESETS.DEPLOY.get() - 0.02;
	}

	private static boolean moving(Flywheel motor, double expectedVoltage) {
		return allowsMotion(motor.getCommandedVoltage(), motor.getVelocity(), expectedVoltage);
	}

	static boolean allowsMotion(double voltage, double velocity, double expectedVoltage) {
		// Encoder signs include controller inversion, so require measured motion
		// and compare direction using the robot's logical voltage convention.
		return Double.isFinite(voltage) && Double.isFinite(velocity) && Double.isFinite(expectedVoltage)
				&& Math.abs(voltage) > 0.1 && Math.abs(velocity) > 0.5
				&& Math.signum(voltage) == Math.signum(expectedVoltage);
	}
}
