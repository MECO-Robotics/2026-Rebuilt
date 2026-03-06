package frc.robot.commands.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.flywheel.Flywheel;
import java.util.function.DoubleSupplier;

/** Continuously applies a voltage command to a flywheel subsystem. */
public class FlywheelVoltageCommand extends Command {
	private Flywheel flywheel;
	private DoubleSupplier voltage;

	/**
	 * Creates a flywheel voltage command.
	 *
	 * @param flywheel
	 *            target flywheel subsystem
	 * @param voltage
	 *            supplier for commanded voltage
	 */
	public FlywheelVoltageCommand(Flywheel flywheel, DoubleSupplier voltage) {
		this.flywheel = flywheel;
		this.voltage = voltage;
		addRequirements(flywheel);
	}

	@Override
	public void execute() {
		flywheel.setVoltage(voltage.getAsDouble());
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
