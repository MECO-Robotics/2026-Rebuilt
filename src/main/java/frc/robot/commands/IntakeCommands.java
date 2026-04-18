package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.subsystems.IntakeConstants.RACK_PRESETS;
import frc.robot.constants.subsystems.IntakeConstants.ROLLER_PRESETS;
import frc.robot.simulation.IntakeSim;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.position_joint.PositionJoint;

/**
 * Factory methods for coordinated intake-rack and intake-roller command groups.
 */
public class IntakeCommands {

	private static IntakeSim intakeSimulation;

	/**
	 * Registers the simulation helper used to mirror intake state in desktop sim.
	 */
	public static void setIntakeSimulation(IntakeSim sim) {
		intakeSimulation = sim;
	}

	/** Starts the simulated intake when simulation support is available. */
	private static Command activateIntakeSimulation() {
		return intakeSimulation != null ? intakeSimulation.startIntake() : Commands.none();
	}

	/** Stops the simulated intake when simulation support is available. */
	private static Command deactivateIntakeSimulation() {
		return intakeSimulation != null ? intakeSimulation.stopIntake() : Commands.none();
	}

	/**
	 * Stows the intake using open-loop rack velocity while running the roller and
	 * conveyor inward.
	 */
	public static Command stowIntakeVelocity(PositionJoint rack, Flywheel roller, Flywheel conveyer) {
		return Commands.parallel(PositionJoint.setVelocity(rack, () -> -0.39),
				Flywheel.setVoltage(conveyer, () -> -ROLLER_PRESETS.INTAKE.getAsDouble()),
				Flywheel.setVoltage(roller, ROLLER_PRESETS.INTAKE), deactivateIntakeSimulation());
	}

	/**
	 * Stows the intake to the configured safe rack preset while running the roller
	 * and conveyor inward.
	 */
	public static Command stowIntake(PositionJoint rack, Flywheel roller, Flywheel conveyer) {
		return Commands.parallel(PositionJoint.setPosition(rack, RACK_PRESETS.SAFE, true),
				Flywheel.setVoltage(conveyer, () -> -ROLLER_PRESETS.INTAKE.getAsDouble()),
				Flywheel.setVoltage(roller, ROLLER_PRESETS.INTAKE), deactivateIntakeSimulation());
	}

	/**
	 * Deploys the intake using open-loop rack velocity while keeping the roller
	 * idle.
	 */
	public static Command deployIntakeVelocity(PositionJoint rotationMotor, Flywheel rollerMotor) {
		return Commands.parallel(PositionJoint.setVelocity(rotationMotor, () -> 0.39),
				Flywheel.setVoltage(rollerMotor, ROLLER_PRESETS.IDLE), activateIntakeSimulation());
	}

	/**
	 * Deploys the intake to the configured rack preset while keeping the roller
	 * idle.
	 */
	public static Command deployIntake(PositionJoint rotationMotor, Flywheel rollerMotor) {
		return Commands.parallel(PositionJoint.setPosition(rotationMotor, RACK_PRESETS.DEPLOY, true),
				Flywheel.setVoltage(rollerMotor, ROLLER_PRESETS.IDLE), activateIntakeSimulation());
	}

	/** Runs only the intake roller at the configured intake voltage. */
	public static Command spinIntake(Flywheel rollerMotor) {
		return Commands.parallel(Flywheel.setVoltage(rollerMotor, ROLLER_PRESETS.INTAKE), activateIntakeSimulation());
	}

	/** Stops the intake roller and clears the simulated intake-running state. */
	public static Command idleIntake(Flywheel rollerMotor) {
		return Commands.parallel(Flywheel.setVoltage(rollerMotor, ROLLER_PRESETS.IDLE), deactivateIntakeSimulation());
	}

	/** Idles the rack, roller, and conveyor together. */
	public static Command idle(PositionJoint rack, Flywheel roller, Flywheel conveyer) {
		return Commands.parallel(PositionJoint.setVelocity(rack, () -> 0), Flywheel.setVoltage(conveyer, () -> 0),
				Flywheel.setVoltage(roller, () -> 0), deactivateIntakeSimulation());
	}
}
