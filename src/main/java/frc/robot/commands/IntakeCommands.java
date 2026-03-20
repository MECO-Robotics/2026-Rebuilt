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

	public static void setIntakeSimulation(IntakeSim sim) {
		intakeSimulation = sim;
	}

	/**
	 * Stows the intake by intaking and running conveyer until the intake is below
	 * the safe position, then stopping the rollers
	 */
	public static Command stowIntake(PositionJoint rack, Flywheel roller, Flywheel conveyer) {
		return Commands.sequence(
				Commands.deadline(Commands.waitUntil(() -> rack.getPosition() < RACK_PRESETS.SAFE.get())),
				PositionJoint.setPosition(rack, RACK_PRESETS.STOW),
				Flywheel.setVoltage(conveyer, ROLLER_PRESETS.INTAKE),
				Flywheel.setVoltage(roller, ROLLER_PRESETS.INTAKE),
				intakeSimulation != null ? intakeSimulation.stopIntake() : Commands.none(),
				Commands.parallel(Flywheel.setVoltage(conveyer, ROLLER_PRESETS.IDLE),
						Flywheel.setVoltage(roller, ROLLER_PRESETS.IDLE)));
	}

	/**
	 * Deploys the intake by moving the rotation motor to the down position and
	 * setting the roller motor to intake speed.
	 */
	public static Command deployIntake(PositionJoint rotationMotor, Flywheel rollerMotor) {
		return Commands.parallel(PositionJoint.setPosition(rotationMotor, RACK_PRESETS.DEPLOY),
				Flywheel.setVoltage(rollerMotor, ROLLER_PRESETS.IDLE),
				intakeSimulation != null ? intakeSimulation.startIntake() : Commands.none());
	}

	public static Command spinIntake(Flywheel rollerMotor) {
		return Flywheel.setVoltage(rollerMotor, ROLLER_PRESETS.INTAKE);
	}

	public static Command idleIntake(Flywheel rollerMotor) {
		return Flywheel.setVoltage(rollerMotor, ROLLER_PRESETS.IDLE);
	}
}
