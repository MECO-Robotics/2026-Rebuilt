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
	public static Command stowIntakeVelocity(PositionJoint rack, Flywheel roller, Flywheel conveyer) {
		return Commands.parallel(PositionJoint.setVelocity(rack, () -> -0.39),
				Flywheel.setVoltage(conveyer, () -> -ROLLER_PRESETS.INTAKE.getAsDouble()),
				Flywheel.setVoltage(roller, ROLLER_PRESETS.INTAKE));
	}

	/**
	 * Stows the intake by intaking and running conveyer until the intake is below
	 * the safe position, then stopping the rollers
	 */
	public static Command stowIntake(PositionJoint rack, Flywheel roller, Flywheel conveyer) {
		return Commands.parallel(PositionJoint.setPosition(rack, RACK_PRESETS.SAFE),
				Flywheel.setVoltage(conveyer, () -> -ROLLER_PRESETS.INTAKE.getAsDouble()),
				Flywheel.setVoltage(roller, ROLLER_PRESETS.INTAKE));
	}

	/**
	 * Deploys the intake by moving the rotation motor to the down position and
	 * setting the roller motor to intake speed.
	 */
	public static Command deployIntakeVelocity(PositionJoint rotationMotor, Flywheel rollerMotor) {
		return Commands.parallel(PositionJoint.setVelocity(rotationMotor, () -> 0.39),
				Flywheel.setVoltage(rollerMotor, ROLLER_PRESETS.IDLE));
	}

	/**
	 * Deploys the intake by moving the rotation motor to the down position and
	 * setting the roller motor to intake speed.
	 */
	public static Command deployIntake(PositionJoint rotationMotor, Flywheel rollerMotor) {
		return Commands.parallel(PositionJoint.setPosition(rotationMotor, RACK_PRESETS.DEPLOY),
				Flywheel.setVoltage(rollerMotor, ROLLER_PRESETS.IDLE));
	}

	public static Command spinIntake(Flywheel rollerMotor) {
		return Flywheel.setVoltage(rollerMotor, ROLLER_PRESETS.INTAKE);
	}

	public static Command idleIntake(Flywheel rollerMotor) {
		return Flywheel.setVoltage(rollerMotor, ROLLER_PRESETS.IDLE);
	}

	public static Command idle(PositionJoint rack, Flywheel roller, Flywheel conveyer) {
		return Commands.parallel(PositionJoint.setVelocity(rack, () -> 0), Flywheel.setVoltage(conveyer, () -> 0),
				Flywheel.setVoltage(roller, () -> 0));
	}
}
