package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.flywheel.FlywheelVoltageCommand;
import frc.robot.commands.position_joint.PositionJointPositionCommand;
import frc.robot.simulation.IntakeSim;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.position_joint.PositionJoint;
import frc.robot.util.mechanical_advantage.LoggedTunableNumber;

/**
 * Factory methods for coordinated intake-rack and intake-roller command groups.
 */
public class IntakeCommands {

	private static IntakeSim intakeSimulation;

	public static void setIntakeSimulation(IntakeSim sim) {
		intakeSimulation = sim;
	}

	/** Intake rotation preset positions. */
	public static final class INTAKE_POSITIONS {
		public static final LoggedTunableNumber STOW = new LoggedTunableNumber("IntakePosition/Stow", 0);
		public static final LoggedTunableNumber DEPLOY = new LoggedTunableNumber("IntakePosition/Deploy", .21);
		public static final LoggedTunableNumber SAFE = new LoggedTunableNumber("IntakePosition/Safe", 0.1);
	}

	/** Intake roller preset voltages. */
	public final class ROLLER_VOLTS {
		public static final LoggedTunableNumber INTAKE = new LoggedTunableNumber("IntakeVolts/IntakeSpeed", 10);
		public static final LoggedTunableNumber SLOW = new LoggedTunableNumber("IntakeVolts/Slow", 7);
		public static final LoggedTunableNumber EJECT = new LoggedTunableNumber("IntakeVolts/Eject", -10);
		public static final LoggedTunableNumber STOP = new LoggedTunableNumber("IntakeVolts/Stop", 0);
	}

	/**
	 * Stows the intake by moving the rotation motor to the stop position and
	 * stopping the roller.
	 */
	public static Command stowIntake(PositionJoint rotationMotor, Flywheel rollerMotor) {
		return Commands.deadline(new PositionJointPositionCommand(rotationMotor, INTAKE_POSITIONS.STOW),
				new FlywheelVoltageCommand(rollerMotor, ROLLER_VOLTS.INTAKE),
				intakeSimulation != null ? intakeSimulation.stopIntake() : Commands.none());
	}

	/**
	 * Deploys the intake by moving the rotation motor to the down position and
	 * setting the roller motor to intake speed.
	 */
	public static Command deployIntake(PositionJoint rotationMotor, Flywheel rollerMotor) {
		return Commands.parallel(new PositionJointPositionCommand(rotationMotor, INTAKE_POSITIONS.DEPLOY),
				new FlywheelVoltageCommand(rollerMotor, ROLLER_VOLTS.INTAKE),
				intakeSimulation != null ? intakeSimulation.startIntake() : Commands.none());
	}

	/**
	 * Spins the wheels to intake, and moves the rotation motor repeatedly up and
	 * down for feeding stuck balls to the shooter.
	 */
	public static Command agitateIntake(PositionJoint rotationMotor, Flywheel rollerMotor) {
		return Commands.parallel(new FlywheelVoltageCommand(rollerMotor, ROLLER_VOLTS.EJECT), Commands.sequence(
				// TODO: need to fix depending if positions are positive or negative
				PositionJoint.setPosition(rotationMotor, INTAKE_POSITIONS.SAFE),
				Commands.waitUntil(() -> rotationMotor.getPosition() > INTAKE_POSITIONS.SAFE.get()),
				PositionJoint.setPosition(rotationMotor, INTAKE_POSITIONS.DEPLOY),
				Commands.waitUntil(() -> rotationMotor.getPosition() > INTAKE_POSITIONS.DEPLOY.get())).repeatedly());
	}
}
