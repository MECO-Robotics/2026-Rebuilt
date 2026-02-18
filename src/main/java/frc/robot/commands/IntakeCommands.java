package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.flywheel.FlywheelVoltageCommand;
import frc.robot.commands.position_joint.PositionJointPositionCommand;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.position_joint.PositionJoint;
import frc.robot.util.mechanical_advantage.LoggedTunableNumber;

public class IntakeCommands {

  /** Intake rotation preset positions. */
  public static final class ROTATION_POSITIONS {
    public static final LoggedTunableNumber STOW =
        new LoggedTunableNumber("IntakePosition/Stow", 0);
    public static final LoggedTunableNumber DEPLOY =
        new LoggedTunableNumber("IntakePosition/Deploy", Units.degreesToRotations(101));
    public static final LoggedTunableNumber SAFE =
        new LoggedTunableNumber("IntakePosition/Safe", Units.degreesToRotations(101));
  }

  /** Intake roller preset voltages. */
  public final class ROLLER_VOLTS {
    public static final LoggedTunableNumber INTAKE =
        new LoggedTunableNumber("IntakeVolts/IntakeSpeed", -6);
    public static final LoggedTunableNumber SLOW =
        new LoggedTunableNumber("IntakeVolts/Slow", -1.5);
    public static final LoggedTunableNumber EJECT =
        new LoggedTunableNumber("IntakeVolts/Eject", 12);
    public static final LoggedTunableNumber STOP = new LoggedTunableNumber("IntakeVolts/Stop", 0);
  }

  /** Stows the intake by moving the rotation motor to the stop position and stopping the roller. */
  public static Command stowIntake(PositionJoint rotationMotor, Flywheel rollerMotor) {
    return Commands.parallel(
        new PositionJointPositionCommand(rotationMotor, ROTATION_POSITIONS.STOW),
        new FlywheelVoltageCommand(rollerMotor, ROLLER_VOLTS.STOP));
  }

  /**
   * Deploys the intake by moving the rotation motor to the down position and setting the roller
   * motor to intake speed.
   */
  public static Command deployIntake(PositionJoint rotationMotor, Flywheel rollerMotor) {
    return Commands.parallel(
        new PositionJointPositionCommand(rotationMotor, ROTATION_POSITIONS.DEPLOY),
        new FlywheelVoltageCommand(rollerMotor, ROLLER_VOLTS.INTAKE));
  }

  /**
   * Spins the wheels to intake, and moves the rotation motor repeatedly up and down for feeding stuck balls to the
   * shooter.
   */
  public static Command agitateIntake(PositionJoint rotationMotor, Flywheel rollerMotor) {
    return Commands.parallel(
        new FlywheelVoltageCommand(rollerMotor, ROLLER_VOLTS.EJECT),
        Commands.sequence(
                // TODO: need to fix depending if positions are positive or negative
                PositionJoint.setPosition(rotationMotor, ROTATION_POSITIONS.SAFE),
                Commands.waitUntil(
                    () -> rotationMotor.getPosition() > ROTATION_POSITIONS.SAFE.get()),
                PositionJoint.setPosition(rotationMotor, ROTATION_POSITIONS.DEPLOY),
                Commands.waitUntil(
                    () -> rotationMotor.getPosition() > ROTATION_POSITIONS.DEPLOY.get()))
            .repeatedly());
  }
}
