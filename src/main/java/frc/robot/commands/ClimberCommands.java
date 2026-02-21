package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.position_joint.PositionJointPositionCommand;
import frc.robot.subsystems.position_joint.PositionJoint;
import frc.robot.util.mechanical_advantage.LoggedTunableNumber;

public class ClimberCommands {

  /** Intake rotation preset positions. */
  /** Climber rotation preset positions. */
  public static final class ROTATION_POSITIONS {
    public static final LoggedTunableNumber STOW =
        new LoggedTunableNumber("ClimberPosition/Stow", 0);
    public static final LoggedTunableNumber DEPLOY =
        new LoggedTunableNumber("ClimberPosition/Deploy", Units.degreesToRotations(101));
    public static final LoggedTunableNumber SAFE =
        new LoggedTunableNumber("ClimberPosition/Safe", Units.degreesToRotations(101));
  }

  /** Stows the climber by moving the rotation motor to the stow position. */
  public static Command stowClimber(PositionJoint rotationMotorJoint) {
    return new PositionJointPositionCommand(rotationMotorJoint, ROTATION_POSITIONS.STOW);
  }
}
