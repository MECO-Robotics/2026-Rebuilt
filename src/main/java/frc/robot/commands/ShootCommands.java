package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.flywheel.FlywheelVoltageCommand;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.util.mechanical_advantage.LoggedTunableNumber;

public class ShootCommands {

    /** Conveyor roller preset voltages. */
  public final class CONVEYOR_VOLTS {
    public static final LoggedTunableNumber FEED =
        new LoggedTunableNumber("ConveyorVolts/IntakeSpeed", -10);
    public static final LoggedTunableNumber SLOW =
        new LoggedTunableNumber("ConveyorVolts/Slow", -1.5);
    public static final LoggedTunableNumber EJECT =
        new LoggedTunableNumber("ConveyorVolts/Eject", 12);
    public static final LoggedTunableNumber STOP = new LoggedTunableNumber("ConveyorVolts/Stop", 0);
  }

  /** Indexer roller preset voltages. */
  public final class INDEXER_VOLTS {
    public static final LoggedTunableNumber FEED =
        new LoggedTunableNumber("ConveyorVolts/IntakeSpeed", -10);
    public static final LoggedTunableNumber FEEDOTHER =
        new LoggedTunableNumber("ConveyorVolts/IntakeSpeed", 10);
    public static final LoggedTunableNumber SLOW =
        new LoggedTunableNumber("ConveyorVolts/Slow", -4);
    public static final LoggedTunableNumber EJECT =
        new LoggedTunableNumber("ConveyorVolts/Eject", 12);
    public static final LoggedTunableNumber STOP = new LoggedTunableNumber("ConveyorVolts/Stop", 0);
  }

  /** Shooter roller preset voltages. (NOTE: MAINLY FOR TESTING/SHUTTLE (maybe))*/
  public final class SHOOTER_VOLTS {
    public static final LoggedTunableNumber SHOOT =
        new LoggedTunableNumber("ConveyorVolts/IntakeSpeed", -11);
    public static final LoggedTunableNumber SLOW =
        new LoggedTunableNumber("ConveyorVolts/Slow", -1.5);
    public static final LoggedTunableNumber EJECT =
        new LoggedTunableNumber("ConveyorVolts/Eject", 12);
    public static final LoggedTunableNumber STOP = new LoggedTunableNumber("ConveyorVolts/Stop", 0);
  }


  /** Puts the Shooter and bottom indexers in a slow idle speed, and stopping the top indexer */
  public static Command idleRollers(Flywheel bottomIntakingRoller, Flywheel topIntakingRoller, Flywheel conveyorRoller){
    return Commands.parallel(
        new FlywheelVoltageCommand(bottomIntakingRoller, INDEXER_VOLTS.SLOW),
        new FlywheelVoltageCommand(topIntakingRoller, INDEXER_VOLTS.STOP),
        new FlywheelVoltageCommand(conveyorRoller, CONVEYOR_VOLTS.STOP)
    );
  }

  /** Puts both indexers feeding towards the shooter */
  public static Command feedRollers(Flywheel bottomIntakingRoller, Flywheel topIntakingRoller, Flywheel conveyorRoller){
    return Commands.parallel(
        new FlywheelVoltageCommand(bottomIntakingRoller, INDEXER_VOLTS.FEED),
        new FlywheelVoltageCommand(topIntakingRoller, INDEXER_VOLTS.FEEDOTHER),
        new FlywheelVoltageCommand(conveyorRoller, CONVEYOR_VOLTS.FEED)
    );
  }

//   public static Command manualTest(Flywheel flywheelMotor, Flywheel bottomIntakingRoller, Flywheel topIntakingRoller){


//   }

    
}
