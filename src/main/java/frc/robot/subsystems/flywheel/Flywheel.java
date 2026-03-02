package frc.robot.subsystems.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.flywheel.FlywheelVelocityCommand;
import frc.robot.commands.flywheel.FlywheelVoltageCommand;
import frc.robot.subsystems.flywheel.FlywheelConstants.FlywheelGains;
import frc.robot.util.mechanical_advantage.LinearProfile;
import frc.robot.util.mechanical_advantage.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem wrapper for a velocity-controlled flywheel/roller.
 *
 * <p>This class owns profile generation and tunable gains while {@link FlywheelIO} handles
 * hardware-specific control.
 */
public class Flywheel extends SubsystemBase {
  private final FlywheelIO flywheel;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  private final String name;

  private final LoggedTunableNumber kP;
  private final LoggedTunableNumber kI;
  private final LoggedTunableNumber kD;
  private final LoggedTunableNumber kS;
  private final LoggedTunableNumber kV;
  private final LoggedTunableNumber kA;

  private final LoggedTunableNumber kMaxAccel;

  private final LoggedTunableNumber kTolerance;

  private final LoggedTunableNumber kSetpoint;

  private final LinearProfile profile;
  private double velocitySetpoint;

  private boolean voltageMode = false;

  /**
   * Creates a flywheel subsystem.
   *
   * @param io hardware implementation for this mechanism
   * @param gains default gains and profile limits
   */
  public Flywheel(FlywheelIO io, FlywheelGains gains) {
    super(io.getName());

    flywheel = io;

    name = flywheel.getName();

    kP = new LoggedTunableNumber(name + "/Gains/kP", gains.kP());
    kI = new LoggedTunableNumber(name + "/Gains/kI", gains.kI());
    kD = new LoggedTunableNumber(name + "/Gains/kD", gains.kD());
    kS = new LoggedTunableNumber(name + "/Gains/kS", gains.kS());
    kV = new LoggedTunableNumber(name + "/Gains/kV", gains.kV());
    kA = new LoggedTunableNumber(name + "/Gains/kA", gains.kA());

    kMaxAccel = new LoggedTunableNumber(name + "/Gains/kMaxAccel", gains.kMaxAccel());

    kTolerance = new LoggedTunableNumber(name + "/Gains/kTolerance", gains.kTolerance());

    kSetpoint = new LoggedTunableNumber(name + "/Gains/kSetpoint", 0.0);

    profile = new LinearProfile(gains.kMaxAccel(), 0.02);
  }

  @Override
  public void periodic() {
    flywheel.updateInputs(inputs);
    Logger.processInputs(name, inputs);

    if (!voltageMode) {
      velocitySetpoint = profile.calculateSetpoint();
      flywheel.setVelocity(velocitySetpoint);
    }

    LoggedTunableNumber.ifChanged(
        hashCode(),
        (values) -> {
          flywheel.setGains(
              new FlywheelGains(
                  values[0], values[1], values[2], values[3], values[4], values[5], values[6],
                  values[7]));

          profile.setGoal(values[8], velocitySetpoint);

          profile.setMaxAcceleration(values[6]);
        },
        kP,
        kI,
        kD,
        kS,
        kV,
        kA,
        kMaxAccel,
        kTolerance,
        kSetpoint);
  }

  /** Sets a new velocity goal for profiled closed-loop control. */
  public void setVelocity(double velocity) {
    voltageMode = false;
    profile.setGoal(velocity, velocitySetpoint);
  }

  /** Enables open-loop control and applies a direct voltage command. */
  public void setVoltage(double voltage) {
    voltageMode = true;
    flywheel.setVoltage(voltage);
  }

  /** Returns current measured velocity. */
  public double getVelocity() {
    return inputs.velocity;
  }

  /** Returns current measured mechanism position in rotations. */
  public double getPosition() {
    return inputs.position;
  }

  /** Returns the last requested velocity setpoint from the IO layer. */
  public double getVelocitySetpoint() {
    return inputs.desiredVelocity;
  }

  /** Returns true when measured velocity is within configured tolerance of setpoint. */
  public boolean isFinished() {
    return Math.abs(inputs.velocity - inputs.desiredVelocity) < kTolerance.get();
  }

  /** Builds a command that continuously sets flywheel velocity from a supplier. */
  public static Command setVelocity(Flywheel flywheel, DoubleSupplier velocity) {
    return new FlywheelVelocityCommand(flywheel, velocity);
  }

  /** Builds a command that continuously sets flywheel voltage from a supplier. */
  public static Command setVoltage(Flywheel flywheel, DoubleSupplier voltage) {
    return new FlywheelVoltageCommand(flywheel, voltage);
  }
}
