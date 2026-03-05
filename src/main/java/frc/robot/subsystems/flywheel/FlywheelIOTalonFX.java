package frc.robot.subsystems.flywheel;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.constants.types.FlywheelConstants.FlywheelGains;
import frc.robot.constants.types.FlywheelConstants.FlywheelHardwareConfig;

import java.util.ArrayList;

/** TalonFX-backed implementation of {@link FlywheelIO}. */
public class FlywheelIOTalonFX implements FlywheelIO {
  private final String name;

  private final TalonFX[] motors;
  private final TalonFXConfiguration leaderConfig;

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Angle> position;

  private final ArrayList<StatusSignal<Angle>> positions = new ArrayList<>();
  private final ArrayList<StatusSignal<AngularVelocity>> velocities = new ArrayList<>();

  private final ArrayList<StatusSignal<Voltage>> voltages = new ArrayList<>();
  private final ArrayList<StatusSignal<Current>> currents = new ArrayList<>();

  private final boolean[] motorsConnected;

  private final double[] motorPositions;
  private final double[] motorVelocities;

  private final double[] motorVoltages;
  private final double[] motorCurrents;

  private final Alert[] motorAlerts;

  private double velocitySetpoint = 0.0;

  private MotorAlignmentValue motorval;

  /**
   * Creates a TalonFX flywheel IO implementation.
   *
   * @param name subsystem/logging name
   * @param config hardware mapping and mechanism constants
   */
  public FlywheelIOTalonFX(String name, FlywheelHardwareConfig config) {
    this.name = name;
    CANBus canBus = new CANBus(config.canBus());
    int numMotors = config.canIds().length;

    assert numMotors > 0 && (numMotors == config.reversed().length);

    motors = new TalonFX[numMotors];
    motorsConnected = new boolean[numMotors];
    motorPositions = new double[numMotors];
    motorVelocities = new double[numMotors];
    motorVoltages = new double[numMotors];
    motorCurrents = new double[numMotors];
    motorAlerts = new Alert[numMotors];

    motors[0] = new TalonFX(config.canIds()[0], canBus);
    leaderConfig =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast)
                    .withInverted(
                        config.reversed()[0]
                            ? InvertedValue.Clockwise_Positive
                            : InvertedValue.CounterClockwise_Positive))
            .withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(config.gearRatio())
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(config.currentLimit())
                    .withSupplyCurrentLimitEnable(true));

    tryUntilOk(5, () -> motors[0].getConfigurator().apply(leaderConfig));

    velocity = motors[0].getVelocity();
    position = motors[0].getPosition();

    positions.add(motors[0].getPosition());
    velocities.add(motors[0].getVelocity());

    voltages.add(motors[0].getSupplyVoltage());
    currents.add(motors[0].getStatorCurrent());

    motorAlerts[0] =
        new Alert(
            name,
            name + " Leader Motor Disconnected! CAN ID: " + config.canIds()[0],
            AlertType.kError);

    for (int i = 1; i < config.canIds().length; i++) {
      motorval = config.reversed()[i] ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned;
      motors[i] = new TalonFX(config.canIds()[i], canBus);
      motors[i].setControl(new Follower(motors[0].getDeviceID(), motorval));

      motorAlerts[i] =
          new Alert(
              name,
              name + " Follower Motor " + i + " Disconnected! CAN ID: " + config.canIds()[i],
              AlertType.kError);

      positions.add(motors[i].getPosition());
      velocities.add(motors[i].getVelocity());

      voltages.add(motors[i].getSupplyVoltage());
      currents.add(motors[i].getStatorCurrent());
    }
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    BaseStatusSignal.refreshAll(velocity, position);

    inputs.velocity = velocity.getValueAsDouble();
    inputs.desiredVelocity = velocitySetpoint;

    inputs.position = position.getValueAsDouble();

    for (int i = 0; i < motors.length; i++) {
      motorsConnected[i] =
          BaseStatusSignal.refreshAll(
                  positions.get(i), velocities.get(i), voltages.get(i), currents.get(i))
              .isOK();

      motorPositions[i] = positions.get(i).getValueAsDouble();
      motorVelocities[i] = velocities.get(i).getValueAsDouble();

      motorVoltages[i] = voltages.get(i).getValueAsDouble();
      motorCurrents[i] = motors[i].getStatorCurrent().getValueAsDouble();

      motorAlerts[i].set(motorsConnected[i]);
    }

    inputs.motorsConnected = motorsConnected;

    inputs.motorPositions = motorPositions;
    inputs.motorVelocities = motorVelocities;

    inputs.motorVoltages = motorVoltages;
    inputs.motorCurrents = motorCurrents;
  }

  @Override
  public void setVelocity(double velocity) {
    velocitySetpoint = velocity;

    motors[0].setControl(velocityRequest.withVelocity(velocity));
  }

  @Override
  public void setVoltage(double voltage) {
    motors[0].setControl(voltageRequest.withOutput(voltage));
  }

  @Override
  public void setGains(FlywheelGains gains) {
    motors[0]
        .getConfigurator()
        .apply(
            new Slot0Configs()
                .withKP(gains.kP())
                .withKI(gains.kI())
                .withKD(gains.kD())
                .withKV(gains.kV())
                .withKA(gains.kA())
                .withKS(gains.kS()));

    System.out.println(name + " gains set to " + gains);
  }

  /** Returns this flywheel's loggable subsystem name. */
  @Override
  public String getName() {
    return name;
  }
}
