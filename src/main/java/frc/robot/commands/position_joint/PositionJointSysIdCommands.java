package frc.robot.commands.position_joint;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.position_joint.PositionJoint;
import java.util.function.DoubleConsumer;
import org.littletonrobotics.junction.Logger;

/** Grouped SysId command factories for position-joint mechanisms. */
public class PositionJointSysIdCommands {
	private PositionJointSysIdCommands() {
	}

	public static Command quasistatic(PositionJoint positionJoint, SysIdRoutine.Direction direction) {
		String stateLogKey = positionJoint.getName() + "/SysIdState";
		return warmup(positionJoint).andThen(createRoutine(positionJoint, stateLogKey).quasistatic(direction));
	}

	public static Command dynamic(PositionJoint positionJoint, SysIdRoutine.Direction direction) {
		String stateLogKey = positionJoint.getName() + "/SysIdState";
		return warmup(positionJoint).andThen(createRoutine(positionJoint, stateLogKey).dynamic(direction));
	}

	private static SysIdRoutine createRoutine(PositionJoint positionJoint, String stateLogKey) {
		DoubleConsumer characterizationConsumer = positionJoint::setVoltage;
		return new SysIdRoutine(
				new SysIdRoutine.Config(null, null, null,
						(state) -> Logger.recordOutput(stateLogKey, state.toString())),
				new SysIdRoutine.Mechanism((voltage) -> characterizationConsumer.accept(voltage.in(Volts)), null,
						positionJoint));
	}

	private static Command warmup(PositionJoint positionJoint) {
		return positionJoint.run(() -> positionJoint.setVoltage(0.0)).withTimeout(1.0);
	}
}
