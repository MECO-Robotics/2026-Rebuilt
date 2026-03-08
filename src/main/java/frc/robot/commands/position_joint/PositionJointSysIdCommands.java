package frc.robot.commands.position_joint;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.position_joint.PositionJoint;
import frc.robot.util.SysIdResultsPublisher;
import frc.robot.util.SysIdRunStats;
import java.util.function.DoubleConsumer;

/** Grouped SysId command factories for position-joint mechanisms. */
public class PositionJointSysIdCommands {
	private PositionJointSysIdCommands() {
	}

	public static Command quasistatic(PositionJoint positionJoint, SysIdRoutine.Direction direction) {
		String testName = positionJoint.getName() + " Quasistatic " + direction.name();
		final double[] appliedVolts = new double[]{0.0};
		return warmup(positionJoint)
				.andThen(withResults(createRoutine(positionJoint, appliedVolts).quasistatic(direction), positionJoint,
						testName, appliedVolts));
	}

	public static Command dynamic(PositionJoint positionJoint, SysIdRoutine.Direction direction) {
		String testName = positionJoint.getName() + " Dynamic " + direction.name();
		final double[] appliedVolts = new double[]{0.0};
		return warmup(positionJoint).andThen(withResults(createRoutine(positionJoint, appliedVolts).dynamic(direction),
				positionJoint, testName, appliedVolts));
	}

	private static SysIdRoutine createRoutine(PositionJoint positionJoint, double[] appliedVolts) {
		DoubleConsumer characterizationConsumer = positionJoint::setVoltage;
		return new SysIdRoutine(new SysIdRoutine.Config(null, null, null, null),
				new SysIdRoutine.Mechanism((voltage) -> {
					appliedVolts[0] = voltage.in(Volts);
					characterizationConsumer.accept(appliedVolts[0]);
				}, null, positionJoint));
	}

	private static Command withResults(Command sysIdCommand, PositionJoint positionJoint, String testName,
			double[] appliedVolts) {
		SysIdRunStats stats = new SysIdRunStats(positionJoint::getPosition, positionJoint::getVelocity, appliedVolts);
		return sysIdCommand.deadlineFor(Commands.run(stats::sample)).beforeStarting(stats::start)
				.finallyDo((interrupted) -> {
					stats.finish();
					SysIdResultsPublisher.publish(testName, interrupted, stats);
				});
	}

	private static Command warmup(PositionJoint positionJoint) {
		return positionJoint.run(() -> positionJoint.setVoltage(0.0)).withTimeout(1.0);
	}
}
