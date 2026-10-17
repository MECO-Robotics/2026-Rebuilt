package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.shooter.ShooterCommands;
import frc.robot.constants.subsystems.IntakeConstants.RACK_PRESETS;
import frc.robot.constants.subsystems.IntakeConstants.ROLLER_PRESETS;
import frc.robot.simulation.IntakeSim;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.position_joint.PositionJoint;

/**
 * Factory methods for coordinated intake-rack and intake-roller command groups.
 */
public class IntakeCommands {
	private static final double COMPLIANCE_ENTER_METERS = 0.01;
	private static final double COMPLIANCE_EXIT_METERS = 0.02;

	private static IntakeSim intakeSimulation;

	/**
	 * Registers the simulation helper used to mirror intake state in desktop sim.
	 */
	public static void setIntakeSimulation(IntakeSim sim) {
		intakeSimulation = sim;
	}

	/** Holds the rack at a saved feeding position with a saved retraction speed. */
	public static Command holdRackRetractedWhileFeeding(PositionJoint rack,
			java.util.function.BooleanSupplier feeding) {
		ShotIntakeSettings.initialize();
		return new Command() {
			private boolean wasFeeding;
			private ShotIntakeSettings settings;
			{
				addRequirements(rack);
			}
			@Override
			public void initialize() {
				wasFeeding = false;
				rack.clearProfileConstraintsOverride();
				rack.setComplianceAfterTarget(false);
			}
			@Override
			public void execute() {
				boolean isFeeding = feeding.getAsBoolean();
				if (isFeeding) {
					if (!wasFeeding)
						settings = ShotIntakeSettings.load();
					rack.setPosition(settings.holdPositionMeters(), settings.retractSpeedMetersPerSecond());
					org.littletonrobotics.junction.Logger.recordOutput("Intake/Shot/AppliedRetractSpeedMps",
							settings.retractSpeedMetersPerSecond());
					org.littletonrobotics.junction.Logger.recordOutput("Intake/Shot/AppliedHoldPositionMeters",
							settings.holdPositionMeters());
				} else {
					rack.clearProfileConstraintsOverride();
					rack.setPosition(RACK_PRESETS.DEPLOY.get());
				}
				wasFeeding = isFeeding;
			}
			@Override
			public void end(boolean interrupted) {
				rack.clearProfileConstraintsOverride();
				if (edu.wpi.first.wpilibj.DriverStation.isTeleopEnabled()) {
					rack.setPosition(RACK_PRESETS.DEPLOY.get());
				} else {
					rack.stop();
				}
			}
		}.withName("ShotIntakeHold");
	}

	/** Starts the simulated intake when simulation support is available. */
	private static Command activateIntakeSimulation() {
		return intakeSimulation != null ? intakeSimulation.startIntake() : Commands.none();
	}

	/** Stops the simulated intake when simulation support is available. */
	private static Command deactivateIntakeSimulation() {
		return intakeSimulation != null ? intakeSimulation.stopIntake() : Commands.none();
	}

	/**
	 * Stows the intake using open-loop rack velocity while running the roller and
	 * conveyor inward.
	 */
	public static Command stowIntakeVelocity(PositionJoint rack, Flywheel roller, Flywheel conveyer) {
		return Commands.parallel(PositionJoint.setVelocity(rack, () -> -0.39),
				Flywheel.setVoltage(conveyer, () -> -ROLLER_PRESETS.INTAKE.getAsDouble()),
				Flywheel.setVoltage(roller, ROLLER_PRESETS.INTAKE), deactivateIntakeSimulation());
	}

	/**
	 * Stows the intake to the configured safe rack preset while running the roller
	 * and conveyor inward.
	 */
	public static Command stowIntake(PositionJoint rack, Flywheel roller, Flywheel conveyer) {
		return Commands.parallel(PositionJoint.setPosition(rack, RACK_PRESETS.SAFE, true),
				Flywheel.setVoltage(conveyer, () -> -ROLLER_PRESETS.INTAKE.getAsDouble()),
				Flywheel.setVoltage(roller, ROLLER_PRESETS.INTAKE), deactivateIntakeSimulation());
	}

	/**
	 * Deploys the intake using open-loop rack velocity while keeping the roller
	 * idle.
	 */
	public static Command deployIntakeVelocity(PositionJoint rotationMotor, Flywheel rollerMotor) {
		return Commands.parallel(PositionJoint.setVelocity(rotationMotor, () -> 0.39),
				Flywheel.setVoltage(rollerMotor, ROLLER_PRESETS.IDLE), activateIntakeSimulation());
	}

	/**
	 * Deploys the intake to the configured rack preset while keeping the roller
	 * idle.
	 */
	public static Command deployIntake(PositionJoint rotationMotor, Flywheel rollerMotor) {
		return Commands.parallel(deployRack(rotationMotor), Flywheel.setVoltage(rollerMotor, ROLLER_PRESETS.IDLE),
				activateIntakeSimulation());
	}

	/**
	 * Deploys only the rack, leaving roller ownership to the active intake action.
	 */
	public static Command deployRack(PositionJoint rack) {
		return PositionJoint.setPosition(rack, RACK_PRESETS.DEPLOY, true)
				.beforeStarting(() -> rack.setComplianceThresholds(COMPLIANCE_ENTER_METERS, COMPLIANCE_EXIT_METERS));
	}

	/** Continuously holds the rack deployed with wall-strike compliance enabled. */
	public static Command holdDeployedRack(PositionJoint rack) {
		return PositionJoint.holdPosition(rack, RACK_PRESETS.DEPLOY, true)
				.beforeStarting(() -> rack.setComplianceThresholds(COMPLIANCE_ENTER_METERS, COMPLIANCE_EXIT_METERS));
	}

	/** Stows only the rack without claiming the intake roller or conveyor. */
	public static Command stowRack(PositionJoint rack) {
		return PositionJoint.setPosition(rack, RACK_PRESETS.STOW);
	}

	/** Runs only the intake roller at the configured intake voltage. */
	public static Command spinIntake(Flywheel rollerMotor, Flywheel conveyer) {
		return Commands.parallel(Flywheel.setVoltage(rollerMotor, ROLLER_PRESETS.INTAKE),
				Flywheel.setVoltage(conveyer, () -> -ROLLER_PRESETS.INTAKE.getAsDouble() / 2),
				activateIntakeSimulation()).finallyDo(interrupted -> {
					rollerMotor.stop();
					conveyer.stop();
					if (intakeSimulation != null) {
						intakeSimulation.setRunning(false);
					}
				});
	}

	/**
	 * Deploys the intake and runs the complete acquisition path as one command.
	 */
	public static Command acquire(PositionJoint rack, Flywheel intakeRoller, Flywheel conveyor, Flywheel bottomIndexer,
			Flywheel topIndexer) {
		return Commands.parallel(holdDeployedRack(rack), spinIntake(intakeRoller, conveyor),
				ShooterCommands.acquireFeedPath(bottomIndexer, topIndexer)).finallyDo(interrupted -> {
					intakeRoller.stop();
					conveyor.stop();
					bottomIndexer.stop();
					topIndexer.stop();
					if (intakeSimulation != null) {
						intakeSimulation.setRunning(false);
					}
				});
	}

	/**
	 * Ejects through the intake using independent conveyor and indexer directions.
	 */
	public static Command eject(Flywheel intakeRoller, Flywheel conveyor, Flywheel bottomIndexer, Flywheel topIndexer) {
		return Commands.parallel(Flywheel.setVoltage(intakeRoller, ROLLER_PRESETS.EJECT),
				Flywheel.setVoltage(conveyor, () -> ROLLER_PRESETS.INTAKE.getAsDouble() / 2.0),
				Flywheel.setVoltage(bottomIndexer,
						() -> -frc.robot.constants.subsystems.ShooterConstants.INDEXER_PRESET.FEED_BOTTOM.get()),
				Flywheel.setVoltage(topIndexer,
						() -> -frc.robot.constants.subsystems.ShooterConstants.INDEXER_PRESET.FEED_TOP.get()),
				activateIntakeSimulation()).finallyDo(interrupted -> {
					intakeRoller.stop();
					conveyor.stop();
					bottomIndexer.stop();
					topIndexer.stop();
					if (intakeSimulation != null) {
						intakeSimulation.setRunning(false);
					}
				});
	}

	/** Runs only the intake roller at the configured intake voltage. */
	public static Command reverseIntake(Flywheel rollerMotor) {
		return Commands.parallel(Flywheel.setVoltage(rollerMotor, () -> -ROLLER_PRESETS.INTAKE.get()),
				activateIntakeSimulation());
	}

	/** Stops the intake roller and clears the simulated intake-running state. */
	public static Command idleIntake(Flywheel rollerMotor) {
		return Commands.parallel(Flywheel.idle(rollerMotor), deactivateIntakeSimulation());
	}

	/** Idles the rack, roller, and conveyor together. */
	public static Command idle(PositionJoint rack, Flywheel roller, Flywheel conveyer) {
		return Commands.parallel(PositionJoint.setVelocity(rack, () -> 0), Flywheel.setVoltage(conveyer, () -> 0),
				Flywheel.setVoltage(roller, () -> 0), deactivateIntakeSimulation());
	}
}
