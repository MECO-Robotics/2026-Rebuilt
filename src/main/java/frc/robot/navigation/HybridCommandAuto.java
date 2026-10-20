package frc.robot.navigation;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
import org.littletonrobotics.junction.Logger;

/**
 * Owns the complete mechanism/drive tree; a failed drive never advances to
 * scoring.
 */
final class HybridCommandAuto extends Command {
	private final Command tree;
	private final List<HybridAutoSupervisor> followers = new ArrayList<>();
	private final BooleanSupplier healthy;
	private final Runnable stop;
	private final double timeout;
	private double started;
	private boolean running;
	private String failure;

	HybridCommandAuto(HybridAutoDefinition definition, Function<String, Command> named,
			Function<String, HybridAutoSupervisor> follower, Subsystem drive, BooleanSupplier healthy, Runnable stop,
			double timeout) {
		this.healthy = healthy;
		this.stop = stop;
		this.timeout = timeout;
		tree = definition.build(named, name -> {
			var supervisor = follower.apply(name);
			followers.add(supervisor);
			return new WrapperCommand(supervisor) {
				@Override
				public boolean isFinished() {
					// SAFE_ABORT must not look like success to a surrounding sequential group.
					return supervisor.state() == HybridAutoSupervisor.NavigationState.COMPLETE;
				}
			};
		});
		addRequirements(drive);
		addRequirements(tree.getRequirements());
		CommandScheduler.getInstance().registerComposedCommands(tree);
	}

	@Override
	public void initialize() {
		failure = "";
		Logger.recordOutput("Hybrid/AutoFailure", "");
		started = Timer.getFPGATimestamp();
		running = false;
		// Later leaves retain their last run's state until initialized.
		for (var follower : followers)
			follower.prepareForAuto();
		try {
			if (!healthy.getAsBoolean()) {
				fail("Localization unavailable before mechanisms start");
				return;
			}
			running = true;
			tree.initialize();
			checkFollowers();
		} catch (RuntimeException ex) {
			fail("Auto initialization: " + ex.getMessage());
		}
	}

	@Override
	public void execute() {
		try {
			if (!failure.isEmpty())
				return;
			if (!healthy.getAsBoolean()) {
				fail("Localization lost during auto");
				return;
			}
			if (Timer.getFPGATimestamp() - started > timeout) {
				fail("Whole-auto deadline");
				return;
			}
			tree.execute();
			checkFollowers();
		} catch (RuntimeException ex) {
			fail("Auto execution: " + ex.getMessage());
		}
	}

	private void checkFollowers() {
		for (var follower : followers) {
			if (follower.state() == HybridAutoSupervisor.NavigationState.SAFE_ABORT) {
				fail(follower.reason());
				return;
			}
		}
	}

	private void fail(String reason) {
		failure = reason;
		stop.run();
		Logger.recordOutput("Hybrid/AutoFailure", reason);
	}

	@Override
	public boolean isFinished() {
		return !failure.isEmpty() || (running && tree.isFinished());
	}

	@Override
	public void end(boolean interrupted) {
		try {
			if (running) {
				running = false;
				tree.end(interrupted || !failure.isEmpty());
			}
		} finally {
			stop.run();
		}
	}

	String failure() {
		return failure;
	}

	int completedPaths() {
		return followers.stream().mapToInt(HybridAutoSupervisor::completedObjectives).sum();
	}

	int totalPaths() {
		return followers.size();
	}

	int recoveryAttempts() {
		return followers.stream().mapToInt(HybridAutoSupervisor::recoveryAttempts).sum();
	}
}
