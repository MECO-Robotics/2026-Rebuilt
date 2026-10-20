package frc.robot.navigation;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.simulation.HybridSimulationEnvironment;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import org.littletonrobotics.junction.Logger;

/**
 * Starting-pose setup and observable outcome for a desktop-only whole-auto
 * trial.
 */
public final class HybridSimulationAuto extends WrapperCommand {
	private final HybridCommandAuto auto;
	private final CommandSwerveDrivetrain drive;
	private final PathPlannerPath first;
	private final String autoName;
	private boolean allowed;
	private double started;
	private Result result;

	public record Result(String auto, String alliance, String outcome, double seconds, int completedPaths,
			int totalPaths, int recoveryAttempts, boolean productionBudgetExceeded, String reason) {
		public String csv() {
			return quote(auto) + "," + alliance + "," + outcome + "," + seconds + "," + completedPaths + ","
					+ totalPaths + "," + recoveryAttempts + "," + productionBudgetExceeded + "," + quote(reason) + "\n";
		}
		private static String quote(String text) {
			return "\"" + text.replace("\"", "\"\"") + "\"";
		}
	}

	HybridSimulationAuto(String name, HybridCommandAuto auto, CommandSwerveDrivetrain drive, PathPlannerPath first) {
		super(auto);
		HybridSimulationEnvironment.requireEnabled();
		this.auto = auto;
		this.drive = drive;
		this.first = first;
		this.autoName = name;
		setName("SIM TEST | " + name);
	}

	public Pose2d startingPose() {
		var pose = new Pose2d(first.getPoint(0).position, first.getIdealStartingState().rotation());
		return !first.preventFlipping
				&& DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red
						? FlippingUtil.flipFieldPose(pose)
						: pose;
	}

	@Override
	public void initialize() {
		allowed = HybridSimulationEnvironment.enabled() && DriverStation.getAlliance().isPresent();
		result = null;
		started = Timer.getFPGATimestamp();
		SmartDashboard.putNumber("HybridSim/ElapsedSeconds", 0);
		SmartDashboard.putNumber("HybridSim/CompletedPaths", 0);
		SmartDashboard.putNumber("HybridSim/TotalPaths", auto.totalPaths());
		SmartDashboard.putNumber("HybridSim/RecoveryAttempts", 0);
		SmartDashboard.putBoolean("HybridSim/ProductionBudgetExceeded", false);
		SmartDashboard.putString("HybridSim/Reason", "");
		if (!allowed) {
			drive.stop();
			return;
		}
		// One reset before execution; recovery never teleports or resets odometry.
		drive.resetPose(startingPose());
		SmartDashboard.putString("HybridSim/Environment", "EMPTY TEST ARENA / GROUND-TRUTH NAVIGATION");
		SmartDashboard.putString("HybridSim/Auto", autoName);
		SmartDashboard.putString("HybridSim/Outcome", "RUNNING");
		super.initialize();
	}

	@Override
	public void execute() {
		if (allowed) {
			super.execute();
			SmartDashboard.putNumber("HybridSim/ElapsedSeconds", Timer.getFPGATimestamp() - started);
			SmartDashboard.putNumber("HybridSim/CompletedPaths", auto.completedPaths());
			SmartDashboard.putNumber("HybridSim/RecoveryAttempts", auto.recoveryAttempts());
		}
	}

	@Override
	public boolean isFinished() {
		return !allowed || super.isFinished();
	}

	@Override
	public void end(boolean interrupted) {
		if (allowed)
			super.end(interrupted);
		drive.stop();
		double elapsed = Timer.getFPGATimestamp() - started;
		String reason = !allowed
				? "Desktop profile or alliance unavailable"
				: interrupted ? "Interrupted/disabled" : auto.failure();
		String outcome = !allowed || interrupted ? "INTERRUPTED" : reason.isEmpty() ? "COMPLETE" : "SAFE_ABORT";
		result = new Result(autoName, DriverStation.getAlliance().map(Enum::name).orElse("Unknown"), outcome, elapsed,
				allowed ? auto.completedPaths() : 0, auto.totalPaths(), allowed ? auto.recoveryAttempts() : 0,
				elapsed > AutoSafetyConfig.defaults().missionTimeout(), reason);
		SmartDashboard.putString("HybridSim/Outcome", outcome);
		SmartDashboard.putString("HybridSim/Reason", reason);
		SmartDashboard.putNumber("HybridSim/ElapsedSeconds", elapsed);
		SmartDashboard.putBoolean("HybridSim/ProductionBudgetExceeded", result.productionBudgetExceeded());
		Logger.recordOutput("HybridSim/Result", result.toString());
		if (HybridSimulationEnvironment.enabled()) {
			try {
				var file = Path.of("build/reports/hybrid/interactive-runs.csv");
				Files.createDirectories(file.getParent());
				if (!Files.exists(file))
					Files.writeString(file, CSV_HEADER);
				Files.writeString(file, result.csv(), StandardOpenOption.APPEND);
			} catch (java.io.IOException ex) {
				DriverStation.reportWarning("Could not save hybrid simulation report: " + ex.getMessage(), false);
			}
		}
	}

	public static final String CSV_HEADER = "auto,alliance,outcome,seconds,completed_paths,total_paths,recovery_attempts,production_budget_exceeded,reason\n";

	public Result result() {
		return result;
	}
}
