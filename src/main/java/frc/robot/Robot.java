package frc.robot;

import com.revrobotics.util.StatusLogger;

import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.BuildConstants;
import frc.robot.constants.Constants;
import frc.robot.simulation.RobotSimulation;
import java.nio.file.Files;
import java.nio.file.Path;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
	private Command autonomousCommand;
	private RobotContainer robotContainer;

	public Robot() {
		RobotSimulation.configureArenaOverride(Constants.currentMode);
		// Record metadata
		Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
		Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
		Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
		Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
		Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
		Logger.recordMetadata("GitDirty", switch (BuildConstants.DIRTY) {
			case 0 -> "All changes committed";
			case 1 -> "Uncommitted changes";
			default -> "Unknown";
		});

		// Set up data receivers & replay source
		switch (Constants.currentMode) {
			case REAL :
				// Prefer the USB stick; preserve logs on the roboRIO if no USB is mounted.
				Path usbLogDirectory = Path.of("/U/logs");
				Path onboardLogDirectory = Path.of("/home/lvuser/logs");
				Path logDirectory = Files.isDirectory(Path.of("/U")) ? usbLogDirectory : onboardLogDirectory;
				try {
					Files.createDirectories(logDirectory);
					Logger.addDataReceiver(new WPILOGWriter(logDirectory.toString()));
				} catch (Exception exception) {
					// The onboard directory is a last resort if the USB path cannot be prepared.
					if (logDirectory.equals(usbLogDirectory)) {
						try {
							Files.createDirectories(onboardLogDirectory);
							Logger.addDataReceiver(new WPILOGWriter(onboardLogDirectory.toString()));
						} catch (Exception fallbackException) {
							throw new IllegalStateException("Unable to initialize USB or onboard WPILOG storage",
									fallbackException);
						}
					} else {
						throw new IllegalStateException("Unable to initialize onboard WPILOG storage", exception);
					}
				}
				Logger.addDataReceiver(new NT4Publisher());
				break;

			case SIM :
				// Running a physics simulator, log to NT
				Logger.addDataReceiver(new NT4Publisher());
				break;

			case REPLAY :
				// Replaying a log, set up replay source
				setUseTiming(false); // Run as fast as possible
				String logPath = LogFileUtil.findReplayLog();
				Logger.setReplaySource(new WPILOGReader(logPath));
				Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
				break;
		}

		// Initialize URCL
		Logger.registerURCL(URCL.startExternal());
		StatusLogger.disableAutoLogging(); // Disable REVLib's built-in logging

		// Start AdvantageKit logger
		Logger.start();

		// Instantiate our RobotContainer. This will perform all our button bindings,
		// and put our autonomous chooser on the dashboard.
		robotContainer = new RobotContainer();
	}

	/** This function is called periodically during all modes. */
	@Override
	public void robotPeriodic() {
		// Optionally switch the thread to high priority to improve loop
		// timing (see the template project documentation for details)
		Threads.setCurrentThreadPriority(true, 99);

		// Runs the Scheduler. This is responsible for polling buttons, adding
		// newly-scheduled commands, running already-scheduled commands, removing
		// finished or interrupted commands, and running subsystem periodic() methods.
		// This must be called from the robot's periodic block in order for anything in
		// the Command-based framework to work.
		CommandScheduler.getInstance().run();
		// A completed or interrupted auto must not fall through to mechanism defaults.
		if (isAutonomousEnabled() && (autonomousCommand == null || !autonomousCommand.isScheduled())) {
			robotContainer.stopAutonomousOutputs();
		}
		if (Constants.currentMode == Constants.Mode.SIM || Constants.currentMode == Constants.Mode.REPLAY) {
			robotContainer.updateVisualization();
		}

		// Update dashboard outputs
		robotContainer.updateDashboardOutputs();

		// Return to non-RT thread priority (do not modify the first argument)
		Threads.setCurrentThreadPriority(false, 10);
	}

	/** This function is called once when the robot is disabled. */
	@Override
	public void disabledInit() {
		robotContainer.systemCheckDisabled();
		CommandScheduler.getInstance().cancelAll();
		robotContainer.drivetrain.stop();
	}

	/** This function is called periodically when disabled. */
	@Override
	public void disabledPeriodic() {
		robotContainer.drivetrain.stop();
	}

	/**
	 * This autonomous runs the autonomous command selected by your
	 * {@link RobotContainer} class.
	 */
	@Override
	public void autonomousInit() {
		autonomousCommand = robotContainer.getAutonomousCommand();
		if (Constants.currentMode != Constants.Mode.REAL) {
			robotContainer.simulationAutonomousInit(autonomousCommand);
		}

		// schedule the autonomous command (example)
		if (autonomousCommand != null) {
			CommandScheduler.getInstance().schedule(autonomousCommand);
		}
	}

	/**
	 * Stop immediately when autonomous is exited, including a mode change or
	 * disable.
	 */
	@Override
	public void autonomousExit() {
		robotContainer.stopAutonomousOutputs();
	}

	/** This function is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {
	}

	/** This function is called once when teleop is enabled. */
	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
		robotContainer.simulationTeleopInit();
	}

	/** This function is called periodically during operator control. */
	@Override
	public void teleopPeriodic() {
	}

	/** This function is called once when test mode is enabled. */
	@Override
	public void testInit() {
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();
		robotContainer.systemCheckTestInit();
	}

	/** This function is called periodically during test mode. */
	@Override
	public void testPeriodic() {
	}

	/** This function is called once when Test mode is exited. */
	@Override
	public void testExit() {
		robotContainer.systemCheckTestExit();
	}

	/** This function is called once when the robot is first started up. */
	@Override
	public void simulationInit() {
	}

	/** This function is called periodically whilst in simulation. */
	@Override
	public void simulationPeriodic() {
		robotContainer.simulationPeriodic();
	}
}
