package frc.robot.commands.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.constants.drive.DrivetrainConstants;
import frc.robot.constants.subsystems.ShooterConstants;
import frc.robot.constants.subsystems.ShooterConstants.CONVEYOR_PRESET;
import frc.robot.constants.subsystems.ShooterConstants.HOOD_PRESET;
import frc.robot.constants.subsystems.ShooterConstants.INDEXER_PRESET;
import frc.robot.constants.subsystems.ShooterConstants.SHOOTER_PRESET;
import frc.robot.simulation.LaunchedFuelSim;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.position_joint.PositionJoint;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Factory methods for coordinated shooter/indexer/conveyor command groups. */
public class ShooterCommands {
	private static LaunchedFuelSim launchedFuelSimulation;

	/** Registers the sim hook that spawns launched fuel during feed commands. */
	public static void setLaunchedFuelSimulation(LaunchedFuelSim sim) {
		launchedFuelSimulation = sim;
	}

	/**
	 * Idles both indexers and the conveyor without spinning the shooter flywheel.
	 */
	public static Command idleRollers(Flywheel bottomIntakingRoller, Flywheel topIntakingRoller,
			Flywheel conveyorRoller) {
		return Commands.parallel(Flywheel.idle(bottomIntakingRoller), Flywheel.idle(topIntakingRoller),
				Flywheel.idle(conveyorRoller));
	}

	/** Idles both indexers without touching the conveyor. */
	public static Command idleRollers(Flywheel bottomIntakingRoller, Flywheel topIntakingRoller) {
		return Commands.parallel(Flywheel.idle(bottomIntakingRoller), Flywheel.idle(topIntakingRoller));
	}

	/** Feeds both indexers and the conveyor toward the shooter. */
	public static Command feedRollers(Flywheel bottomIntakingRoller, Flywheel topIntakingRoller,
			Flywheel conveyorRoller) {
		return Commands.deadline(Flywheel.setVoltage(bottomIntakingRoller, INDEXER_PRESET.FEED_BOTTOM),
				Flywheel.setVoltage(topIntakingRoller, INDEXER_PRESET.FEED_TOP),
				Flywheel.setVoltage(conveyorRoller, CONVEYOR_PRESET.FEED),
				launchedFuelSimulation != null ? launchedFuelSimulation.launchCommand() : Commands.none());
	}

	/** Runs the storage indexers away from the shooter while acquiring fuel. */
	public static Command acquireFeedPath(Flywheel bottomIndexer, Flywheel topIndexer) {
		return Commands.parallel(
				Flywheel.setVoltage(bottomIndexer, () -> -INDEXER_PRESET.FEED_BOTTOM.getAsDouble() / 2.0),
				Flywheel.setVoltage(topIndexer, () -> -INDEXER_PRESET.FEED_TOP.getAsDouble() / 2.0));
	}

	/**
	 * Runs both storage indexers toward the shooter without starting the conveyor.
	 */
	public static Command runFeedIndexers(Flywheel bottomIndexer, Flywheel topIndexer) {
		return Commands.parallel(Flywheel.setVoltage(bottomIndexer, INDEXER_PRESET.FEED_BOTTOM),
				Flywheel.setVoltage(topIndexer, INDEXER_PRESET.FEED_TOP));
	}

	/** Pulses the two indexers without running the conveyor. */
	public static Command agitateIntake(Flywheel bottomIntakingRoller, Flywheel topIntakingRoller) {
		return Commands.deadline(Flywheel.setVoltage(bottomIntakingRoller, INDEXER_PRESET.FEED_BOTTOM),
				Flywheel.setVoltage(topIntakingRoller, INDEXER_PRESET.FEED_TOP),
				launchedFuelSimulation != null ? launchedFuelSimulation.launchCommand() : Commands.none());
	}

	/** Pulses the two indexers without running the conveyor. */
	public static Command unagitateIntake(Flywheel bottomIntakingRoller, Flywheel topIntakingRoller) {
		return Commands.deadline(
				Flywheel.setVoltage(bottomIntakingRoller, () -> -INDEXER_PRESET.FEED_BOTTOM.getAsDouble() / 2),
				Flywheel.setVoltage(topIntakingRoller, () -> -INDEXER_PRESET.FEED_TOP.getAsDouble() / 2),
				launchedFuelSimulation != null ? launchedFuelSimulation.launchCommand() : Commands.none());
	}

	/** Stows the hood and stops the shooter flywheel. */
	public static Command shooterIdle(Flywheel shooterRoller, PositionJoint hood) {
		return Commands.deadline(PositionJoint.setPosition(hood, HOOD_PRESET.STOW), Flywheel.idle(shooterRoller));
	}

	/** Applies the close hub shot preset. */
	public static Command hubPreset(Flywheel shooterRoller, PositionJoint hood) {
		return Commands.parallel(PositionJoint.holdPosition(hood, HOOD_PRESET.HUB),
				Flywheel.holdVelocity(shooterRoller, SHOOTER_PRESET.HUB));
	}

	/** Applies the ferry shot preset. */
	public static Command ferryPreset(Flywheel shooter, PositionJoint hood) {
		return Commands.parallel(PositionJoint.holdPosition(hood, HOOD_PRESET.FERRY),
				Flywheel.holdVelocity(shooter, SHOOTER_PRESET.FERRY));
	}

	/** Applies the trench shot preset. */
	public static Command trenchPreset(Flywheel shooter, PositionJoint hood) {
		return Commands.parallel(PositionJoint.holdPosition(hood, HOOD_PRESET.TRENCH),
				Flywheel.holdVelocity(shooter, SHOOTER_PRESET.TRENCH));
	}

	/**
	 * A driver-held hub shot that aims and spins continuously, then feeds only
	 * while every stationary-shot readiness condition is satisfied.
	 */
	public static Command coordinatedHubShot(CommandSwerveDrivetrain drive, PositionJoint hood, Flywheel shooter,
			Flywheel bottomIndexer, Flywheel topIndexer, Flywheel conveyor, DoubleSupplier xSupplier,
			DoubleSupplier ySupplier, BooleanSupplier poseReady, BooleanSupplier forceFeed,
			Consumer<Boolean> readyFeedback) {
		return coordinatedShot(drive, hood, shooter, bottomIndexer, topIndexer, conveyor, xSupplier, ySupplier,
				ShooterCommands::currentAllianceHubTarget, poseReady, forceFeed, () -> false, () -> 0.0, () -> 0.0,
				readyFeedback, false);
	}

	/**
	 * A driver-held hub shot with a dashboard-selectable calibration mode. While
	 * calibration is enabled, live hood and flywheel values replace the shot table
	 * and feeding requires the force-feed control.
	 */
	public static Command coordinatedHubShot(CommandSwerveDrivetrain drive, PositionJoint hood, Flywheel shooter,
			Flywheel bottomIndexer, Flywheel topIndexer, Flywheel conveyor, DoubleSupplier xSupplier,
			DoubleSupplier ySupplier, BooleanSupplier poseReady, BooleanSupplier forceFeed,
			BooleanSupplier calibrationMode, DoubleSupplier calibrationHoodRotations,
			DoubleSupplier calibrationFlywheelRps, Consumer<Boolean> readyFeedback) {
		return coordinatedShot(drive, hood, shooter, bottomIndexer, topIndexer, conveyor, xSupplier, ySupplier,
				ShooterCommands::currentAllianceHubTarget, poseReady, forceFeed, calibrationMode,
				calibrationHoodRotations, calibrationFlywheelRps, readyFeedback, false);
	}

	/**
	 * A driver-held shot that latches a supplied hub or ferry target when
	 * scheduled, then keeps aiming at that target until the button is released.
	 */
	public static Command coordinatedSelectedShot(CommandSwerveDrivetrain drive, PositionJoint hood, Flywheel shooter,
			Flywheel bottomIndexer, Flywheel topIndexer, Flywheel conveyor, DoubleSupplier xSupplier,
			DoubleSupplier ySupplier, Supplier<ShotTarget> targetSupplier, BooleanSupplier poseReady,
			BooleanSupplier forceFeed, BooleanSupplier calibrationMode, DoubleSupplier calibrationHoodRotations,
			DoubleSupplier calibrationFlywheelRps, Consumer<Boolean> readyFeedback) {
		return coordinatedShot(drive, hood, shooter, bottomIndexer, topIndexer, conveyor, xSupplier, ySupplier,
				targetSupplier, poseReady, forceFeed, calibrationMode, calibrationHoodRotations, calibrationFlywheelRps,
				readyFeedback, false);
	}

	/**
	 * An autonomous hub shot that waits up to two seconds for readiness and
	 * accumulates one second of ready feed time, with a three-second hard timeout.
	 */
	public static Command autonomousHubShot(CommandSwerveDrivetrain drive, PositionJoint hood, Flywheel shooter,
			Flywheel bottomIndexer, Flywheel topIndexer, Flywheel conveyor, BooleanSupplier poseReady) {
		return coordinatedShot(drive, hood, shooter, bottomIndexer, topIndexer, conveyor, () -> 0.0, () -> 0.0,
				ShooterCommands::currentAllianceHubTarget, poseReady, () -> false, () -> false, () -> 0.0, () -> 0.0,
				ready -> {
				}, true);
	}

	private static Command coordinatedShot(CommandSwerveDrivetrain drive, PositionJoint hood, Flywheel shooter,
			Flywheel bottomIndexer, Flywheel topIndexer, Flywheel conveyor, DoubleSupplier xSupplier,
			DoubleSupplier ySupplier, Supplier<ShotTarget> targetSupplier, BooleanSupplier poseReady,
			BooleanSupplier forceFeed, BooleanSupplier calibrationMode, DoubleSupplier calibrationHoodRotations,
			DoubleSupplier calibrationFlywheelRps, Consumer<Boolean> readyFeedback, boolean finishAfterFeed) {
		ShotCoordinator coordinator = new ShotCoordinator(drive, hood, shooter, bottomIndexer, topIndexer, conveyor,
				targetSupplier, poseReady, forceFeed, calibrationMode, calibrationHoodRotations, calibrationFlywheelRps,
				readyFeedback, finishAfterFeed);

		Command mechanisms = Commands.run(coordinator::execute, hood, shooter, bottomIndexer, topIndexer, conveyor)
				.beforeStarting(coordinator::initialize).finallyDo(interrupted -> coordinator.end());
		if (finishAfterFeed) {
			mechanisms = mechanisms.until(coordinator::isAutoComplete)
					.withTimeout(ShooterConstants.AUTO_TOTAL_TIMEOUT_SECONDS);
		}

		Command aim = DriveCommands.joystickDriveAtAngle(drive, xSupplier, ySupplier, coordinator::targetHeading,
				DrivetrainConstants.MAX_SPEED);
		return Commands.deadline(mechanisms, aim).withName(finishAfterFeed ? "AutonomousHubShot" : "CoordinatedShot");
	}

	private static ShotTarget currentAllianceHubTarget() {
		return ShotTargetSelector.hub(DriverStation.getAlliance().orElse(Alliance.Red));
	}

	/** Pure readiness evaluation used by the coordinated command and unit tests. */
	static ShotReadiness evaluateReadiness(boolean poseReady, ShotSolution solution, ChassisSpeeds speeds,
			Rotation2d actualHeading, double hoodPosition, double flywheelVelocity) {
		if (!poseReady) {
			return ShotReadiness.POSE_NOT_READY;
		}
		if (!solution.calibrated()) {
			return ShotReadiness.OUT_OF_RANGE;
		}
		if (Math.hypot(speeds.vxMetersPerSecond,
				speeds.vyMetersPerSecond) > ShooterConstants.MAX_SHOOTING_TRANSLATION_METERS_PER_SECOND
				|| Math.abs(speeds.omegaRadiansPerSecond) > ShooterConstants.MAX_SHOOTING_ROTATION_RADIANS_PER_SECOND) {
			return ShotReadiness.MOVING_TOO_FAST;
		}
		if (Math.abs(actualHeading.minus(solution.targetHeading())
				.getDegrees()) > ShooterConstants.HEADING_READY_TOLERANCE_DEGREES) {
			return ShotReadiness.AIMING;
		}
		if (Math.abs(hoodPosition - solution.hoodRotations()) > ShooterConstants.HOOD_READY_TOLERANCE_ROTATIONS) {
			return ShotReadiness.HOOD_MOVING;
		}
		if (Math.abs(flywheelVelocity
				- solution.flywheelRotationsPerSecond()) > ShooterConstants.FLYWHEEL_READY_TOLERANCE_RPS) {
			return ShotReadiness.FLYWHEEL_SPINNING_UP;
		}
		return ShotReadiness.READY;
	}

	static double clampCalibrationHoodRotations(double requestedRotations) {
		return Double.isFinite(requestedRotations)
				? MathUtil.clamp(requestedRotations, ShooterConstants.HOOD_GAINS.kMinPosition(),
						ShooterConstants.HOOD_GAINS.kMaxPosition())
				: ShooterConstants.HOOD_GAINS.kMinPosition();
	}

	static double clampCalibrationFlywheelRps(double requestedRps) {
		return Double.isFinite(requestedRps)
				? MathUtil.clamp(requestedRps, 0.0, ShooterConstants.MAX_CALIBRATION_FLYWHEEL_RPS)
				: 0.0;
	}

	private static final class ShotCoordinator {
		private final CommandSwerveDrivetrain drive;
		private final PositionJoint hood;
		private final Flywheel shooter;
		private final Flywheel bottomIndexer;
		private final Flywheel topIndexer;
		private final Flywheel conveyor;
		private final Supplier<ShotTarget> targetSupplier;
		private final BooleanSupplier poseReady;
		private final BooleanSupplier forceFeed;
		private final BooleanSupplier calibrationMode;
		private final DoubleSupplier calibrationHoodRotations;
		private final DoubleSupplier calibrationFlywheelRps;
		private final Consumer<Boolean> readyFeedback;
		private final boolean finishAfterFeed;
		private final Debouncer readinessDebouncer = new Debouncer(ShooterConstants.READY_DEBOUNCE_SECONDS);
		private final Timer totalTimer = new Timer();
		private final Timer feedTimer = new Timer();

		private ShotTarget target;
		private ShotSolution solution;
		private ShotReadiness readiness = ShotReadiness.POSE_NOT_READY;
		private boolean feeding;
		private boolean feedStarted;
		private boolean calibrationActive;

		ShotCoordinator(CommandSwerveDrivetrain drive, PositionJoint hood, Flywheel shooter, Flywheel bottomIndexer,
				Flywheel topIndexer, Flywheel conveyor, Supplier<ShotTarget> targetSupplier, BooleanSupplier poseReady,
				BooleanSupplier forceFeed, BooleanSupplier calibrationMode, DoubleSupplier calibrationHoodRotations,
				DoubleSupplier calibrationFlywheelRps, Consumer<Boolean> readyFeedback, boolean finishAfterFeed) {
			this.drive = drive;
			this.hood = hood;
			this.shooter = shooter;
			this.bottomIndexer = bottomIndexer;
			this.topIndexer = topIndexer;
			this.conveyor = conveyor;
			this.targetSupplier = targetSupplier;
			this.poseReady = poseReady;
			this.forceFeed = forceFeed;
			this.calibrationMode = calibrationMode;
			this.calibrationHoodRotations = calibrationHoodRotations;
			this.calibrationFlywheelRps = calibrationFlywheelRps;
			this.readyFeedback = readyFeedback;
			this.finishAfterFeed = finishAfterFeed;
		}

		void initialize() {
			target = targetSupplier.get();
			if (target == null) {
				target = currentAllianceHubTarget();
			}
			solution = ShooterCalculator.calculate(drive.getState().Pose, target);
			readiness = ShotReadiness.POSE_NOT_READY;
			feeding = false;
			feedStarted = false;
			calibrationActive = false;
			readinessDebouncer.calculate(false);
			totalTimer.restart();
			feedTimer.stop();
			feedTimer.reset();
			readyFeedback.accept(false);
		}

		void execute() {
			boolean ferrySideChanged = false;
			if (target.mode() == ShotTarget.Mode.FERRY) {
				ShotTarget poseSelectedTarget = ShotTargetSelector
						.ferry(DriverStation.getAlliance().orElse(Alliance.Red), drive.getState().Pose);
				ferrySideChanged = !poseSelectedTarget.position().equals(target.position());
				target = poseSelectedTarget;
			}
			solution = ShooterCalculator.calculate(drive.getState().Pose, target);
			calibrationActive = calibrationMode.getAsBoolean();
			if (calibrationActive) {
				double hoodRotations = clampCalibrationHoodRotations(calibrationHoodRotations.getAsDouble());
				double flywheelRps = clampCalibrationFlywheelRps(calibrationFlywheelRps.getAsDouble());
				solution = new ShotSolution(solution.distanceMeters(), solution.targetHeading(), hoodRotations,
						flywheelRps, solution.calibrated());
			}
			hood.setPosition(solution.hoodRotations());
			shooter.setVelocity(solution.flywheelRotationsPerSecond());

			ShotReadiness rawReadiness = determineReadiness();
			boolean forced = forceFeed.getAsBoolean();
			boolean ready = forced || readinessDebouncer
					.calculate(!ferrySideChanged && !calibrationActive && rawReadiness == ShotReadiness.READY);
			readiness = forced
					? ShotReadiness.FORCE_FEED_OVERRIDE
					: calibrationActive ? ShotReadiness.CALIBRATION_MODE : rawReadiness;
			setFeeding(ready);

			if (feeding && launchedFuelSimulation != null) {
				launchedFuelSimulation.tryLaunch();
			}

			Logger.recordOutput("Shooter/DistanceToTargetMeters", solution.distanceMeters());
			Logger.recordOutput("Shooter/DistanceToHubMeters", solution.distanceMeters());
			Logger.recordOutput("Shooter/ShotTargetMode", target.mode().toString());
			Logger.recordOutput("Shooter/ShotTargetX", target.position().getX());
			Logger.recordOutput("Shooter/ShotTargetY", target.position().getY());
			Logger.recordOutput("Shooter/SolutionCalibrated", solution.calibrated());
			Logger.recordOutput("Shooter/Readiness", readiness.toString());
			Logger.recordOutput("Shooter/ReadyToFeed", ready);
			Logger.recordOutput("Shooter/Feeding", feeding);
			Logger.recordOutput("Shooter/CalibrationMode", calibrationActive);
			Logger.recordOutput("Shooter/CalibrationAppliedHoodRotations", solution.hoodRotations());
			Logger.recordOutput("Shooter/CalibrationAppliedFlywheelRPS", solution.flywheelRotationsPerSecond());
			Logger.recordOutput("Shooter/CalibrationMeasuredHoodRotations", hood.getPosition());
			Logger.recordOutput("Shooter/CalibrationMeasuredFlywheelRPS", shooter.getVelocity());
			Logger.recordOutput("Shooter/CalibrationDistanceInches",
					edu.wpi.first.math.util.Units.metersToInches(solution.distanceMeters()));
			SmartDashboard.putString("Shooter/Readiness", readiness.toString());
			SmartDashboard.putString("Shooter/TargetMode", target.mode().toString());
			SmartDashboard.putBoolean("Shooter/Ready", ready);
			SmartDashboard.putBoolean("Shooter/CalibrationMode", calibrationActive);
		}

		private ShotReadiness determineReadiness() {
			ShotReadiness evaluated = evaluateReadiness(poseReady.getAsBoolean(), solution, drive.getPhysicsSpeeds(),
					drive.getState().Pose.getRotation(), hood.getPosition(), shooter.getVelocity());
			return target.mode() == ShotTarget.Mode.FERRY && evaluated == ShotReadiness.OUT_OF_RANGE
					? ShotReadiness.UNSAFE_FERRY_PATH
					: evaluated;
		}

		private void setFeeding(boolean shouldFeed) {
			feeding = shouldFeed;
			readyFeedback.accept(shouldFeed);
			if (shouldFeed) {
				bottomIndexer.setVoltage(INDEXER_PRESET.FEED_BOTTOM.get());
				topIndexer.setVoltage(INDEXER_PRESET.FEED_TOP.get());
				conveyor.setVoltage(CONVEYOR_PRESET.FEED.get());
				feedStarted = true;
				if (finishAfterFeed) {
					feedTimer.start();
				}
			} else {
				bottomIndexer.stop();
				topIndexer.stop();
				conveyor.stop();
				feedTimer.stop();
			}
		}

		boolean isAutoComplete() {
			return feedTimer.hasElapsed(ShooterConstants.AUTO_FEED_SECONDS)
					|| (!feedStarted && totalTimer.hasElapsed(ShooterConstants.AUTO_READY_TIMEOUT_SECONDS));
		}

		Rotation2d targetHeading() {
			return solution != null
					? solution.targetHeading()
					: ShooterCalculator.calculate(drive.getState().Pose, currentAllianceHubTarget()).targetHeading();
		}

		void end() {
			setFeeding(false);
			shooter.stop();
			hood.setComplianceAfterTarget(false);
			hood.setPosition(HOOD_PRESET.STOW.get());
			totalTimer.stop();
			readyFeedback.accept(false);
			Logger.recordOutput("Shooter/Feeding", false);
			Logger.recordOutput("Shooter/CalibrationMode", false);
			SmartDashboard.putBoolean("Shooter/Ready", false);
			SmartDashboard.putBoolean("Shooter/CalibrationMode", false);
		}
	}
}
