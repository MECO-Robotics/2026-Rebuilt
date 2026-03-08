package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Mode;
import frc.robot.constants.drive.AzimuthMotorConstants.AzimuthMotorGains;
import frc.robot.constants.drive.AzimuthMotorConstants.AzimuthMotorHardwareConfig;
import frc.robot.constants.drive.DriveConstants;
import frc.robot.constants.drive.DriveMotorConstants.DriveMotorGains;
import frc.robot.constants.drive.DriveMotorConstants.DriveMotorHardwareConfig;
import frc.robot.subsystems.drive.azimuth_motor.AzimuthMotorIO;
import frc.robot.subsystems.drive.drive_motor.DriveMotorIO;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIOInputsAutoLogged;
import frc.robot.subsystems.drive.gyro.GyroIOSim;
import frc.robot.subsystems.drive.module.Module;
import frc.robot.subsystems.drive.odometry_threads.PhoenixOdometryThread;
import frc.robot.subsystems.drive.odometry_threads.SparkOdometryThread;
import frc.robot.util.mechanical_advantage.LoggedTunableNumber;
import frc.robot.util.mechanical_advantage.swerve.ModuleLimits;
import frc.robot.util.mechanical_advantage.swerve.SwerveSetpoint;
import frc.robot.util.mechanical_advantage.swerve.SwerveSetpointGenerator;
import frc.robot.util.pathplanner.AdvancedPPHolonomicDriveController;
import frc.robot.util.pathplanner.LocalADStarAK;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BiFunction;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Main swerve drive subsystem coordinating modules, odometry, and auto
 * integration.
 */
public class Drive extends SubsystemBase {
	public static final Lock odometryLock = new ReentrantLock();
	private final GyroIO gyroIO;
	private SwerveDriveSimulation swerveDriveSimulation;
	private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
	private final Module[] modules = new Module[4]; // FL, FR, BL, BR
	private final Alert gyroDisconnectedAlert = new Alert("Drive", "Disconnected gyro, using kinematics as fallback.",
			AlertType.kError);
	private final Alert gyroSampleMismatchAlert = new Alert("Drive",
			"Gyro odometry sample mismatch, using kinematics fallback.", AlertType.kWarning);

	private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
	private Rotation2d rawGyroRotation = new Rotation2d();
	private SwerveModulePosition[] lastModulePositions = // For delta tracking
			new SwerveModulePosition[]{new SwerveModulePosition(), new SwerveModulePosition(),
					new SwerveModulePosition(), new SwerveModulePosition()};
	private final SwerveModulePosition[] modulePositionsBuffer = new SwerveModulePosition[4];
	private final SwerveModulePosition[] moduleDeltasBuffer = new SwerveModulePosition[4];
	private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation,
			lastModulePositions, new Pose2d());

	private final SwerveSetpointGenerator setpointGenerator;
	private ModuleLimits currentModuleLimits = DriveConstants.defaultModuleLimits;
	private SwerveSetpoint currentSetpoint = new SwerveSetpoint(new ChassisSpeeds(), new SwerveModuleState[]{
			new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()},
			new double[4]);

	private final LoggedTunableNumber kMaxDriveVelocity;
	private final LoggedTunableNumber kMaxDriveAcceleration;
	private final LoggedTunableNumber kMaxDriveDeceleration;
	private final LoggedTunableNumber kMaxSteeringVelocity;

	private final LoggedTunableNumber drivekP;
	private final LoggedTunableNumber drivekI;
	private final LoggedTunableNumber drivekD;
	private final LoggedTunableNumber drivekS;
	private final LoggedTunableNumber drivekV;
	private final LoggedTunableNumber drivekA;

	private final LoggedTunableNumber azimuthkP;
	private final LoggedTunableNumber azimuthkI;
	private final LoggedTunableNumber azimuthkD;
	private final LoggedTunableNumber azimuthkS;
	private final LoggedTunableNumber azimuthkV;
	private final LoggedTunableNumber azimuthkA;

	private final LoggedTunableNumber translationkP;
	private final LoggedTunableNumber translationkI;
	private final LoggedTunableNumber translationkD;
	private final LoggedTunableNumber rotationkP;
	private final LoggedTunableNumber rotationkI;
	private final LoggedTunableNumber rotationkD;
	private final LoggedTunableNumber useAzimuthVelocityFF;

	/**
	 * Creates a drive subsystem from module configs/factory builders.
	 *
	 * <p>
	 * Owns Maple swerve simulation internally when running in sim and routes the
	 * shared simulation state to gyro and module IO.
	 */
	public static Drive fromModuleConfigs(Supplier<GyroIO> realGyroSupplier,
			BiFunction<String, DriveMotorHardwareConfig, Supplier<DriveMotorIO>> driveFactoryBuilder,
			BiFunction<String, AzimuthMotorHardwareConfig, Supplier<AzimuthMotorIO>> azimuthFactoryBuilder,
			DriveMotorHardwareConfig frontLeftDriveConfig, AzimuthMotorHardwareConfig frontLeftAzimuthConfig,
			DriveMotorHardwareConfig frontRightDriveConfig, AzimuthMotorHardwareConfig frontRightAzimuthConfig,
			DriveMotorHardwareConfig backLeftDriveConfig, AzimuthMotorHardwareConfig backLeftAzimuthConfig,
			DriveMotorHardwareConfig backRightDriveConfig, AzimuthMotorHardwareConfig backRightAzimuthConfig,
			DriveMotorGains driveGains, AzimuthMotorGains azimuthGains, PhoenixOdometryThread phoenixOdometryThread,
			SparkOdometryThread sparkOdometryThread) {
		SwerveDriveSimulation sim = null;
		if (Constants.currentMode == Constants.Mode.SIM) {
			sim = new SwerveDriveSimulation(DriveConstants.mapleSimConfig, new Pose2d(3.0, 3.0, new Rotation2d()));
			applyMapleSimContactTuning(sim);
			SimulatedArena.getInstance().addDriveTrainSimulation(sim);
		}
		final SwerveDriveSimulation simDrive = sim;

		GyroIO gyroIO = GyroIO.fromMode(realGyroSupplier,
				simDrive != null ? GyroIO.simFactory(simDrive.getGyroSimulation()) : GyroIOSim::new);

		Module frontLeftModule = Module.fromMode("FrontLeft", frontLeftDriveConfig, frontLeftAzimuthConfig,
				driveFactoryBuilder, azimuthFactoryBuilder, simDrive != null ? simDrive.getModules()[0] : null);
		Module frontRightModule = Module.fromMode("FrontRight", frontRightDriveConfig, frontRightAzimuthConfig,
				driveFactoryBuilder, azimuthFactoryBuilder, simDrive != null ? simDrive.getModules()[1] : null);
		Module backLeftModule = Module.fromMode("BackLeft", backLeftDriveConfig, backLeftAzimuthConfig,
				driveFactoryBuilder, azimuthFactoryBuilder, simDrive != null ? simDrive.getModules()[2] : null);
		Module backRightModule = Module.fromMode("BackRight", backRightDriveConfig, backRightAzimuthConfig,
				driveFactoryBuilder, azimuthFactoryBuilder, simDrive != null ? simDrive.getModules()[3] : null);

		Drive drive = new Drive(gyroIO, frontLeftModule, frontRightModule, backLeftModule, backRightModule, driveGains,
				azimuthGains, phoenixOdometryThread, sparkOdometryThread);
		drive.swerveDriveSimulation = simDrive;
		if (simDrive != null) {
			drive.setPose(simDrive.getSimulatedDriveTrainPose());
		}
		return drive;
	}

	/**
	 * Creates a four-module swerve drive subsystem.
	 *
	 * @param gyroIO
	 *            gyro abstraction
	 * @param flModuleIO
	 *            front-left module wrapper
	 * @param frModuleIO
	 *            front-right module wrapper
	 * @param blModuleIO
	 *            back-left module wrapper
	 * @param brModuleIO
	 *            back-right module wrapper
	 * @param driveGains
	 *            initial drive motor gains
	 * @param azimuthGains
	 *            initial azimuth motor gains
	 * @param phoenixOdometryThread
	 *            optional Phoenix odometry thread
	 * @param sparkOdometryThread
	 *            optional Spark odometry thread
	 */
	public Drive(GyroIO gyroIO, Module flModuleIO, Module frModuleIO, Module blModuleIO, Module brModuleIO,
			DriveMotorGains driveGains, AzimuthMotorGains azimuthGains, PhoenixOdometryThread phoenixOdometryThread,
			SparkOdometryThread sparkOdometryThread) {
		this.swerveDriveSimulation = null;
		this.gyroIO = gyroIO;
		modules[0] = flModuleIO;
		modules[1] = frModuleIO;
		modules[2] = blModuleIO;
		modules[3] = brModuleIO;
		for (int i = 0; i < 4; i++) {
			modulePositionsBuffer[i] = new SwerveModulePosition();
			moduleDeltasBuffer[i] = new SwerveModulePosition();
		}
		setpointGenerator = new SwerveSetpointGenerator(kinematics, DriveConstants.moduleTranslations[0],
				DriveConstants.moduleTranslations[1], DriveConstants.moduleTranslations[2],
				DriveConstants.moduleTranslations[3]);

		drivekP = new LoggedTunableNumber("Drive/DriveMotors/Gains/kP", driveGains.kP());
		drivekI = new LoggedTunableNumber("Drive/DriveMotors/Gains/kI", driveGains.kI());
		drivekD = new LoggedTunableNumber("Drive/DriveMotors/Gains/kD", driveGains.kD());
		drivekS = new LoggedTunableNumber("Drive/DriveMotors/Gains/kS", driveGains.kS());
		drivekV = new LoggedTunableNumber("Drive/DriveMotors/Gains/kV", driveGains.kV());
		drivekA = new LoggedTunableNumber("Drive/DriveMotors/Gains/kA", driveGains.kA());

		azimuthkP = new LoggedTunableNumber("Drive/AzimuthMotors/Gains/kP", azimuthGains.kP());
		azimuthkI = new LoggedTunableNumber("Drive/AzimuthMotors/Gains/kI", azimuthGains.kI());
		azimuthkD = new LoggedTunableNumber("Drive/AzimuthMotors/Gains/kD", azimuthGains.kD());
		azimuthkS = new LoggedTunableNumber("Drive/AzimuthMotors/Gains/kS", azimuthGains.kS());
		azimuthkV = new LoggedTunableNumber("Drive/AzimuthMotors/Gains/kV", azimuthGains.kV());
		azimuthkA = new LoggedTunableNumber("Drive/AzimuthMotors/Gains/kA", azimuthGains.kA());

		translationkP = new LoggedTunableNumber("Pathplanner/TranslationalP", DriveConstants.translationPID.kP);
		translationkI = new LoggedTunableNumber("Pathplanner/TranslationalI", DriveConstants.translationPID.kI);
		translationkD = new LoggedTunableNumber("Pathplanner/TranslationalD", DriveConstants.translationPID.kD);
		rotationkP = new LoggedTunableNumber("Pathplanner/RotationalP", DriveConstants.rotationPID.kP);
		rotationkI = new LoggedTunableNumber("Pathplanner/RotationalI", DriveConstants.rotationPID.kI);
		rotationkD = new LoggedTunableNumber("Pathplanner/RotationalD", DriveConstants.rotationPID.kD);
		useAzimuthVelocityFF = new LoggedTunableNumber("Drive/UseAzimuthVelocityFF", 0.0);

		// Load the configured gains immediately so sim IO PID/FF are initialized at
		// startup.
		for (int i = 0; i < 4; i++) {
			modules[i].setGains(driveGains, azimuthGains);
		}

		kMaxDriveVelocity = new LoggedTunableNumber("Drive/ModuleLimits/kMaxDriveVelocityMetersPerSec",
				currentModuleLimits.maxDriveVelocity());
		kMaxDriveAcceleration = new LoggedTunableNumber("Drive/ModuleLimits/kMaxDriveAccelerationMetersPerSecSq",
				currentModuleLimits.maxDriveAcceleration());
		kMaxDriveDeceleration = new LoggedTunableNumber("Drive/ModuleLimits/kMaxDriveDecelerationMetersPerSecSq",
				currentModuleLimits.maxDriveDeceleration());
		kMaxSteeringVelocity = new LoggedTunableNumber("Drive/ModuleLimits/kMaxSteeringVelocityRadPerSec",
				currentModuleLimits.maxSteeringVelocity());

		// Usage reporting for swerve template
		HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

		// Start phoenix odometry thread
		if (phoenixOdometryThread != null) {
			phoenixOdometryThread.start();
		}

		// Start spark odometry thread
		if (sparkOdometryThread != null) {
			sparkOdometryThread.start();
		}

		// Configure AutoBuilder for PathPlanner
		AutoBuilder.configure(this::getPose, this::setPose, this::getChassisSpeeds, this::runVelocity,
				new AdvancedPPHolonomicDriveController(DriveConstants.translationPID, DriveConstants.rotationPID, 0.02,
						translationkP, translationkI, translationkD, rotationkP, rotationkI, rotationkD),
				DriveConstants.ppConfig, () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red, this);
		Pathfinding.setPathfinder(new LocalADStarAK());
		PathPlannerLogging.setLogActivePathCallback((activePath) -> {
			Logger.recordOutput("Drive/Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
		});
		PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
			Logger.recordOutput("Drive/Odometry/TrajectorySetpoint", targetPose);
		});
	}

	@Override
	public void periodic() {
		odometryLock.lock(); // Prevents odometry updates while reading data
		gyroIO.updateInputs(gyroInputs);
		Logger.processInputs("Drive/Gyro", gyroInputs);
		for (var module : modules) {
			module.periodic();
		}
		odometryLock.unlock();

		// Stop moving when disabled
		if (DriverStation.isDisabled()) {
			for (var module : modules) {
				module.stop();
			}
			// Keep generator history aligned to real module angles/speeds between enable
			// cycles.
			currentSetpoint = new SwerveSetpoint(getChassisSpeeds(), getModuleStates(), new double[4]);
		}

		// Log empty setpoint states when disabled
		if (DriverStation.isDisabled()) {
			Logger.recordOutput("Drive/SwerveStates/Setpoints", new SwerveModuleState[]{});
			Logger.recordOutput("Drive/SwerveStates/SetpointsOptimized", new SwerveModuleState[]{});
		}

		// Update odometry
		double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
		int sampleCount = sampleTimestamps.length;
		boolean hasGyroSampleMismatch = false;
		SwerveModulePosition[] module0Positions = modules[0].getOdometryPositions();
		SwerveModulePosition[] module1Positions = modules[1].getOdometryPositions();
		SwerveModulePosition[] module2Positions = modules[2].getOdometryPositions();
		SwerveModulePosition[] module3Positions = modules[3].getOdometryPositions();
		for (int i = 0; i < sampleCount; i++) {
			// Read wheel positions and deltas from each module
			modulePositionsBuffer[0] = module0Positions[i];
			modulePositionsBuffer[1] = module1Positions[i];
			modulePositionsBuffer[2] = module2Positions[i];
			modulePositionsBuffer[3] = module3Positions[i];
			for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
				moduleDeltasBuffer[moduleIndex] = new SwerveModulePosition(
						modulePositionsBuffer[moduleIndex].distanceMeters
								- lastModulePositions[moduleIndex].distanceMeters,
						modulePositionsBuffer[moduleIndex].angle);
				lastModulePositions[moduleIndex] = modulePositionsBuffer[moduleIndex];
			}

			// Update gyro angle
			if (gyroInputs.connected && i < gyroInputs.odometryYawPositions.length) {
				// Use the real gyro angle
				rawGyroRotation = gyroInputs.odometryYawPositions[i];
			} else {
				hasGyroSampleMismatch |= gyroInputs.connected;
				// Use the angle delta from the kinematics and module deltas
				Twist2d twist = kinematics.toTwist2d(moduleDeltasBuffer);
				rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
			}

			// Apply update
			poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositionsBuffer);
		}

		// Update gyro alert
		gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
		gyroSampleMismatchAlert.set(hasGyroSampleMismatch && Constants.currentMode != Mode.SIM);

		LoggedTunableNumber.ifChanged(hashCode(), (values) -> {
			for (int i = 0; i < 4; i++) {
				modules[i].setGains(
						new DriveMotorGains(values[0], values[1], values[2], values[3], values[4], values[5]),
						new AzimuthMotorGains(values[6], values[7], values[8], values[9], values[10], values[11]));
			}
		}, drivekP, drivekI, drivekD, drivekS, drivekV, drivekA, azimuthkP, azimuthkI, azimuthkD, azimuthkS, azimuthkV,
				azimuthkA);

		LoggedTunableNumber.ifChanged(hashCode(), (values) -> {
			currentModuleLimits = new ModuleLimits(values[0], values[1], values[2], values[3]);
		}, kMaxDriveVelocity, kMaxDriveAcceleration, kMaxDriveDeceleration, kMaxSteeringVelocity);

		if (Constants.currentMode == Mode.SIM && swerveDriveSimulation != null) {
			Logger.recordOutput("Drive/Simulation/MaplePose", swerveDriveSimulation.getSimulatedDriveTrainPose());
		}

	}

	/**
	 * Runs the drive at the desired velocity.
	 *
	 * @param speeds
	 *            Speeds in meters/sec
	 */
	public void runVelocity(ChassisSpeeds speeds) {
		// Discretize to improve tracking of simultaneous translation + rotation
		// commands.
		ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
		currentSetpoint = setpointGenerator.generateSetpoint(currentModuleLimits, currentSetpoint, discreteSpeeds,
				new Translation2d(), 0.02);
		Logger.recordOutput("Drive/SwerveStates/Setpoints", currentSetpoint.moduleStates());
		Logger.recordOutput("Drive/SwerveChassisSpeeds/Setpoints", discreteSpeeds);
		Logger.recordOutput("Drive/SwerveStates/AzimuthVelocityFF", currentSetpoint.azimuthVelocityFF());

		// Send setpoints to modules
		double azimuthFFScale = useAzimuthVelocityFF.get();
		for (int i = 0; i < 4; i++) {
			modules[i].runSetpoint(currentSetpoint.moduleStates()[i],
					currentSetpoint.azimuthVelocityFF()[i] * azimuthFFScale);
		}

		Logger.recordOutput("Drive/SwerveStates/SetpointsOptimized", currentSetpoint.moduleStates());
	}

	/** Runs the drive in a straight line with the specified drive output. */
	public void runCharacterization(double output) {
		for (int i = 0; i < 4; i++) {
			modules[i].runDriveCharacterization(output);
		}
	}

	/** Runs azimuth characterization with zero drive motor output. */
	public void runAzimuthCharacterization(double output) {
		for (int i = 0; i < 4; i++) {
			modules[i].runAzimuthCharacterization(output);
		}
	}

	/** Stops the drive. */
	public void stop() {
		runVelocity(new ChassisSpeeds());
	}

	/**
	 * Stops the drive and turns the modules to an X arrangement to resist movement.
	 * The modules will return to their normal orientations the next time a nonzero
	 * velocity is requested.
	 */
	public void stopWithX() {
		Rotation2d[] headings = new Rotation2d[4];
		for (int i = 0; i < 4; i++) {
			headings[i] = getModuleTranslations()[i].getAngle();
		}
		kinematics.resetHeadings(headings);
		stop();
	}

	/**
	 * Returns the module states (turn angles and drive velocities) for all of the
	 * modules.
	 */
	@AutoLogOutput(key = "Drive/SwerveStates/Measured")
	private SwerveModuleState[] getModuleStates() {
		SwerveModuleState[] states = new SwerveModuleState[4];
		for (int i = 0; i < 4; i++) {
			states[i] = modules[i].getState();
		}
		return states;
	}

	/**
	 * Returns the module positions (turn angles and drive positions) for all of the
	 * modules.
	 */
	private SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] states = new SwerveModulePosition[4];
		for (int i = 0; i < 4; i++) {
			states[i] = modules[i].getPosition();
		}
		return states;
	}

	/** Returns the measured chassis speeds of the robot. */
	@AutoLogOutput(key = "Drive/SwerveChassisSpeeds/Measured")
	public ChassisSpeeds getChassisSpeeds() {
		return kinematics.toChassisSpeeds(getModuleStates());
	}

	/** Returns the position of each module in radians. */
	public double[] getWheelRadiusCharacterizationPositions() {
		double[] values = new double[4];
		for (int i = 0; i < 4; i++) {
			values[i] = modules[i].getWheelRadiusCharacterizationPosition();
		}
		return values;
	}

	/**
	 * Returns the average velocity of the modules in rotations/sec (Phoenix native
	 * units).
	 */
	public double getFFCharacterizationVelocity() {
		double output = 0.0;
		for (int i = 0; i < 4; i++) {
			output += modules[i].getFFCharacterizationVelocity() / 4.0;
		}
		return output;
	}

	/** Returns the current odometry pose. */
	@AutoLogOutput(key = "Drive/Odometry/Robot")
	public Pose2d getPose() {
		return poseEstimator.getEstimatedPosition();
	}

	/** Returns the current odometry rotation. */
	public Rotation2d getRotation() {
		return getPose().getRotation();
	}

	/** Resets the current odometry pose. */
	public void setPose(Pose2d pose) {
		poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
	}

	/** Adds a new timestamped vision measurement. */
	public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds,
			Matrix<N3, N1> visionMeasurementStdDevs) {
		poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
	}

	/** Returns the maximum linear speed in meters per sec. */
	public double getMaxLinearSpeedMetersPerSec() {
		return DriveConstants.maxSpeedAt12Volts.in(MetersPerSecond);
	}

	/** Returns the maximum angular speed in radians per sec. */
	public double getMaxAngularSpeedRadPerSec() {
		return getMaxLinearSpeedMetersPerSec() / DriveConstants.driveBaseRadius;
	}

	/** Returns an array of module translations. */
	public static Translation2d[] getModuleTranslations() {
		return DriveConstants.moduleTranslations;
	}

	/** Get the drivetrain simulation object. */
	public SwerveDriveSimulation getSimulation() {
		return swerveDriveSimulation;
	}

	private static void applyMapleSimContactTuning(SwerveDriveSimulation sim) {
		for (var fixture : sim.getFixtures()) {
			fixture.setFriction(DriveConstants.mapleSimBumperFriction);
			fixture.setRestitution(DriveConstants.mapleSimBumperRestitution);
		}
	}
}
