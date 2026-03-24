package frc.robot.simulation;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.simulation.MapleSimConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.position_joint.PositionJoint;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

/**
 * Launches held fuel from the intake simulation into Maple's projectile
 * simulation.
 */
public class LaunchedFuelSim {
	private final CommandSwerveDrivetrain drive;
	private final IntakeSim intakeSim;
	private final PositionJoint hood;
	private final Flywheel shooterFlywheel;
	private int successfulScoreCount = 0;

	private double lastBurstTimestampSeconds = Double.NEGATIVE_INFINITY;
	private double nextBurstShotTimestampSeconds = Double.NEGATIVE_INFINITY;
	private int pendingBurstShots = 0;
	private int burstShotIndex = 0;

	public LaunchedFuelSim(CommandSwerveDrivetrain drive, IntakeSim intakeSim, PositionJoint hood,
			Flywheel shooterFlywheel) {
		this.drive = drive;
		this.intakeSim = intakeSim;
		this.hood = hood;
		this.shooterFlywheel = shooterFlywheel;
	}

	/** Attempts to launch a staggered burst of fuel from robot storage. */
	public void tryLaunch() {
		double nowSeconds = Timer.getFPGATimestamp();
		boolean inSim = Constants.currentMode == Constants.Mode.SIM && drive.getSimulation() != null;
		boolean hasVelocity = hasLaunchVelocity();
		boolean cooldownReady = pastCooldown(nowSeconds);

		Logger.recordOutput("FieldSimulation/LaunchDebug/InSim", inSim);
		Logger.recordOutput("FieldSimulation/LaunchDebug/HasLaunchVelocity", hasVelocity);
		Logger.recordOutput("FieldSimulation/LaunchDebug/CooldownReady", cooldownReady);
		Logger.recordOutput("FieldSimulation/LaunchDebug/StoredFuelCount", intakeSim.getStoredFuelCount());
		Logger.recordOutput("FieldSimulation/LaunchDebug/PendingBurstShots", pendingBurstShots);

		if (!inSim || !hasVelocity) {
			return;
		}

		if (pendingBurstShots == 0 && cooldownReady) {
			beginBurst(nowSeconds);
		}

		if (pendingBurstShots == 0 || nowSeconds < nextBurstShotTimestampSeconds) {
			return;
		}

		double projectileSpeedMps = getProjectileSpeedMps();
		int launchedFuelCount = 0;
		while (pendingBurstShots > 0 && nowSeconds >= nextBurstShotTimestampSeconds) {
			if (!intakeSim.launchFuel()) {
				pendingBurstShots = 0;
				break;
			}

			launchSingleFuel(burstShotIndex, projectileSpeedMps);
			launchedFuelCount++;
			burstShotIndex++;
			pendingBurstShots--;

			if (pendingBurstShots > 0) {
				nextBurstShotTimestampSeconds += getRandomStaggerDelaySeconds();
			}
		}

		Logger.recordOutput("FieldSimulation/LaunchDebug/LaunchedFuelCount", launchedFuelCount);
	}

	public Command launchCommand() {
		return Commands.run(this::tryLaunch);
	}

	public void resetSuccessfulScoreCount() {
		successfulScoreCount = 0;
		Logger.recordOutput("FieldSimulation/SuccessfulScoreCount", successfulScoreCount);
	}

	public int getSuccessfulScoreCount() {
		return successfulScoreCount;
	}

	private boolean hasLaunchVelocity() {
		return Math.abs(shooterFlywheel.getVelocity()) >= MapleSimConstants.MIN_FLYWHEEL_RPS_FOR_SHOT;
	}

	private boolean pastCooldown(double nowSeconds) {
		return nowSeconds - lastBurstTimestampSeconds >= MapleSimConstants.SHOT_COOLDOWN_SECONDS;
	}

	private void beginBurst(double nowSeconds) {
		pendingBurstShots = MapleSimConstants.FUEL_PER_SHOT;
		burstShotIndex = 0;
		nextBurstShotTimestampSeconds = nowSeconds;
		lastBurstTimestampSeconds = nowSeconds;
	}

	private double getRandomStaggerDelaySeconds() {
		double averageDelay = MapleSimConstants.SHOT_COOLDOWN_SECONDS / MapleSimConstants.FUEL_PER_SHOT;
		double jitterAmplitude = averageDelay * MapleSimConstants.SHOT_STAGGER_RANDOMNESS_RATIO;
		double randomizedDelay = averageDelay + (Math.random() * 2.0 - 1.0) * jitterAmplitude;
		return Math.max(MapleSimConstants.MIN_SHOT_STAGGER_SECONDS, randomizedDelay);
	}

	private void launchSingleFuel(int burstIndex, double projectileSpeedMps) {
		double randomizedAngleRadians = getRandomizedLaunchAngleRadians();
		double randomizedSpeedMps = getRandomizedProjectileSpeedMps(projectileSpeedMps);
		Pose3d[] emptyTrajectory = new Pose3d[0];

		SimulatedArena.getInstance()
				.addGamePieceProjectile(new RebuiltFuelOnFly(drive.getPhysicsPose().getTranslation(),
						getShooterTranslationForBurstIndex(burstIndex), drive.getPhysicsSpeeds(),
						drive.getPhysicsPose().getRotation().plus(MapleSimConstants.SHOOTER_YAW_OFFSET),
						Meters.of(MapleSimConstants.SHOOTER_HEIGHT_METERS), MetersPerSecond.of(randomizedSpeedMps),
						Radians.of(Math.PI / 2.0 - randomizedAngleRadians))
								.withTargetPosition(() -> new Translation3d(FieldConstants.Hub.hubPosition().getX(),
										FieldConstants.Hub.hubPosition().getY(), FieldConstants.Hub.hubHeight))
								.withTargetTolerance(
										new Translation3d(Units.feetToMeters(2), Units.feetToMeters(2), 0.1))
								.withHitTargetCallBack(() -> {
									successfulScoreCount++;
									Logger.recordOutput("FieldSimulation/SuccessfulScoreCount", successfulScoreCount);
									SimulatedArena.getInstance().addGamePiece(createHubBackSpawnFuel());
								})
								.withProjectileTrajectoryDisplayCallBack(
										pose3ds -> Logger.recordOutput("Flywheel/FuelProjectileSuccessfulShot",
												pose3ds.toArray(Pose3d[]::new)),
										pose3ds -> Logger.recordOutput("Flywheel/FuelProjectileUnsuccessfulShot",
												pose3ds.toArray(Pose3d[]::new)))
								.withProjectileTrajectoryDisplayCallBack(
										pose3ds -> Logger.recordOutput("Flywheel/FuelProjectileSuccessfulShot",
												pose3ds.toArray(Pose3d[]::new)),
										pose3ds -> Logger.recordOutput("Flywheel/FuelProjectileUnsuccessfulShot",
												pose3ds.toArray(Pose3d[]::new))));

		Logger.recordOutput("Flywheel/FuelProjectileSuccessfulShot", emptyTrajectory);
		Logger.recordOutput("Flywheel/FuelProjectileUnsuccessfulShot", emptyTrajectory);
	}

	private double getRandomizedLaunchAngleRadians() {
		double baseAngleRadians = getLaunchAngleRadians();
		double jitterRadians = (Math.random() * 2.0 - 1.0) * MapleSimConstants.SHOT_ANGLE_RANDOMNESS_RADIANS;
		return MathUtil.clamp(baseAngleRadians + jitterRadians, MapleSimConstants.MIN_LAUNCH_ANGLE_RADIANS,
				MapleSimConstants.MAX_LAUNCH_ANGLE_RADIANS);
	}

	private double getRandomizedProjectileSpeedMps(double baseSpeedMps) {
		double jitterScale = 1.0 + (Math.random() * 2.0 - 1.0) * MapleSimConstants.SHOT_VELOCITY_RANDOMNESS_RATIO;
		return baseSpeedMps * jitterScale;
	}

	private double getLaunchAngleRadians() {
		double hoodAngle = Rotation2d.fromRotations(hood.getPosition()).getRadians()
				+ MapleSimConstants.HOOD_ANGLE_OFFSET_RADIANS;
		return MathUtil.clamp(hoodAngle, MapleSimConstants.MIN_LAUNCH_ANGLE_RADIANS,
				MapleSimConstants.MAX_LAUNCH_ANGLE_RADIANS);
	}

	private double getProjectileSpeedMps() {
		return Math.abs(shooterFlywheel.getVelocity()) * MapleSimConstants.MPS_PER_FLYWHEEL_RPS;
	}

	private Translation2d getShooterTranslationForBurstIndex(int burstIndex) {
		double centeredIndex = burstIndex - (MapleSimConstants.FUEL_PER_SHOT - 1) / 2.0;
		double lateralOffset = centeredIndex * MapleSimConstants.FUEL_BURST_LATERAL_SPACING_METERS;
		return MapleSimConstants.SHOOTER_TRANSLATION_ON_ROBOT.plus(new Translation2d(0.0, lateralOffset));
	}

	private Translation2d getHubBackSpawnPosition() {
		double backDirection = Constants.isAllianceRed() ? -1.0 : 1.0;
		return FieldConstants.Hub.hubPosition()
				.plus(new Translation2d(backDirection * MapleSimConstants.HUB_BACK_SPAWN_X_OFFSET_METERS, 0.0));
	}

	private RebuiltFuelOnField createHubBackSpawnFuel() {
		double backDirection = Constants.isAllianceRed() ? -1.0 : 1.0;
		double baseHeadingRadians = backDirection > 0.0 ? 0.0 : Math.PI;
		double randomHeadingOffsetRadians = (Math.random() * 2.0 - 1.0)
				* Math.toRadians(MapleSimConstants.HUB_BACK_SPAWN_DIRECTION_RNG_DEG);
		double randomizedHeadingRadians = baseHeadingRadians + randomHeadingOffsetRadians;
		double velocityX = MapleSimConstants.HUB_BACK_SPAWN_SPEED_MPS * Math.cos(randomizedHeadingRadians);
		double velocityY = MapleSimConstants.HUB_BACK_SPAWN_SPEED_MPS * Math.sin(randomizedHeadingRadians);

		RebuiltFuelOnField spawnedFuel = new RebuiltFuelOnField(getHubBackSpawnPosition());
		spawnedFuel.setVelocity(new ChassisSpeeds(velocityX, velocityY, 0.0));
		return spawnedFuel;
	}
}
