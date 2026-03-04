package frc.robot.simulation;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

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
import frc.robot.constants.MapleSimConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.position_joint.PositionJoint;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

/** Launches held fuel from the intake simulation into Maple's projectile simulation. */
public class LaunchedFuelSim {
  private final Drive drive;
  private final IntakeSim intakeSim;
  private final PositionJoint hood;
  private final Flywheel shooterFlywheel;

  private double lastLaunchTimestampSeconds = Double.NEGATIVE_INFINITY;

  public LaunchedFuelSim(
      Drive drive, IntakeSim intakeSim, PositionJoint hood, Flywheel shooterFlywheel) {
    this.drive = drive;
    this.intakeSim = intakeSim;
    this.hood = hood;
    this.shooterFlywheel = shooterFlywheel;
  }

  /** Attempts to launch one fuel from robot storage into the simulated arena. */
  public void tryLaunch() {
    boolean inSim = Constants.currentMode == Constants.Mode.SIM && drive.getSimulation() != null;
    boolean hasVelocity = hasLaunchVelocity();
    boolean cooldownReady = pastCooldown();
    int storedFuel = intakeSim.getStoredFuelCount();
    double projectileSpeedMps = getProjectileSpeedMps();

    Logger.recordOutput("FieldSimulation/LaunchDebug/InSim", inSim);
    Logger.recordOutput("FieldSimulation/LaunchDebug/HasLaunchVelocity", hasVelocity);
    Logger.recordOutput("FieldSimulation/LaunchDebug/CooldownReady", cooldownReady);
    Logger.recordOutput("FieldSimulation/LaunchDebug/StoredFuelCount", storedFuel);
    Logger.recordOutput("FieldSimulation/LaunchDebug/ProjectileSpeedMps", projectileSpeedMps);

    if (!inSim) {
      return;
    }

    if (!hasVelocity || !cooldownReady) {
      return;
    }

    if (!intakeSim.launchFuel()) {
      return;
    }

    SimulatedArena.getInstance()
        .addGamePieceProjectile(
            new RebuiltFuelOnFly(
                    drive.getPose().getTranslation(),
                    MapleSimConstants.SHOOTER_TRANSLATION_ON_ROBOT,
                    ChassisSpeeds.fromRobotRelativeSpeeds(
                        drive.getChassisSpeeds(), drive.getRotation()),
                    drive.getRotation().plus(MapleSimConstants.SHOOTER_YAW_OFFSET),
                    Meters.of(MapleSimConstants.SHOOTER_HEIGHT_METERS),
                    MetersPerSecond.of(projectileSpeedMps),
                    Radians.of(Math.PI / 2 - getLaunchAngleRadians()))
                // Set the target center to the Rebbuilt Hub of the current alliance
                .withTargetPosition(
                    () ->
                        new Translation3d(
                            FieldConstants.Hub.hubPosition().getX(),
                            FieldConstants.Hub.hubPosition().getY(),
                            FieldConstants.Hub.hubHeight))
                // Set the tolerance: x: ±0.5m, y: ±1.2m, z: ±0.3m (this is the size of the
                // speaker's "mouth")
                .withTargetTolerance(
                    new Translation3d(Units.feetToMeters(2), Units.feetToMeters(2), 0.1))
                // Set a callback to run when the fuel hits the target
                .withHitTargetCallBack(
                    () -> SimulatedArena.getInstance().addGamePiece(createHubBackSpawnFuel()))
                // Configure callbacks to visualize the flight trajectory of the projectile
                .withProjectileTrajectoryDisplayCallBack(
                    // Callback for when the fuel will eventually hit the target (if configured)
                    (pose3ds) ->
                        Logger.recordOutput(
                            "Flywheel/FuelProjectileSuccessfulShot",
                            pose3ds.toArray(Pose3d[]::new)),
                    // Callback for when the fuel will eventually miss the target, or if no target
                    // is configured
                    (pose3ds) ->
                        Logger.recordOutput(
                            "Flywheel/FuelProjectileUnsuccessfulShot",
                            pose3ds.toArray(Pose3d[]::new))));

    lastLaunchTimestampSeconds = Timer.getFPGATimestamp();
    Logger.recordOutput(
        "FieldSimulation/LaunchDebug/LastLaunchTimestampSec", lastLaunchTimestampSeconds);
  }

  public Command launchCommand() {
    return Commands.run(this::tryLaunch);
  }

  private boolean hasLaunchVelocity() {
    return Math.abs(shooterFlywheel.getVelocity()) >= MapleSimConstants.MIN_FLYWHEEL_RPS_FOR_SHOT;
  }

  private boolean pastCooldown() {
    return Timer.getFPGATimestamp() - lastLaunchTimestampSeconds
        >= MapleSimConstants.SHOT_COOLDOWN_SECONDS;
  }

  private double getLaunchAngleRadians() {
    double hoodAngle =
        Rotation2d.fromRotations(hood.getPosition()).getRadians()
            + MapleSimConstants.HOOD_ANGLE_OFFSET_RADIANS;
    return MathUtil.clamp(
        hoodAngle,
        MapleSimConstants.MIN_LAUNCH_ANGLE_RADIANS,
        MapleSimConstants.MAX_LAUNCH_ANGLE_RADIANS);
  }

  private double getProjectileSpeedMps() {
    return Math.abs(shooterFlywheel.getVelocity()) * MapleSimConstants.MPS_PER_FLYWHEEL_RPS;
  }

  private Translation2d getHubBackSpawnPosition() {
    double backDirection = Constants.isAllianceRed() ? -1.0 : 1.0;
    return FieldConstants.Hub.hubPosition()
        .plus(
            new Translation2d(
                backDirection * MapleSimConstants.HUB_BACK_SPAWN_X_OFFSET_METERS, 0.0));
  }

  private RebuiltFuelOnField createHubBackSpawnFuel() {
    double backDirection = Constants.isAllianceRed() ? -1.0 : 1.0;
    double baseHeadingRadians = backDirection > 0.0 ? 0.0 : Math.PI;
    double randomHeadingOffsetRadians =
        (Math.random() * 2.0 - 1.0)
            * Math.toRadians(MapleSimConstants.HUB_BACK_SPAWN_DIRECTION_RNG_DEG);
    double randomizedHeadingRadians = baseHeadingRadians + randomHeadingOffsetRadians;
    double velocityX =
        MapleSimConstants.HUB_BACK_SPAWN_SPEED_MPS * Math.cos(randomizedHeadingRadians);
    double velocityY =
        MapleSimConstants.HUB_BACK_SPAWN_SPEED_MPS * Math.sin(randomizedHeadingRadians);
    RebuiltFuelOnField spawnedFuel = new RebuiltFuelOnField(getHubBackSpawnPosition());
    spawnedFuel.setVelocity(new ChassisSpeeds(velocityX, velocityY, 0.0));
    return spawnedFuel;
  }
}
