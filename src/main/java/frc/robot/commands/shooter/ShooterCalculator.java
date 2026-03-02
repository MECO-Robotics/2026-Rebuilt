package frc.robot.commands.shooter;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants.Hub;
import frc.robot.commands.shooter.ShooterCommands.Maps;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.position_joint.PositionJoint;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/** Continuously sets a hood joint position from a distance-to-angle heuristic. */
public class ShooterCalculator {
  // - Get drive position
  // - Find distance between shooter and hub
  // - run calculation for hood and shooter
  // - run positionjoint position for hood
  // - run flywheel velocity for shooter

  public static final Translation2d robotToShooter = new Translation2d(-.19, 0);

  public static Command calculateAndShoot(Drive drive, PositionJoint hood, Flywheel shooter) {

    Pose2d shooterPosition =
        drive.getPose().transformBy(new Transform2d(robotToShooter, Rotation2d.kZero));
    Supplier<Distance> distance =
        () -> Meters.of(Hub.hubPosition().getDistance(shooterPosition.getTranslation()));

    DoubleSupplier hoodPosition =
        () -> Maps.shooterDeflectorAngleMap.get(distance.get()).in(Units.Rotations);

    DoubleSupplier shooterVelocity =
        () -> Maps.shooterVelocityMap.get(distance.get()).in(Units.RevolutionsPerSecond);

    return PositionJoint.setPosition(hood, hoodPosition)
        .alongWith(Flywheel.setVelocity(shooter, shooterVelocity));
  }
}
