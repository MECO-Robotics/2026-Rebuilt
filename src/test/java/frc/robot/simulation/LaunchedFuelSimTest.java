package frc.robot.simulation;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.commands.shooter.ShooterCalculator;
import frc.robot.constants.simulation.MapleSimConstants;
import org.junit.jupiter.api.Test;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

class LaunchedFuelSimTest {
	private static final double EPSILON = 1e-9;

	@Test
	void shooterOriginAndLaunchDirectionUseTheIntakeSide() {
		assertTrue(MapleSimConstants.SHOOTER_TRANSLATION_ON_ROBOT.getX() < 0.0);
		assertEquals(Math.PI, Math.abs(MapleSimConstants.SHOOTER_YAW_OFFSET.getRadians()), EPSILON);
		assertEquals(Math.PI, Math.abs(LaunchedFuelSim.getShooterFacing(Rotation2d.kZero).getRadians()), EPSILON);
	}

	@Test
	void mapleFrameConversionKeepsProjectileAtPhysicalShooterExit() {
		Pose2d robotPose = new Pose2d(4.0, 3.0, Rotation2d.fromDegrees(37.0));
		int burstIndex = 0;
		double centeredIndex = burstIndex - (MapleSimConstants.FUEL_PER_SHOT - 1) / 2.0;
		Translation2d robotFrameOffset = MapleSimConstants.SHOOTER_TRANSLATION_ON_ROBOT
				.plus(new Translation2d(0.0, centeredIndex * MapleSimConstants.FUEL_BURST_LATERAL_SPACING_METERS));
		Translation2d expectedOrigin = robotPose.getTranslation()
				.plus(robotFrameOffset.rotateBy(robotPose.getRotation()));

		Rotation2d shooterFacing = LaunchedFuelSim.getShooterFacing(robotPose.getRotation());
		Translation2d actualOrigin = robotPose.getTranslation()
				.plus(LaunchedFuelSim.getMapleShooterTranslationForBurstIndex(burstIndex).rotateBy(shooterFacing));

		assertEquals(expectedOrigin.getX(), actualOrigin.getX(), EPSILON);
		assertEquals(expectedOrigin.getY(), actualOrigin.getY(), EPSILON);
	}

	@Test
	void autoAimPointsTheSimulatedProjectileAtTheTarget() {
		Translation2d target = Translation2d.kZero;
		Pose2d unalignedPose = new Pose2d(2.0, 0.0, Rotation2d.kZero);
		Rotation2d alignedHeading = ShooterCalculator.calculate(unalignedPose, target).targetHeading();
		Pose2d alignedPose = new Pose2d(unalignedPose.getTranslation(), alignedHeading);
		Translation2d shooterExit = alignedPose.getTranslation()
				.plus(MapleSimConstants.SHOOTER_TRANSLATION_ON_ROBOT.rotateBy(alignedPose.getRotation()));

		Rotation2d targetBearing = target.minus(shooterExit).getAngle();
		Rotation2d projectileHeading = LaunchedFuelSim.getShooterFacing(alignedPose.getRotation());

		assertEquals(0.0, projectileHeading.minus(targetBearing).getRadians(), EPSILON);
	}

	@Test
	void maplePitchIsComplementOfHoodDeflectionFromVertical() {
		double hoodDeflection = Units.degreesToRadians(25.0);

		assertEquals(Units.degreesToRadians(65.0), LaunchedFuelSim.toMapleLaunchPitchRadians(hoodDeflection), EPSILON);
	}

	@Test
	void changingHoodPositionChangesProjectilePitch() {
		double zeroHoodDeflection = LaunchedFuelSim.getHoodDeflectionFromVerticalRadians(0.0);
		double maximumHoodDeflection = LaunchedFuelSim.getHoodDeflectionFromVerticalRadians(0.049);
		double zeroHoodPitch = LaunchedFuelSim.toMapleLaunchPitchRadians(zeroHoodDeflection);
		double maximumHoodPitch = LaunchedFuelSim.toMapleLaunchPitchRadians(maximumHoodDeflection);

		assertEquals(74.0, Units.radiansToDegrees(zeroHoodPitch), 1e-9);
		assertEquals(56.36, Units.radiansToDegrees(maximumHoodPitch), 1e-9);
		assertTrue(zeroHoodPitch > maximumHoodPitch,
				"Increasing hood rotations should lower the projectile pitch from vertical toward horizontal");
	}

	@Test
	void mapleProjectileReceivesChangedVerticalVelocity() {
		double launchSpeedMetersPerSecond = 10.0;
		double zeroHoodPitch = LaunchedFuelSim
				.toMapleLaunchPitchRadians(LaunchedFuelSim.getHoodDeflectionFromVerticalRadians(0.0));
		double maximumHoodPitch = LaunchedFuelSim
				.toMapleLaunchPitchRadians(LaunchedFuelSim.getHoodDeflectionFromVerticalRadians(0.049));
		RebuiltFuelOnFly zeroHoodProjectile = createProjectile(launchSpeedMetersPerSecond, zeroHoodPitch);
		RebuiltFuelOnFly maximumHoodProjectile = createProjectile(launchSpeedMetersPerSecond, maximumHoodPitch);

		assertEquals(launchSpeedMetersPerSecond * Math.sin(zeroHoodPitch), zeroHoodProjectile.getVelocity3dMPS().getZ(),
				1e-9);
		assertEquals(launchSpeedMetersPerSecond * Math.sin(maximumHoodPitch),
				maximumHoodProjectile.getVelocity3dMPS().getZ(), 1e-9);
		assertTrue(zeroHoodProjectile.getVelocity3dMPS().getZ() > maximumHoodProjectile.getVelocity3dMPS().getZ());
	}

	private RebuiltFuelOnFly createProjectile(double launchSpeedMetersPerSecond, double launchPitchRadians) {
		return new RebuiltFuelOnFly(Translation2d.kZero, Translation2d.kZero, new ChassisSpeeds(), Rotation2d.kZero,
				edu.wpi.first.units.Units.Meters.of(MapleSimConstants.SHOOTER_HEIGHT_METERS),
				edu.wpi.first.units.Units.MetersPerSecond.of(launchSpeedMetersPerSecond),
				edu.wpi.first.units.Units.Radians.of(launchPitchRadians));
	}
}
