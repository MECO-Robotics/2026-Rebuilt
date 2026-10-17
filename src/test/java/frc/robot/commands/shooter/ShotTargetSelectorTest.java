package frc.robot.commands.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.FieldConstants.AllianceZone;
import frc.robot.constants.FieldConstants.Hub;
import frc.robot.constants.FieldConstants.Trench;
import frc.robot.constants.subsystems.ShooterConstants;
import frc.robot.constants.vision.VisionConstants;
import org.junit.jupiter.api.Test;

class ShotTargetSelectorTest {
	private static final double EPSILON = 1e-9;
	private static final Pose2d NEUTRAL_ZONE_POSE = new Pose2d(VisionConstants.aprilTagLayout.getFieldLength() / 2.0,
			VisionConstants.aprilTagLayout.getFieldWidth() / 2.0, Rotation2d.kZero);

	@Test
	void selectsRedHubWhenAnyRedSideTagIsVisible() {
		ShotTarget target = ShotTargetSelector.select(Alliance.Red, NEUTRAL_ZONE_POSE, new int[]{18, 8});

		assertEquals(ShotTarget.Mode.HUB, target.mode());
		assertEquals(Hub.hubPosition(Alliance.Red), target.position());
	}

	@Test
	void selectsBlueHubWhenAnyBlueSideTagIsVisible() {
		ShotTarget target = ShotTargetSelector.select(Alliance.Blue, NEUTRAL_ZONE_POSE, new int[]{3, 24});

		assertEquals(ShotTarget.Mode.HUB, target.mode());
		assertEquals(Hub.hubPosition(Alliance.Blue), target.position());
	}

	@Test
	void selectsAllianceFerryTargetWhenOnlyOpponentTagsOrNoTagsAreVisible() {
		assertEquals(ShotTarget.Mode.FERRY,
				ShotTargetSelector.select(Alliance.Red, NEUTRAL_ZONE_POSE, new int[]{18, 32}).mode());
		assertEquals(ShotTarget.Mode.FERRY,
				ShotTargetSelector.select(Alliance.Blue, NEUTRAL_ZONE_POSE, new int[]{1, 16}).mode());
		assertEquals(ShotTarget.Mode.FERRY,
				ShotTargetSelector.select(Alliance.Red, NEUTRAL_ZONE_POSE, new int[0]).mode());
		assertEquals(ShotTarget.Mode.FERRY, ShotTargetSelector.select(Alliance.Blue, NEUTRAL_ZONE_POSE, null).mode());
	}

	@Test
	void alwaysSelectsHubWhenRobotIsAlreadyInItsAllianceZone() {
		Pose2d redAllianceZonePose = new Pose2d(AllianceZone.boundaryX(Alliance.Red) + 0.1, 1.0, Rotation2d.kZero);
		Pose2d blueAllianceZonePose = new Pose2d(AllianceZone.boundaryX(Alliance.Blue) - 0.1, 7.0, Rotation2d.kZero);

		assertEquals(ShotTarget.Mode.HUB,
				ShotTargetSelector.select(Alliance.Red, redAllianceZonePose, new int[0]).mode());
		assertEquals(ShotTarget.Mode.HUB,
				ShotTargetSelector.select(Alliance.Blue, blueAllianceZonePose, new int[0]).mode());
	}

	@Test
	void includesEveryOfficialTagOnTheCorrectAllianceSide() {
		for (int tagId = 1; tagId <= 16; tagId++) {
			assertTrue(ShotTargetSelector.seesAllianceSideTag(Alliance.Red, new int[]{tagId}));
			assertFalse(ShotTargetSelector.seesAllianceSideTag(Alliance.Blue, new int[]{tagId}));
		}
		for (int tagId = 17; tagId <= 32; tagId++) {
			assertTrue(ShotTargetSelector.seesAllianceSideTag(Alliance.Blue, new int[]{tagId}));
			assertFalse(ShotTargetSelector.seesAllianceSideTag(Alliance.Red, new int[]{tagId}));
		}
	}

	@Test
	void ferryTargetsProvideTwoSafePointsAndMatchTheRobotsFieldHalf() {
		double fieldLength = VisionConstants.aprilTagLayout.getFieldLength();
		double fieldWidth = VisionConstants.aprilTagLayout.getFieldWidth();
		Pose2d lowerNeutralPose = new Pose2d(fieldLength / 2.0, fieldWidth / 2.0 - 0.01, Rotation2d.kZero);
		Pose2d upperNeutralPose = new Pose2d(fieldLength / 2.0, fieldWidth / 2.0 + 0.01, Rotation2d.kZero);

		Translation2d redLowerTarget = ShotTargetSelector.ferry(Alliance.Red, lowerNeutralPose).position();
		Translation2d redUpperTarget = ShotTargetSelector.ferry(Alliance.Red, upperNeutralPose).position();
		Translation2d blueLowerTarget = ShotTargetSelector.ferry(Alliance.Blue, lowerNeutralPose).position();
		Translation2d blueUpperTarget = ShotTargetSelector.ferry(Alliance.Blue, upperNeutralPose).position();

		assertTrue(redLowerTarget.getY() < fieldWidth / 2.0);
		assertTrue(redUpperTarget.getY() > fieldWidth / 2.0);
		assertTrue(blueLowerTarget.getY() < fieldWidth / 2.0);
		assertTrue(blueUpperTarget.getY() > fieldWidth / 2.0);
		assertEquals(fieldWidth, redLowerTarget.getY() + redUpperTarget.getY(), 0.001);
		assertEquals(fieldWidth, blueLowerTarget.getY() + blueUpperTarget.getY(), 0.001);
		assertEquals(fieldLength, redLowerTarget.getX() + blueUpperTarget.getX(), 0.001);
		assertEquals(fieldLength, redUpperTarget.getX() + blueLowerTarget.getX(), 0.001);
		for (Translation2d target : new Translation2d[]{redLowerTarget, redUpperTarget}) {
			assertTrue(AllianceZone.contains(Alliance.Red, target));
			assertTrue(target.getDistance(Hub.hubPosition(Alliance.Red)) > 2.0);
		}
		for (Translation2d target : new Translation2d[]{blueLowerTarget, blueUpperTarget}) {
			assertTrue(AllianceZone.contains(Alliance.Blue, target));
			assertTrue(target.getDistance(Hub.hubPosition(Alliance.Blue)) > 2.0);
		}
	}

	@Test
	void ferrySolutionsOnBothSidesPassTrajectorySafetyChecks() {
		double fieldLength = VisionConstants.aprilTagLayout.getFieldLength();
		double fieldWidth = VisionConstants.aprilTagLayout.getFieldWidth();
		for (Alliance alliance : Alliance.values()) {
			for (double robotY : new double[]{0.5, fieldWidth - 0.5}) {
				Pose2d robotPose = new Pose2d(fieldLength / 2.0, robotY, Rotation2d.kZero);
				ShotTarget target = ShotTargetSelector.ferry(alliance, robotPose);
				ShotSolution solution = ShooterCalculator.calculate(robotPose, target);

				assertTrue(solution.calibrated(), alliance + " ferry target at Y=" + robotY + " must be safe");
				assertTrue(solution.flywheelRotationsPerSecond() <= ShooterConstants.SHOOTER_PRESET.FERRY.get());
			}
		}
	}

	@Test
	void ferrySolutionUsesFerryHoodAndRangeAdjustedSpeed() {
		ShotTarget ferryTarget = ShotTargetSelector.ferry(Alliance.Red, NEUTRAL_ZONE_POSE);
		Pose2d robotPose = NEUTRAL_ZONE_POSE;
		ShotSolution ferrySolution = ShooterCalculator.calculate(robotPose, ferryTarget);
		ShotSolution aimedSolution = ShooterCalculator.calculate(robotPose, ferryTarget.position());

		assertEquals(aimedSolution.targetHeading().getRadians(), ferrySolution.targetHeading().getRadians(), EPSILON);
		assertEquals(ShooterConstants.HOOD_PRESET.FERRY.get(), ferrySolution.hoodRotations(), EPSILON);
		assertEquals(ShooterCalculator.calculateFerryFlywheelRps(ferrySolution.distanceMeters(),
				ferrySolution.hoodRotations()), ferrySolution.flywheelRotationsPerSecond(), EPSILON);
		assertTrue(ferrySolution.calibrated());

		double launchPitch = Math.PI / 2.0 - ShooterConstants.HOOD_ZERO_DEFLECTION_FROM_VERTICAL_RADIANS
				- Rotation2d.fromRotations(ferrySolution.hoodRotations()).getRadians();
		double speed = ferrySolution.flywheelRotationsPerSecond()
				* ShooterConstants.PROJECTILE_METERS_PER_SECOND_PER_FLYWHEEL_RPS;
		double verticalSpeed = speed * Math.sin(launchPitch);
		double flightTime = (verticalSpeed + Math
				.sqrt(verticalSpeed * verticalSpeed + 2.0 * 9.80665 * ShooterConstants.SHOOTER_RELEASE_HEIGHT_METERS))
				/ 9.80665;
		double landingDistance = speed * Math.cos(launchPitch) * flightTime;
		assertEquals(ferrySolution.distanceMeters(), landingDistance, 1e-6);
	}

	@Test
	void ferrySolutionBlocksAutomaticFeedWhenSafeLandingPointIsOutOfReach() {
		Pose2d farCorner = new Pose2d(0.0, VisionConstants.aprilTagLayout.getFieldWidth(), Rotation2d.kZero);
		ShotSolution solution = ShooterCalculator.calculate(farCorner,
				ShotTargetSelector.ferry(Alliance.Red, farCorner));

		assertFalse(solution.calibrated());
		assertEquals(ShooterConstants.SHOOTER_PRESET.FERRY.get(), solution.flywheelRotationsPerSecond(), EPSILON);
	}

	@Test
	void ferrySolutionBlocksWhenShooterIsTooCloseToClearEntireTrench() {
		double robotX = Trench.neutralEdgeX(Alliance.Red) + 0.14;
		Pose2d underTrenchPose = new Pose2d(robotX, 1.0, Rotation2d.kZero);
		ShotSolution solution = ShooterCalculator.calculate(underTrenchPose,
				ShotTargetSelector.ferry(Alliance.Red, underTrenchPose));

		assertFalse(solution.calibrated());
	}

	@Test
	void ferrySolutionBlocksPathsThroughHubStructure() {
		Translation2d target = ShotTargetSelector.ferry(Alliance.Red, NEUTRAL_ZONE_POSE).position();
		Translation2d hub = Hub.hubPosition(Alliance.Red);
		Translation2d shooterPosition = hub.plus(hub.minus(target));
		Pose2d robotPose = new Pose2d(shooterPosition.minus(ShooterConstants.SHOOTER_EXIT_TRANSLATION),
				Rotation2d.kZero);
		ShotSolution solution = ShooterCalculator.calculate(robotPose, new ShotTarget(ShotTarget.Mode.FERRY, target));

		assertFalse(solution.calibrated());
	}
}
