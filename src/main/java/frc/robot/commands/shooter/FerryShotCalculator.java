package frc.robot.commands.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.FieldConstants.AllianceZone;
import frc.robot.constants.FieldConstants.Hub;
import frc.robot.constants.FieldConstants.Trench;
import frc.robot.constants.subsystems.ShooterConstants;
import frc.robot.constants.vision.VisionConstants;

/**
 * Pure ballistic planning and field-structure safety checks for ferry shots.
 */
final class FerryShotCalculator {
	private static final int HOOD_SEARCH_STEPS = 50;
	private static final double GRAVITY_METERS_PER_SECOND_SQUARED = 9.80665;
	private static final double ANGLE_VARIATION_RADIANS = Math.toRadians(1.0);
	private static final double SPEED_VARIATION_RATIO = 0.01;
	private static final double STRUCTURE_CLEARANCE_MARGIN_METERS = 0.10;
	private static final double BURST_HALF_WIDTH_METERS = 0.12;
	private static final double HUB_HALF_WIDTH_METERS = 0.5 * edu.wpi.first.math.util.Units.inchesToMeters(47.0);

	private FerryShotCalculator() {
	}

	/** Returns the lowest safe ferry arc supported by the configured mechanisms. */
	static FerryShotPlan calculate(Pose2d robotPose, ShotTarget target) {
		Translation2d shooterPosition = robotPose
				.transformBy(new Transform2d(ShooterConstants.SHOOTER_EXIT_TRANSLATION, Rotation2d.kZero))
				.getTranslation();
		double distanceMeters = shooterPosition.getDistance(target.position());
		double minimumHoodRotations = ShooterConstants.HOOD_GAINS.kMinPosition();
		double maximumHoodRotations = MathUtil.clamp(ShooterConstants.HOOD_PRESET.FERRY.get(), minimumHoodRotations,
				ShooterConstants.HOOD_GAINS.kMaxPosition());
		double maximumFlywheelRps = Math.max(0.0, ShooterConstants.SHOOTER_PRESET.FERRY.get());
		Alliance alliance = target.position().getX() < VisionConstants.aprilTagLayout.getFieldLength() / 2.0
				? Alliance.Blue
				: Alliance.Red;

		if (isHorizontalPathSafe(alliance, shooterPosition, target.position())) {
			for (int step = 0; step <= HOOD_SEARCH_STEPS; step++) {
				double hoodRotations = maximumHoodRotations
						- (maximumHoodRotations - minimumHoodRotations) * step / HOOD_SEARCH_STEPS;
				double flywheelRps = calculateFlywheelRps(distanceMeters, hoodRotations);
				if (Double.isFinite(flywheelRps) && flywheelRps <= maximumFlywheelRps
						&& clearsTrench(alliance, shooterPosition, target.position(), hoodRotations, flywheelRps)) {
					return new FerryShotPlan(hoodRotations, flywheelRps, true);
				}
			}
		}

		double fallbackRps = calculateFlywheelRps(distanceMeters, maximumHoodRotations);
		return new FerryShotPlan(maximumHoodRotations, MathUtil.clamp(fallbackRps, 0.0, maximumFlywheelRps), false);
	}

	/** Calculates RPS for a shot that lands on the carpet at the given distance. */
	static double calculateFlywheelRps(double distanceMeters, double hoodRotations) {
		double launchPitchRadians = launchPitchRadians(hoodRotations);
		double cosine = Math.cos(launchPitchRadians);
		double heightTerm = ShooterConstants.SHOOTER_RELEASE_HEIGHT_METERS
				+ distanceMeters * Math.tan(launchPitchRadians);
		if (!Double.isFinite(distanceMeters) || distanceMeters < 0.0 || cosine <= 0.0 || heightTerm <= 0.0) {
			return Double.POSITIVE_INFINITY;
		}
		double projectileSpeedMetersPerSecond = Math.sqrt(GRAVITY_METERS_PER_SECOND_SQUARED * distanceMeters
				* distanceMeters / (2.0 * cosine * cosine * heightTerm));
		return projectileSpeedMetersPerSecond / ShooterConstants.PROJECTILE_METERS_PER_SECOND_PER_FLYWHEEL_RPS;
	}

	private static boolean isHorizontalPathSafe(Alliance alliance, Translation2d shooterPosition,
			Translation2d targetPosition) {
		if (!AllianceZone.contains(alliance, targetPosition)) {
			return false;
		}
		boolean startsOnNeutralSide = alliance == Alliance.Blue
				? shooterPosition.getX() > Trench.neutralEdgeX(alliance)
				: shooterPosition.getX() < Trench.neutralEdgeX(alliance);
		if (!startsOnNeutralSide) {
			return false;
		}

		double hubKeepOutRadius = Math.hypot(HUB_HALF_WIDTH_METERS, HUB_HALF_WIDTH_METERS) + Trench.FUEL_RADIUS_METERS
				+ BURST_HALF_WIDTH_METERS + STRUCTURE_CLEARANCE_MARGIN_METERS;
		return distanceToSegment(Hub.hubPosition(alliance), shooterPosition, targetPosition) > hubKeepOutRadius;
	}

	private static boolean clearsTrench(Alliance alliance, Translation2d shooterPosition, Translation2d targetPosition,
			double hoodRotations, double flywheelRps) {
		double minimumCenterHeight = Trench.HEIGHT_METERS + Trench.FUEL_RADIUS_METERS
				+ STRUCTURE_CLEARANCE_MARGIN_METERS;
		return projectileHeightAtX(shooterPosition, targetPosition, Trench.neutralEdgeX(alliance), hoodRotations,
				flywheelRps) >= minimumCenterHeight
				&& projectileHeightAtX(shooterPosition, targetPosition, Trench.allianceEdgeX(alliance), hoodRotations,
						flywheelRps) >= minimumCenterHeight;
	}

	private static double projectileHeightAtX(Translation2d shooterPosition, Translation2d targetPosition,
			double fieldX, double hoodRotations, double flywheelRps) {
		double deltaX = targetPosition.getX() - shooterPosition.getX();
		if (Math.abs(deltaX) < 1e-9) {
			return Double.NEGATIVE_INFINITY;
		}
		double interpolation = (fieldX - shooterPosition.getX()) / deltaX;
		if (interpolation < 0.0 || interpolation > 1.0) {
			return Double.NEGATIVE_INFINITY;
		}

		double horizontalDistance = shooterPosition.getDistance(targetPosition) * interpolation;
		double conservativePitch = launchPitchRadians(hoodRotations) - ANGLE_VARIATION_RADIANS;
		double conservativeSpeed = flywheelRps * ShooterConstants.PROJECTILE_METERS_PER_SECOND_PER_FLYWHEEL_RPS
				* (1.0 - SPEED_VARIATION_RATIO);
		double horizontalSpeed = conservativeSpeed * Math.cos(conservativePitch);
		if (horizontalSpeed <= 0.0) {
			return Double.NEGATIVE_INFINITY;
		}
		double timeSeconds = horizontalDistance / horizontalSpeed;
		return ShooterConstants.SHOOTER_RELEASE_HEIGHT_METERS
				+ conservativeSpeed * Math.sin(conservativePitch) * timeSeconds
				- 0.5 * GRAVITY_METERS_PER_SECOND_SQUARED * timeSeconds * timeSeconds;
	}

	private static double launchPitchRadians(double hoodRotations) {
		double hoodDeflectionFromVertical = ShooterConstants.HOOD_ZERO_DEFLECTION_FROM_VERTICAL_RADIANS
				+ Rotation2d.fromRotations(hoodRotations).getRadians();
		return Math.PI / 2.0 - hoodDeflectionFromVertical;
	}

	private static double distanceToSegment(Translation2d point, Translation2d start, Translation2d end) {
		Translation2d segment = end.minus(start);
		double lengthSquared = segment.getNorm() * segment.getNorm();
		if (lengthSquared <= 1e-9) {
			return point.getDistance(start);
		}
		double projection = ((point.getX() - start.getX()) * segment.getX()
				+ (point.getY() - start.getY()) * segment.getY()) / lengthSquared;
		projection = MathUtil.clamp(projection, 0.0, 1.0);
		return point.getDistance(start.plus(segment.times(projection)));
	}

	record FerryShotPlan(double hoodRotations, double flywheelRps, boolean safe) {
	}
}
