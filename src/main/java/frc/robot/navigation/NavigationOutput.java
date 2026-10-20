package frc.robot.navigation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.Consumer;

/**
 * One output gate per supervisor. Rate limits in the field frame across
 * controller handoffs.
 */
final class NavigationOutput {
	private final AutoSafetyConfig config;
	private final Consumer<ChassisSpeeds> output;
	private ChassisSpeeds previous = new ChassisSpeeds();
	private double translationLimit;
	private ChassisSpeeds requested = new ChassisSpeeds();
	NavigationOutput(AutoSafetyConfig config, Consumer<ChassisSpeeds> output) {
		this.config = config;
		this.output = output;
		translationLimit = config.maxSpeed();
	}
	void reset(ChassisSpeeds measured, Rotation2d heading) {
		previous = ChassisSpeeds.fromRobotRelativeSpeeds(measured, heading);
		double speed = Math.hypot(previous.vxMetersPerSecond, previous.vyMetersPerSecond);
		if (speed > config.maxSpeed()) {
			previous.vxMetersPerSecond *= config.maxSpeed() / speed;
			previous.vyMetersPerSecond *= config.maxSpeed() / speed;
		}
		previous.omegaRadiansPerSecond = MathUtil.clamp(previous.omegaRadiansPerSecond, -config.maxOmega(),
				config.maxOmega());
		requested = new ChassisSpeeds();
	}
	void request(ChassisSpeeds speeds) {
		if (!finite(speeds))
			throw new IllegalArgumentException("Nonfinite controller output");
		requested = speeds;
	}
	ChassisSpeeds calculate(Rotation2d heading, double dt) {
		var next = ChassisSpeeds.fromRobotRelativeSpeeds(requested, heading);
		double norm = Math.hypot(next.vxMetersPerSecond, next.vyMetersPerSecond);
		if (norm > translationLimit) {
			next.vxMetersPerSecond *= translationLimit / norm;
			next.vyMetersPerSecond *= translationLimit / norm;
		}
		double dx = next.vxMetersPerSecond - previous.vxMetersPerSecond;
		double dy = next.vyMetersPerSecond - previous.vyMetersPerSecond;
		double factor = Math.min(1, config.maxAcceleration() * dt / Math.max(1e-9, Math.hypot(dx, dy)));
		next.vxMetersPerSecond = previous.vxMetersPerSecond + dx * factor;
		next.vyMetersPerSecond = previous.vyMetersPerSecond + dy * factor;
		next.omegaRadiansPerSecond = previous.omegaRadiansPerSecond
				+ MathUtil.clamp(MathUtil.clamp(next.omegaRadiansPerSecond, -config.maxOmega(), config.maxOmega())
						- previous.omegaRadiansPerSecond, -config.maxAlpha() * dt, config.maxAlpha() * dt);
		previous = next;
		var robot = ChassisSpeeds.fromFieldRelativeSpeeds(next, heading);
		return robot;
	}
	void setTranslationLimit(double value) {
		if (!Double.isFinite(value) || value <= 0)
			throw new IllegalArgumentException("Invalid speed cap");
		translationLimit = Math.min(config.maxSpeed(), value);
	}
	void publish(ChassisSpeeds speeds) {
		output.accept(speeds);
	}
	void stop() {
		previous = new ChassisSpeeds();
		requested = new ChassisSpeeds();
		output.accept(new ChassisSpeeds());
	}
	static boolean finite(ChassisSpeeds s) {
		return s != null && Double.isFinite(s.vxMetersPerSecond) && Double.isFinite(s.vyMetersPerSecond)
				&& Double.isFinite(s.omegaRadiansPerSecond);
	}
}
