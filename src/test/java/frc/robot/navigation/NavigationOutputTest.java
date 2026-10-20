package frc.robot.navigation;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

class NavigationOutputTest {
	private static final double DT = 0.02;
	private static final AutoSafetyConfig CONFIG = AutoSafetyConfig.defaults();

	private static double translation(ChassisSpeeds s) {
		return Math.hypot(s.vxMetersPerSecond, s.vyMetersPerSecond);
	}

	@Test
	void calculatesWithoutPublishingAndStopClearsPendingMotion() {
		List<ChassisSpeeds> published = new ArrayList<>();
		var output = new NavigationOutput(CONFIG, published::add);
		output.reset(new ChassisSpeeds(), Rotation2d.kZero);
		output.request(new ChassisSpeeds(2, 0, 1));
		var candidate = output.calculate(Rotation2d.kZero, DT);
		assertTrue(published.isEmpty(), "Safety must inspect output before the drivetrain sees it");
		output.publish(candidate);
		assertEquals(1, published.size());
		assertEquals(CONFIG.maxAcceleration() * DT, published.get(0).vxMetersPerSecond, 1e-9);
		output.stop();
		assertEquals(0, translation(published.get(1)), 1e-9);
		assertEquals(0, published.get(1).omegaRadiansPerSecond, 1e-9);
		var afterStop = output.calculate(Rotation2d.kZero, DT);
		assertEquals(0, translation(afterStop), 1e-9);
		assertEquals(0, afterStop.omegaRadiansPerSecond, 1e-9);
	}

	@Test
	void limitsVectorAccelerationAndAngularAccelerationAndMaximumSpeeds() {
		var output = new NavigationOutput(CONFIG, ignored -> {
		});
		output.reset(new ChassisSpeeds(), Rotation2d.kZero);
		output.request(new ChassisSpeeds(20, 20, 30));
		var previous = new ChassisSpeeds();
		for (int i = 0; i < 200; i++) {
			var next = output.calculate(Rotation2d.kZero, DT);
			assertTrue(translation(next) <= CONFIG.maxSpeed() + 1e-9);
			assertTrue(Math.abs(next.omegaRadiansPerSecond) <= CONFIG.maxOmega() + 1e-9);
			assertTrue(Math.hypot(next.vxMetersPerSecond - previous.vxMetersPerSecond,
					next.vyMetersPerSecond - previous.vyMetersPerSecond) <= CONFIG.maxAcceleration() * DT + 1e-9);
			assertTrue(Math.abs(next.omegaRadiansPerSecond - previous.omegaRadiansPerSecond) <= CONFIG.maxAlpha() * DT
					+ 1e-9);
			previous = next;
		}
		assertEquals(CONFIG.maxSpeed(), translation(previous), 1e-9);
		assertEquals(CONFIG.maxOmega(), previous.omegaRadiansPerSecond, 1e-9);
		output.request(new ChassisSpeeds(-20, -20, -30));
		var reversal = output.calculate(Rotation2d.kZero, DT);
		assertEquals(CONFIG.maxAcceleration() * DT, Math.hypot(reversal.vxMetersPerSecond - previous.vxMetersPerSecond,
				reversal.vyMetersPerSecond - previous.vyMetersPerSecond), 1e-9);
		assertEquals(CONFIG.maxAlpha() * DT, previous.omegaRadiansPerSecond - reversal.omegaRadiansPerSecond, 1e-9);
	}

	@Test
	void preservesFieldVelocityAcrossHeadingChangeAndControllerHandoff() {
		var output = new NavigationOutput(CONFIG, ignored -> {
		});
		output.reset(new ChassisSpeeds(1, 0, 0), Rotation2d.kZero);
		Rotation2d quarterTurn = Rotation2d.fromDegrees(90);
		output.request(new ChassisSpeeds(0, -1, 0));
		var field = ChassisSpeeds.fromRobotRelativeSpeeds(output.calculate(quarterTurn, DT), quarterTurn);
		assertEquals(1, field.vxMetersPerSecond, 1e-9);
		assertEquals(0, field.vyMetersPerSecond, 1e-9);
		output.request(new ChassisSpeeds(0, 1, 0));
		var handoff = ChassisSpeeds.fromRobotRelativeSpeeds(output.calculate(quarterTurn, DT), quarterTurn);
		assertEquals(1 - CONFIG.maxAcceleration() * DT, handoff.vxMetersPerSecond, 1e-9);
		assertEquals(0, handoff.vyMetersPerSecond, 1e-9);
	}

	@Test
	void rejectsNonfiniteControllerRequests() {
		var output = new NavigationOutput(CONFIG, ignored -> {
		});
		assertThrows(IllegalArgumentException.class, () -> output.request(null));
		assertThrows(IllegalArgumentException.class, () -> output.request(new ChassisSpeeds(Double.NaN, 0, 0)));
		assertThrows(IllegalArgumentException.class,
				() -> output.request(new ChassisSpeeds(0, Double.POSITIVE_INFINITY, 0)));
		assertThrows(IllegalArgumentException.class, () -> output.request(new ChassisSpeeds(0, 0, Double.NaN)));
		assertFalse(NavigationOutput.finite(null));
		assertFalse(NavigationOutput.finite(new ChassisSpeeds(0, 0, Double.NEGATIVE_INFINITY)));
		assertTrue(NavigationOutput.finite(new ChassisSpeeds()));
	}
}
