package frc.robot.subsystems.position_joint;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.subsystems.IntakeConstants;
import org.junit.jupiter.api.Test;

class PositionJointDiagnosticsTest {
	private static class IO implements PositionJointIO {
		boolean configured = true;
		boolean requestAccepted = true;
		boolean connected = true;
		double target;
		public String getName() {
			return "RackDiagnosticsTest";
		}
		public void setPosition(double position, double velocity) {
			target = position;
		}
		public void updateInputs(PositionJointIOInputs inputs) {
			inputs.controllerDiagnosticsSupported = true;
			inputs.configurationHealthy = configured;
			inputs.configurationStatus = new String[]{configured ? "kOk" : "kCANDisconnected"};
			inputs.controlRequestHealthy = requestAccepted;
			inputs.controlStatus = requestAccepted ? "MAXMotion position: kOk" : "MAXMotion position: kTimeout";
			inputs.motorsConnected = new boolean[]{connected};
			inputs.motorVoltages = new double[]{3.2};
			inputs.motorCurrents = new double[]{2};
			inputs.atReverseLimit = true;
		}
	}

	@Test
	void configurationAndRequestErrorsRemainDistinctFromConnectivityAndAppliedOutput() {
		assertTrue(HAL.initialize(500, 0));
		var io = new IO();
		var joint = new PositionJoint(io, IntakeConstants.INTAKE_RACK_GAINS);
		try {
			joint.periodic(); // Initialize staged tuning before sending a command.
			joint.setPosition(.5);
			joint.periodic();
			assertEquals(.35, io.target, 1e-9);
			assertEquals(.5, joint.diagnosticMeasurements().get("RequestedPosition"));
			assertEquals(.35, joint.diagnosticMeasurements().get("ClampedTarget"));
			assertEquals(0, joint.getCommandedVoltage());
			assertEquals(3.2, joint.diagnosticMeasurements().get("MaxAppliedVolts"));
			assertEquals(1, joint.diagnosticMeasurements().get("AtReverseLimit"));
			assertEquals("", joint.controllerDiagnosticFailure());
			io.configured = false;
			joint.periodic();
			assertTrue(joint.controllerDiagnosticFailure().contains("configuration failed"));
			assertTrue(joint.getHealthSnapshot().allConnected());
			io.configured = true;
			io.requestAccepted = false;
			joint.periodic();
			assertTrue(joint.controllerDiagnosticFailure().contains("kTimeout"));
			io.connected = false;
			joint.periodic();
			assertFalse(joint.getHealthSnapshot().allConnected());
		} finally {
			CommandScheduler.getInstance().unregisterSubsystem(joint);
		}
	}

	@Test
	void rackConversionUsesMetresAndMetresPerSecondWithoutAnExtraSixty() {
		double ratio = IntakeConstants.INTAKE_RACK_CONFIG.gearRatio();
		double positionFactor = 1 / ratio;
		double velocityFactor = 1 / (60 * ratio);
		assertEquals(Math.PI * .0254 / 9, positionFactor, 1e-12);
		assertEquals(positionFactor, 60 * velocityFactor, 1e-12);
		assertEquals(.5, IntakeConstants.INTAKE_RACK_GAINS.kMaxVelo());
		assertEquals(100, IntakeConstants.INTAKE_RACK_GAINS.kMaxAccel());
	}
}
