package frc.robot.systemcheck;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import org.junit.jupiter.api.Test;

class SystemCheckCanTopologyTest {
	@Test
	void addsBusIdAndConfiguredNeighborsToHardwareFault() {
		SystemCheckCanTopology.CanDevice device = SystemCheckCanTopology.findDevice("ShooterFlywheel", 0, false)
				.orElseThrow();
		HardwareFault fault = SystemCheckCanTopology.hardwareFault(device, "DISCONNECTED", "No telemetry");

		assertEquals("MECO 2", fault.canBus());
		assertEquals(34, fault.canId());
		assertTrue(fault.upstreamNeighbor().contains("BottomIndexer"));
		assertTrue(fault.downstreamNeighbor().contains("ShooterFlywheel follower"));
		assertFalse(fault.physicalOrderVerified());
	}

	@Test
	void chainDiagnosisIdentifiesFirstConfiguredDisconnectedDevice() {
		DeviceHealth bottomIndexer = new DeviceHealth("BottomIndexer", new boolean[]{true}, new double[]{},
				new double[]{}, new double[]{}, new double[]{}, false, true, new String[]{}, new int[]{31});
		DeviceHealth shooter = new DeviceHealth("ShooterFlywheel", new boolean[]{false, false}, new double[]{},
				new double[]{}, new double[]{}, new double[]{}, false, true, new String[]{}, new int[]{34, 35});

		SystemCheckCanTopology.CanChainDiagnosis diagnosis = SystemCheckCanTopology
				.diagnoseChains(List.of(bottomIndexer, shooter)).stream()
				.filter(candidate -> candidate.busName().equals("MECO 2")).findFirst().orElseThrow();

		assertEquals(2, diagnosis.disconnectedDevices().size());
		assertTrue(diagnosis.likelyBreak().contains("BottomIndexer motor"));
		assertTrue(diagnosis.likelyBreak().contains("ShooterFlywheel leader"));
		assertTrue(diagnosis.likelyBreak().contains("not yet physically verified"));
	}

	@Test
	void chainDiagnosisDistinguishesAnIsolatedDeviceFromADownstreamTrunkLoss() {
		DeviceHealth bottomIndexer = new DeviceHealth("BottomIndexer", new boolean[]{false}, new double[]{},
				new double[]{}, new double[]{}, new double[]{}, false, true, new String[]{}, new int[]{31});
		DeviceHealth shooter = new DeviceHealth("ShooterFlywheel", new boolean[]{true, true}, new double[]{},
				new double[]{}, new double[]{}, new double[]{}, false, true, new String[]{}, new int[]{34, 35});

		SystemCheckCanTopology.CanChainDiagnosis diagnosis = SystemCheckCanTopology
				.diagnoseChains(List.of(bottomIndexer, shooter)).stream()
				.filter(candidate -> candidate.busName().equals("MECO 2")).findFirst().orElseThrow();

		assertTrue(diagnosis.likelyBreak().contains("later configured device responds"));
		assertTrue(diagnosis.likelyBreak().contains("power and local CAN connectors"));
	}
}
