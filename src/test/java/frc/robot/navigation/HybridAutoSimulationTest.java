package frc.robot.navigation;

import static org.junit.jupiter.api.Assertions.*;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.simulation.HybridSimulationEnvironment;
import frc.robot.simulation.RobotSimulation;
import frc.robot.subsystems.vision.Vision;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Map;
import java.util.function.Supplier;
import org.junit.jupiter.api.Test;

/**
 * Run with hybridSimTest: one native robot, sixteen whole-auto trials, isolated
 * from other fixtures.
 */
class HybridAutoSimulationTest {
	@Test
	@SuppressWarnings("unchecked")
	void wholeAutosOnBothAlliancesProduceBoundedMeasuredOutcomes() throws Exception {
		assertTrue(HAL.initialize(500, 0));
		assertTrue(HybridSimulationEnvironment.enabled());
		var scheduler = CommandScheduler.getInstance();
		SimHooks.pauseTiming();
		DriverStationSim.resetData();
		DriverStationSim.setDsAttached(true);
		DriverStationSim.setAutonomous(true);
		DriverStationSim.setEnabled(false);
		for (int port = 0; port < 2; port++) {
			var joystick = new edu.wpi.first.wpilibj.simulation.XboxControllerSim(port);
			joystick.setAxisCount(6);
			joystick.setButtonCount(10);
			joystick.setPOVCount(1);
			joystick.notifyNewData();
		}
		RoboRioSim.setVInVoltage(12.5);
		RobotSimulation.configureArenaOverride(Constants.Mode.SIM);
		var container = new RobotContainer();
		var factoriesField = RobotContainer.class.getDeclaredField("namedCommandFactories");
		factoriesField.setAccessible(true);
		var factories = (Map<String, Supplier<Command>>) factoriesField.get(container);
		var visionField = RobotContainer.class.getDeclaredField("vision");
		visionField.setAccessible(true);
		var vision = (Vision) visionField.get(container);
		var results = new ArrayList<HybridSimulationAuto.Result>();
		var names = AutoBuilder.getAllAutoNames().stream().filter(n -> n.startsWith("HYBRID ")).sorted().toList();
		assertEquals(8, names.size());
		try {
			for (var alliance : new AllianceStationID[]{AllianceStationID.Blue1, AllianceStationID.Red1}) {
				for (var name : names) {
					scheduler.cancelAll();
					DriverStationSim.setEnabled(false);
					DriverStationSim.setAllianceStationId(alliance);
					tick(container);
					var candidate = HybridAutos.fromAutoFile(name, container.drivetrain, vision,
							key -> factories.get(key).get());
					assertInstanceOf(HybridSimulationAuto.class, candidate, candidate.getName());
					var auto = (HybridSimulationAuto) candidate;
					container.simulationAutonomousInit(auto);
					DriverStationSim.setEnabled(true);
					DriverStationSim.notifyNewData();
					DriverStation.refreshData();
					scheduler.schedule(auto);
					assertEquals(0, auto.startingPose().getTranslation()
							.getDistance(container.drivetrain.getPhysicsPose().getTranslation()), 1e-6);
					for (int i = 0; i < 9100 && auto.isScheduled(); i++)
						tick(container);
					assertFalse(auto.isScheduled(), name + " exceeded fixture timeout");
					var result = auto.result();
					assertNotNull(result);
					assertTrue(result.outcome().equals("COMPLETE") || result.outcome().equals("SAFE_ABORT"),
							result.toString());
					if (result.outcome().equals("COMPLETE"))
						assertEquals(result.totalPaths(), result.completedPaths());
					else
						assertFalse(result.reason().isBlank());
					results.add(result);
					System.out.println(result);
				}
			}
			var report = Path.of("build/reports/hybrid/whole-auto-simulation.csv");
			Files.createDirectories(report.getParent());
			Files.writeString(report, HybridSimulationAuto.CSV_HEADER + results.stream()
					.map(HybridSimulationAuto.Result::csv).collect(java.util.stream.Collectors.joining()));
			assertEquals(16, results.size());
			// Cancellation follows the same live command used by the GUI.
			var interrupted = (HybridSimulationAuto) HybridAutos.fromAutoFile(names.get(0), container.drivetrain,
					vision, key -> factories.get(key).get());
			scheduler.schedule(interrupted);
			tick(container);
			DriverStationSim.setEnabled(false);
			tick(container);
			assertFalse(interrupted.isScheduled());
			assertEquals("INTERRUPTED", interrupted.result().outcome());
		} finally {
			scheduler.cancelAll();
			scheduler.unregisterAllSubsystems();
			DriverStationSim.setEnabled(false);
			DriverStationSim.notifyNewData();
			DriverStation.refreshData();
			SimHooks.resumeTiming();
		}
	}

	private static void tick(RobotContainer container) throws InterruptedException {
		SimHooks.stepTiming(.02);
		DriverStationSim.notifyNewData();
		DriverStation.refreshData();
		CommandScheduler.getInstance().run();
		container.simulationPeriodic();
		// Native motor/status workers need wall-clock opportunities between accelerated
		// steps.
		Thread.sleep(1);
	}
}
