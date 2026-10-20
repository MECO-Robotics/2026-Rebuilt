package frc.robot.navigation;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Commands;
import java.nio.file.Files;
import java.util.List;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class HybridAutoFileTest {
	@BeforeAll
	static void initialize() {
		assertTrue(HAL.initialize(500, 0));
	}

	private static String auto(String command) {
		return """
				{"version":"2025.0","resetOdom":false,"choreoAuto":false,"command":%s}
				""".formatted(command);
	}

	@Test
	void deployedSurveyLoadsBothValidStoppingPaths() throws Exception {
		var file = Filesystem.getDeployDirectory().toPath()
				.resolve("pathplanner/autos/" + HybridAutos.SURVEY_AUTO_NAME + ".auto");
		assertEquals(List.of("Hybrid Survey Out", "Hybrid Survey Return"),
				HybridAutoFile.pathNames(Files.readString(file)));
		var paths = HybridAutoFile.load(HybridAutos.SURVEY_AUTO_NAME);
		assertEquals(2, paths.size());
		for (var path : paths)
			assertDoesNotThrow(() -> new AutoMissionPlan.Segment(path.name, path, Commands.none()));
	}

	@Test
	void editedOrderAndNestedSequencesArePreserved() throws Exception {
		assertEquals(List.of("Return", "Out"), HybridAutoFile.pathNames(auto("""
				{"type":"sequential","data":{"commands":[
				  {"type":"path","data":{"pathName":"Return"}},
				  {"type":"sequential","data":{"commands":[
				    {"type":"path","data":{"pathName":"Out"}}
				  ]}}
				]}}
				""")));
	}

	@Test
	void rejectsCommandsThatWouldOtherwiseBeSilentlySkipped() {
		for (var type : List.of("named", "wait", "parallel", "race", "deadline"))
			assertThrows(IllegalArgumentException.class, () -> HybridAutoFile.pathNames(auto("""
					{"type":"sequential","data":{"commands":[
					  {"type":"path","data":{"pathName":"Out"}},
					  {"type":"%s","data":{}}
					]}}
					""".formatted(type))));
	}

	@Test
	void rejectsResetChoreoAndEmptyMissions() {
		var valid = auto("""
				{"type":"path","data":{"pathName":"Out"}}
				""");
		assertThrows(IllegalArgumentException.class,
				() -> HybridAutoFile.pathNames(valid.replace("\"resetOdom\":false", "\"resetOdom\":true")));
		assertThrows(IllegalArgumentException.class,
				() -> HybridAutoFile.pathNames(valid.replace("\"choreoAuto\":false", "\"choreoAuto\":true")));
		assertThrows(IllegalArgumentException.class, () -> HybridAutoFile.pathNames(auto("""
				{"type":"sequential","data":{"commands":[]}}
				""")));
	}
}
