package frc.robot.navigation;

import static org.junit.jupiter.api.Assertions.*;

import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class AutoMissionPlanTest {
	@BeforeAll
	static void initialize() {
		assertTrue(HAL.initialize(500, 0));
	}

	private static PathPlannerPath path(double finalSpeed, List<EventMarker> markers) {
		return new PathPlannerPath(
				PathPlannerPath.waypointsFromPoses(new Pose2d(2, 2, Rotation2d.kZero),
						new Pose2d(3, 2, Rotation2d.kZero)),
				List.of(), List.of(), List.of(), markers, new PathConstraints(2, 2, 3, 6),
				new IdealStartingState(0, Rotation2d.kZero), new GoalEndState(finalSpeed, Rotation2d.kZero), false);
	}

	@Test
	void requiresExplicitStoppingObjectivesWithoutEmbeddedMarkers() {
		assertThrows(IllegalArgumentException.class, () -> new AutoMissionPlan(List.of()));
		assertThrows(IllegalArgumentException.class,
				() -> new AutoMissionPlan.Segment("moving", path(0.5, List.of()), Commands.none()));
		assertThrows(IllegalArgumentException.class, () -> new AutoMissionPlan.Segment("marker",
				path(0, List.of(new EventMarker("score", 0.5, Commands.none()))), Commands.none()));
		assertThrows(IllegalArgumentException.class,
				() -> new AutoMissionPlan.Segment(null, path(0, List.of()), Commands.none()));
		assertThrows(IllegalArgumentException.class,
				() -> new AutoMissionPlan.Segment("missing path", null, Commands.none()));
		assertThrows(IllegalArgumentException.class,
				() -> new AutoMissionPlan.Segment("missing action", path(0, List.of()), null));
	}

	@Test
	void snapshotsMissionStructure() {
		var segment = new AutoMissionPlan.Segment("stop", path(0, List.of()), Commands.none());
		var mutable = new ArrayList<>(List.of(segment));
		var mission = new AutoMissionPlan(mutable);
		mutable.clear();
		assertEquals(List.of(segment), mission.segments());
		assertThrows(UnsupportedOperationException.class, () -> mission.segments().clear());
	}

	@Test
	void rejectsCrossingAndRetracedGeometryThatCouldSkipARequiredBranch() {
		for (var positions : List.of(
				List.of(new Translation2d(2, 2), new Translation2d(4, 4), new Translation2d(2, 4),
						new Translation2d(4, 2)),
				List.of(new Translation2d(2, 2), new Translation2d(4, 2), new Translation2d(4, 3),
						new Translation2d(3, 2)),
				List.of(new Translation2d(2, 2), new Translation2d(4, 2), new Translation2d(5, 2),
						new Translation2d(3, 2)))) {
			var crossing = PathPlannerPath.fromPathPoints(positions.stream().map(PathPoint::new).toList(),
					new PathConstraints(2, 2, 3, 6), new GoalEndState(0, Rotation2d.kZero));
			assertThrows(IllegalArgumentException.class,
					() -> new AutoMissionPlan.Segment("ambiguous branch", crossing, Commands.none()));
		}
	}

	@Test
	void deployedSurveySegmentsAndBothAllianceVariantsHaveFullFootprintClearance() throws Exception {
		var safety = NavigationSafety.fromDeploy(0.9);
		for (String name : List.of("Hybrid Survey Out", "Hybrid Survey Return")) {
			var authored = PathPlannerPath.fromPathFile(name);
			for (boolean flipped : new boolean[]{false, true}) {
				var selected = flipped ? authored.flipPath() : authored;
				assertDoesNotThrow(() -> new AutoMissionPlan.Segment(name, selected, Commands.none()));
				var points = selected.getPathPoses();
				assertTrue(points.size() >= 2);
				for (int i = 0; i < points.size() - 1; i++) {
					assertTrue(safety.safeSegment(points.get(i).getTranslation(), points.get(i + 1).getTranslation()),
							name + " flipped=" + flipped + " unsafe edge=" + i);
				}
			}
		}
	}
}
