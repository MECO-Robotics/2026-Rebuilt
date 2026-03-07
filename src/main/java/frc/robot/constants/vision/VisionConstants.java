package frc.robot.constants.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;

/** Camera geometry and filtering constants for the vision subsystem. */
public class VisionConstants {
	// AprilTag layout
	public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

	// Camera names, must match names configured on coprocessor
	public static String questName = "QuestNav";
	public static String arducamName = "arducam";

	// Robot to camera transforms
	// (Not used by Limelight, configure in web UI instead)
	public static Transform3d robotToQuest = new Transform3d(Units.inchesToMeters(-(27.0 / 2.0)),
			Units.inchesToMeters(0), Units.inchesToMeters(8.5),
			new Rotation3d(Units.degreesToRadians(180), 0.0, Units.degreesToRadians(180)));
	public static Transform3d robotToArducam = new Transform3d(Units.inchesToMeters(0), Units.inchesToMeters(-10),
			Units.inchesToMeters(18.5), new Rotation3d(0.0, Units.degreesToRadians(-35), 0));

	// Basic filtering thresholds
	public static int minTagCountForOdometry = 2;
	public static int minWhitelistedTagCountForOdometry = 2;
	public static double maxAmbiguity = 0.3;
	public static double maxZError = 0.75;
	// Configure this for red alliance; blue is mirrored automatically.
	public static int[] odometryTagWhitelistRed = new int[]{};

	// Standard deviation baselines, for 1 meter distance and 1 tag
	// (Adjusted automatically based on distance and # of tags)
	public static double linearStdDevBaseline = 0.02; // Meters
	public static double angularStdDevBaseline = 0.06; // Radians

	// Standard deviation multipliers for each camera
	// (Adjust to trust some cameras more than others)
	public static double[] cameraStdDevFactors = new double[]{1.0, // Camera 0
			1.0 // Camera 1
	};

	// Multipliers to apply for MegaTag 2 observations
	public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
	public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available

	private static final Map<Integer, Integer> mirroredTagMap = buildMirroredTagMap();

	public static Set<Integer> getOdometryTagWhitelistForCurrentAlliance() {
		Set<Integer> redWhitelist = Arrays.stream(odometryTagWhitelistRed).boxed().collect(Collectors.toSet());
		if (DriverStation.getAlliance().orElse(Alliance.Red) != Alliance.Blue) {
			return redWhitelist;
		}
		return redWhitelist.stream().map((id) -> mirroredTagMap.getOrDefault(id, id)).collect(Collectors.toSet());
	}

	private static Map<Integer, Integer> buildMirroredTagMap() {
		Map<Integer, Integer> mirrored = new HashMap<>();
		double fieldLength = aprilTagLayout.getFieldLength();
		double fieldWidth = aprilTagLayout.getFieldWidth();

		for (var tag : aprilTagLayout.getTags()) {
			double mirrorX = fieldLength - tag.pose.getX();
			double mirrorY = fieldWidth - tag.pose.getY();
			double bestDistance = Double.POSITIVE_INFINITY;
			int bestId = tag.ID;

			for (var candidate : aprilTagLayout.getTags()) {
				double dx = candidate.pose.getX() - mirrorX;
				double dy = candidate.pose.getY() - mirrorY;
				double distance = dx * dx + dy * dy;
				if (distance < bestDistance) {
					bestDistance = distance;
					bestId = candidate.ID;
				}
			}

			mirrored.put(tag.ID, bestId);
		}
		return mirrored;
	}
}
