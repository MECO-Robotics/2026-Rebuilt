package frc.robot.constants.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

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
	public static double maxAmbiguity = 0.3;
	public static double maxZError = 0.75;

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
}
