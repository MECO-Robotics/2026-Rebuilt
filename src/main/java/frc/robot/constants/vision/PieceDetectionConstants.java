package frc.robot.constants.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.net.URI;

public final class PieceDetectionConstants {
	public record PieceDetectionConfig(Transform3d robotToCameraTransform) {
	}

	public record AreaRangeSample(double areaPixels, double rangeMeters) {
	}

	public static final PieceDetectionConfig EXAMPLE_CONFIG = new PieceDetectionConfig(
			new Transform3d(new Translation3d(0.2, 0.1, 0), new Rotation3d(90, 20, 0)));

	public static final String GAME_PIECE_DETECTION_NAME = "GamePieceDetection";
	public static final AreaRangeSample[] GAME_PIECE_AREA_RANGE_SAMPLES = new AreaRangeSample[]{
			// Add real tape-measured samples here as: new AreaRangeSample(areaPixels,
			// rangeMeters)
	};

	public static final boolean ENABLE_FRONT_GAME_PIECE_CAMERA = true;
	public static final String FRONT_GAME_PIECE_CAMERA_NAME = "FrontGamePieceCamera";
	public static final URI FRONT_GAME_PIECE_CAMERA_DATA_URI = URI.create("http://10.83.24.11:5800/data");
	public static final URI FRONT_GAME_PIECE_CAMERA_VIDEO_URI = URI.create("http://10.83.24.11:5800/video");
	public static final URI FRONT_GAME_PIECE_CAMERA_DASHBOARD_URI = URI.create("http://10.83.24.11:5800/");
	public static final PieceDetectionConfig FRONT_GAME_PIECE_CAMERA_CONFIG = new PieceDetectionConfig(
			new Transform3d(new Translation3d(0.2, 0.0, 0.45), new Rotation3d()));
	public static final double FRONT_GAME_PIECE_CAMERA_DISTANCE_GAIN = 7.0;

	public static final boolean ENABLE_REAR_GAME_PIECE_CAMERA = true;
	public static final String REAR_GAME_PIECE_CAMERA_NAME = "RearGamePieceCamera";
	public static final URI REAR_GAME_PIECE_CAMERA_DATA_URI = URI.create("http://10.83.24.12:5800/data");
	public static final URI REAR_GAME_PIECE_CAMERA_VIDEO_URI = URI.create("http://10.83.24.12:5800/video");
	public static final URI REAR_GAME_PIECE_CAMERA_DASHBOARD_URI = URI.create("http://10.83.24.12:5800/");
	public static final PieceDetectionConfig REAR_GAME_PIECE_CAMERA_CONFIG = new PieceDetectionConfig(
			new Transform3d(new Translation3d(-0.2, 0.0, 0.45), new Rotation3d(0.0, 0.0, Math.PI)));
	public static final double REAR_GAME_PIECE_CAMERA_DISTANCE_GAIN = 7.0;
}
