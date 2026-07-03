package frc.robot.util.visualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.nio.charset.StandardCharsets;
import java.util.Locale;

/**
 * Publishes live simulation state to NateSim's Unity UDP bridge.
 */
public class NateSimPublisher {
	private static final String DEFAULT_HOST = "127.0.0.1";
	private static final int DEFAULT_PORT = 5808;

	private final DatagramSocket socket;
	private final InetAddress host;
	private final int port;
	private boolean hasReportedFailure = false;

	public NateSimPublisher() {
		this(DEFAULT_HOST, DEFAULT_PORT);
	}

	public NateSimPublisher(String host, int port) {
		this.port = port;

		DatagramSocket createdSocket = null;
		InetAddress resolvedHost = null;
		try {
			createdSocket = new DatagramSocket();
			resolvedHost = InetAddress.getByName(host);
		} catch (IOException ex) {
			hasReportedFailure = true;
			System.out.println("NateSim publisher disabled: " + ex.getMessage());
		}

		socket = createdSocket;
		this.host = resolvedHost;
	}

	public void publish(Pose2d robotPose, String[] componentNames, Pose3d[] componentPoses, Pose3d[] fuelPoses,
			Pose3d[] projectilePoses, int successfulScoreCount, int launchEventId) {
		if (socket == null || host == null || robotPose == null) {
			return;
		}

		String json = buildJson(robotPose, componentNames, componentPoses, fuelPoses, projectilePoses,
				successfulScoreCount, launchEventId);
		byte[] bytes = json.getBytes(StandardCharsets.UTF_8);
		DatagramPacket packet = new DatagramPacket(bytes, bytes.length, host, port);

		try {
			socket.send(packet);
		} catch (IOException ex) {
			if (!hasReportedFailure) {
				hasReportedFailure = true;
				System.out.println("Failed to publish NateSim frame: " + ex.getMessage());
			}
		}
	}

	private String buildJson(Pose2d robotPose, String[] componentNames, Pose3d[] componentPoses, Pose3d[] fuelPoses,
			Pose3d[] projectilePoses, int successfulScoreCount, int launchEventId) {
		StringBuilder builder = new StringBuilder(2048);
		builder.append("{");
		appendNumberField(builder, "timestamp", edu.wpi.first.wpilibj.Timer.getFPGATimestamp());
		builder.append("\"robotPose\":");
		appendPose2d(builder, robotPose);
		builder.append(",");
		builder.append("\"componentPoses\":");
		appendNamedPose3dArray(builder, componentNames, componentPoses);
		builder.append(",");
		builder.append("\"fuelPoses\":");
		appendPose3dArray(builder, fuelPoses);
		builder.append(",");
		builder.append("\"projectilePoses\":");
		appendPose3dArray(builder, projectilePoses);
		builder.append(",");
		appendIntField(builder, "successfulScoreCount", successfulScoreCount);
		appendIntField(builder, "launchEventId", launchEventId, false);
		builder.append("}");
		return builder.toString();
	}

	private void appendPose2d(StringBuilder builder, Pose2d pose) {
		builder.append("{");
		appendNumberField(builder, "x", pose.getX());
		appendNumberField(builder, "y", pose.getY());
		appendNumberField(builder, "theta", pose.getRotation().getRadians(), false);
		builder.append("}");
	}

	private void appendNamedPose3dArray(StringBuilder builder, String[] names, Pose3d[] poses) {
		if (poses == null) {
			builder.append("[]");
			return;
		}

		builder.append("[");
		for (int i = 0; i < poses.length; i++) {
			if (i > 0) {
				builder.append(",");
			}
			String name = names != null && i < names.length ? names[i] : "";
			appendPose3d(builder, name, poses[i]);
		}
		builder.append("]");
	}

	private void appendPose3dArray(StringBuilder builder, Pose3d[] poses) {
		if (poses == null) {
			builder.append("[]");
			return;
		}

		builder.append("[");
		for (int i = 0; i < poses.length; i++) {
			if (i > 0) {
				builder.append(",");
			}
			appendPose3d(builder, null, poses[i]);
		}
		builder.append("]");
	}

	private void appendPose3d(StringBuilder builder, String name, Pose3d pose) {
		builder.append("{");
		if (name != null) {
			appendStringField(builder, "name", name);
		}
		appendNumberField(builder, "x", pose.getX());
		appendNumberField(builder, "y", pose.getY());
		appendNumberField(builder, "z", pose.getZ());
		appendNumberField(builder, "roll", pose.getRotation().getX());
		appendNumberField(builder, "pitch", pose.getRotation().getY());
		appendNumberField(builder, "yaw", pose.getRotation().getZ(), false);
		builder.append("}");
	}

	private void appendStringField(StringBuilder builder, String name, String value) {
		builder.append("\"").append(name).append("\":\"").append(value.replace("\\", "\\\\").replace("\"", "\\\""))
				.append("\",");
	}

	private void appendNumberField(StringBuilder builder, String name, double value) {
		appendNumberField(builder, name, value, true);
	}

	private void appendNumberField(StringBuilder builder, String name, double value, boolean comma) {
		builder.append("\"").append(name).append("\":").append(String.format(Locale.ROOT, "%.6f", value));
		if (comma) {
			builder.append(",");
		}
	}

	private void appendIntField(StringBuilder builder, String name, int value) {
		appendIntField(builder, name, value, true);
	}

	private void appendIntField(StringBuilder builder, String name, int value, boolean comma) {
		builder.append("\"").append(name).append("\":").append(value);
		if (comma) {
			builder.append(",");
		}
	}
}
