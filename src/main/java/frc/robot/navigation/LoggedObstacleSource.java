package frc.robot.navigation;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Logs complete detector snapshots so replay never consults a live sensor. */
public final class LoggedObstacleSource implements ObstacleSource {
	private final ObstacleSource source;
	private final Inputs inputs = new Inputs();
	public LoggedObstacleSource(ObstacleSource source) {
		this.source = source;
	}
	@Override
	public List<Observation> observations(double now) {
		if (!Logger.hasReplaySource())
			inputs.values = source.observations(now);
		Logger.processInputs("Hybrid/Obstacles", inputs);
		return inputs.values;
	}
	static final class Inputs implements LoggableInputs {
		List<Observation> values = List.of();
		@Override
		public void toLog(LogTable table) {
			boolean valid = values != null && values.size() <= 64 && values.stream()
					.allMatch(o -> o != null && o.min() != null && o.max() != null && o.velocity() != null);
			table.put("SnapshotAvailable", valid);
			double[] flat = new double[valid ? values.size() * 8 : 0];
			if (valid)
				for (int i = 0; i < values.size(); i++) {
					var o = values.get(i);
					int j = i * 8;
					flat[j] = o.min().getX();
					flat[j + 1] = o.min().getY();
					flat[j + 2] = o.max().getX();
					flat[j + 3] = o.max().getY();
					flat[j + 4] = o.timestamp();
					flat[j + 5] = o.confidence();
					flat[j + 6] = o.velocity().getX();
					flat[j + 7] = o.velocity().getY();
				}
			table.put("Observations", flat);
		}
		@Override
		public void fromLog(LogTable table) {
			var flat = table.get("Observations", new double[0]);
			if (!table.get("SnapshotAvailable", false) || flat.length % 8 != 0 || flat.length > 512) {
				values = null;
				return;
			}
			var result = new ArrayList<Observation>();
			for (int j = 0; j < flat.length; j += 8)
				result.add(new Observation(new Translation2d(flat[j], flat[j + 1]),
						new Translation2d(flat[j + 2], flat[j + 3]), flat[j + 4], flat[j + 5],
						new Translation2d(flat[j + 6], flat[j + 7])));
			values = List.copyOf(result);
		}
	}
}
