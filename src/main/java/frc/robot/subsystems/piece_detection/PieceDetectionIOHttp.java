package frc.robot.subsystems.piece_detection;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.constants.vision.PieceDetectionConstants;
import frc.robot.constants.vision.PieceDetectionConstants.PieceDetectionConfig;
import frc.robot.util.mechanical_advantage.LoggedTunableNumber;
import java.io.IOException;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.time.Duration;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

public class PieceDetectionIOHttp implements PieceDetectionIO {
	private static final double KEPT_BALL_RATIO = 60.0 / 70.0;
	private static final double DEFAULT_PICKUP_SECONDS = 1.0;
	private static final double DEFAULT_SHOOT_SECONDS = 2.0;

	private final String name;
	private final PieceDetectionConfig config;
	private final URI dataUri;
	private final HttpClient client = HttpClient.newBuilder().connectTimeout(Duration.ofMillis(150)).build();
	private final JSONParser parser = new JSONParser();
	private final InterpolatingDoubleTreeMap areaRangeMap = new InterpolatingDoubleTreeMap();
	private double minCalibratedArea = Double.POSITIVE_INFINITY;
	private double maxCalibratedArea = Double.NEGATIVE_INFINITY;
	private final LoggedTunableNumber kDistance;
	private final LoggedTunableNumber cruiseSpeedMetersPerSecond;
	private final LoggedTunableNumber pickupSeconds;
	private final LoggedTunableNumber shootSeconds;
	private final Alert cameraDisconnected;

	private boolean connected = false;

	public PieceDetectionIOHttp(String name, PieceDetectionConfig config, URI dataUri, double kDistance) {
		this.name = name;
		this.config = config;
		this.dataUri = dataUri;
		for (PieceDetectionConstants.AreaRangeSample sample : PieceDetectionConstants.GAME_PIECE_AREA_RANGE_SAMPLES) {
			areaRangeMap.put(sample.areaPixels(), sample.rangeMeters());
			minCalibratedArea = Math.min(minCalibratedArea, sample.areaPixels());
			maxCalibratedArea = Math.max(maxCalibratedArea, sample.areaPixels());
		}
		this.kDistance = new LoggedTunableNumber(name + "/Gains/kDistance", kDistance);
		cruiseSpeedMetersPerSecond = new LoggedTunableNumber(name + "/Autonomy/CruiseSpeedMps", 2.0);
		pickupSeconds = new LoggedTunableNumber(name + "/Autonomy/PickupSeconds", DEFAULT_PICKUP_SECONDS);
		shootSeconds = new LoggedTunableNumber(name + "/Autonomy/ShootSeconds", DEFAULT_SHOOT_SECONDS);
		cameraDisconnected = new Alert(name, name + " HTTP game-piece camera disconnected!", AlertType.kWarning);
	}

	@Override
	public void updateInputs(PieceDetectionIOInputs inputs) {
		try {
			HttpRequest request = HttpRequest.newBuilder(dataUri).timeout(Duration.ofMillis(200)).GET().build();
			HttpResponse<String> response = client.send(request, HttpResponse.BodyHandlers.ofString());

			if (response.statusCode() != 200) {
				connected = false;
				clearTarget(inputs);
				return;
			}

			JSONObject root = (JSONObject) parser.parse(response.body());
			List<Group> groups = parseGroups(root);
			connected = true;
			inputs.connected = true;
			inputs.groupCount = groups.size();

			if (groups.isEmpty()) {
				clearTarget(inputs);
				return;
			}

			Group biggest = groups.stream().max(Comparator.comparingInt(Group::count).thenComparingDouble(Group::area))
					.orElse(groups.get(0));
			Group closest = groups.stream().min(Comparator.comparingDouble(Group::distanceMeters))
					.orElse(groups.get(0));
			Selection selection = selectGroup(biggest, closest);

			writeGroupInputs(inputs, biggest, closest, selection);
		} catch (IOException | InterruptedException | ParseException | ClassCastException ex) {
			if (ex instanceof InterruptedException) {
				Thread.currentThread().interrupt();
			}
			connected = false;
			clearTarget(inputs);
		} finally {
			inputs.connected = connected;
			cameraDisconnected.set(!connected);
		}
	}

	private List<Group> parseGroups(JSONObject root) {
		List<Group> groups = new ArrayList<>();
		Object rawGroups = root.get("groups");
		if (!(rawGroups instanceof JSONArray groupArray)) {
			return groups;
		}

		for (Object rawGroup : groupArray) {
			if (!(rawGroup instanceof JSONObject groupJson)) {
				continue;
			}

			int id = number(groupJson.get("id")).intValue();
			int count = number(groupJson.get("count")).intValue();
			double centerX = number(groupJson.get("center_x")).doubleValue();
			double centerY = number(groupJson.get("center_y")).doubleValue();
			double yawDeg = number(groupJson.get("yaw_deg")).doubleValue();
			double area = number(groupJson.get("area")).doubleValue();
			double distanceMeters = distanceMeters(groupJson.get("avg_distance_in"), area);
			String shape = stringValue(groupJson.get("shape"), "circle");

			if (count > 0 && distanceMeters > 0.0) {
				groups.add(new Group(id, count, centerX, centerY, yawDeg, area, distanceMeters, shape));
			}
		}

		return groups;
	}

	private Number number(Object value) {
		return value instanceof Number ? (Number) value : 0.0;
	}

	private String stringValue(Object value, String fallback) {
		if (value instanceof String string && !string.isBlank()) {
			return string;
		}

		return fallback;
	}

	private double distanceMeters(Object avgDistanceIn, double area) {
		if (hasAreaRangeCalibration() && area > 0.0) {
			double clampedArea = Math.max(minCalibratedArea, Math.min(maxCalibratedArea, area));
			return areaRangeMap.get(clampedArea);
		}

		if (avgDistanceIn instanceof Number distanceIn && distanceIn.doubleValue() > 0.0) {
			return Units.inchesToMeters(distanceIn.doubleValue());
		}

		if (area <= 0.0) {
			return 0.0;
		}

		return kDistance.get() / Math.sqrt(area);
	}

	private boolean hasAreaRangeCalibration() {
		return PieceDetectionConstants.GAME_PIECE_AREA_RANGE_SAMPLES.length >= 2;
	}

	private Selection selectGroup(Group biggest, Group closest) {
		double cruiseSpeed = Math.max(0.1, cruiseSpeedMetersPerSecond.get());
		double bigTravelTime = biggest.distanceMeters() / cruiseSpeed;
		double closeCycleTime = cycleTimeSeconds(closest, cruiseSpeed);
		double closeTripsDuringBigTrip = Math.floor(bigTravelTime / closeCycleTime);
		double closeBallsDuringBigTrip = closeTripsDuringBigTrip * keptBalls(closest.count());
		double bigKeptBalls = keptBalls(biggest.count());

		if (closest.id() != biggest.id() && closeTripsDuringBigTrip > 1.0 && closeBallsDuringBigTrip > bigKeptBalls) {
			return new Selection(closest, closeScore(closest, cruiseSpeed), closeTripsDuringBigTrip,
					"closest group can outscore biggest during big-group travel time");
		}

		double biggestScore = closeScore(biggest, cruiseSpeed);
		double closestScore = closeScore(closest, cruiseSpeed);

		if (closest.id() != biggest.id() && closestScore > biggestScore) {
			return new Selection(closest, closestScore, closeTripsDuringBigTrip, "closest group has better balls/sec");
		}

		return new Selection(biggest, biggestScore, closeTripsDuringBigTrip, "biggest group has better expected yield");
	}

	private double cycleTimeSeconds(Group group, double cruiseSpeed) {
		double oneWayTravelTime = group.distanceMeters() / cruiseSpeed;
		return oneWayTravelTime + pickupSeconds.get() + shootSeconds.get();
	}

	private double closeScore(Group group, double cruiseSpeed) {
		return keptBalls(group.count()) / cycleTimeSeconds(group, cruiseSpeed);
	}

	private double keptBalls(int rawBalls) {
		return rawBalls * KEPT_BALL_RATIO;
	}

	private void writeGroupInputs(PieceDetectionIOInputs inputs, Group biggest, Group closest, Selection selection) {
		Group selected = selection.group();

		inputs.biggestGroupBallCount = biggest.count();
		inputs.closestGroupBallCount = closest.count();
		inputs.selectedGroupBallCount = selected.count();
		inputs.biggestGroupDistance = biggest.distanceMeters();
		inputs.closestGroupDistance = closest.distanceMeters();
		inputs.selectedGroupDistance = selected.distanceMeters();
		inputs.selectedGroupYaw = selected.yawDeg();
		inputs.selectedGroupKeptBalls = keptBalls(selected.count());
		inputs.closestGroupTripsDuringBigTrip = selection.closestTripsDuringBigTrip();
		inputs.selectedGroupScore = selection.score();
		inputs.selectedGroupShape = selected.shape();
		inputs.selectedGroupReason = selection.reason();

		inputs.yaw = selected.yawDeg();
		inputs.pitch = 0.0;
		inputs.area = selected.area();
		inputs.distance = selected.distanceMeters();
		double yawRadians = Math.toRadians(selected.yawDeg());
		inputs.robotToPieceTransform = config.robotToCameraTransform()
				.plus(new Transform3d(new Translation3d(selected.distanceMeters() * Math.cos(yawRadians),
						selected.distanceMeters() * Math.sin(yawRadians), 0.0), new Rotation3d()));
		inputs.seesTarget = true;
	}

	private void clearTarget(PieceDetectionIOInputs inputs) {
		inputs.seesTarget = false;
		inputs.yaw = 0.0;
		inputs.pitch = 0.0;
		inputs.area = 0.0;
		inputs.distance = 0.0;
		inputs.robotToPieceTransform = new Transform3d();
		inputs.groupCount = 0;
		inputs.biggestGroupBallCount = 0;
		inputs.closestGroupBallCount = 0;
		inputs.selectedGroupBallCount = 0;
		inputs.biggestGroupDistance = 0.0;
		inputs.closestGroupDistance = 0.0;
		inputs.selectedGroupDistance = 0.0;
		inputs.selectedGroupYaw = 0.0;
		inputs.selectedGroupKeptBalls = 0.0;
		inputs.closestGroupTripsDuringBigTrip = 0.0;
		inputs.selectedGroupScore = 0.0;
		inputs.selectedGroupShape = "";
		inputs.selectedGroupReason = "";
	}

	@Override
	public String getName() {
		return name;
	}

	private record Group(int id, int count, double centerX, double centerY, double yawDeg, double area,
			double distanceMeters, String shape) {
	}

	private record Selection(Group group, double score, double closestTripsDuringBigTrip, String reason) {
	}
}
