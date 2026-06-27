import 'dart:async';

import 'package:flutter/foundation.dart';
import 'package:nt4/nt4.dart';

class DashboardState extends ChangeNotifier {
  DashboardState() {
    _client = NT4Client(
      serverBaseAddress: robotAddress,
      onConnect: () {
        connected = true;
        notifyListeners();
      },
      onDisconnect: () {
        connected = false;
        notifyListeners();
      },
    );
    _publishedTopics = <String, NT4Topic>{
      _clickedPoseTopic: _client.publishNewTopic(_clickedPoseTopic, NT4TypeStr.typeFloat64Arr),
      _requestIdTopic: _client.publishNewTopic(_requestIdTopic, NT4TypeStr.typeInt),
      _killRequestIdTopic:
          _client.publishNewTopic(_killRequestIdTopic, NT4TypeStr.typeInt),
      _autoChooserSelectedTopic:
          _client.publishNewTopic(_autoChooserSelectedTopic, NT4TypeStr.typeStr),
    };
    _client.setProperties(_publishedTopics[_clickedPoseTopic]!, false, true);
    _client.setProperties(_publishedTopics[_requestIdTopic]!, false, true);
    _client.setProperties(_publishedTopics[_killRequestIdTopic]!, false, true);
    _client.setProperties(
        _publishedTopics[_autoChooserSelectedTopic]!, false, true);

    _subscribe('/SmartDashboard/Match Time', 0.25, (value) {
      matchTime = value is String ? value : '--:--.-';
      notifyListeners();
    });
    _subscribe('/SmartDashboard/AutoDrive/Is Red Alliance', 0.5, (value) {
      isRedAlliance = value == true;
      notifyListeners();
    });
    _subscribe('/SmartDashboard/AutoDrive/Enabled', 0.5, (value) {
      robotEnabled = value == true;
      notifyListeners();
    });
    _subscribe('/SmartDashboard/AutoDrive/Mode', 0.5, (value) {
      robotMode = value is String ? value : 'Unknown';
      notifyListeners();
    });
    _subscribe('/SmartDashboard/AutoDrive/Active', 0.25, (value) {
      autoDriveActive = value == true;
      notifyListeners();
    });
    _subscribe('/SmartDashboard/AutoDrive/LastAcceptedRequestId', 0.5,
        (value) {
      lastAcceptedRequestId = _numberValue(value).round();
      notifyListeners();
    });
    _subscribe('/SmartDashboard/AutoDrive/LastRejectedReason', 0.5, (value) {
      lastRejectedReason = value is String ? value : '';
      notifyListeners();
    });
    _subscribe('/SmartDashboard/AutoDrive/LastKillRequestId', 0.5, (value) {
      lastKillRequestId = _numberValue(value).round();
      notifyListeners();
    });
    _subscribe('/SmartDashboard/Auto Choices/options', 1.0, (value) {
      autoOptions = _stringListValue(value);
      if (selectedAutoName.isEmpty && autoOptions.isNotEmpty) {
        selectedAutoName = autoOptions.first;
      }
      notifyListeners();
    });
    _subscribe('/SmartDashboard/Auto Choices/active', 0.5, (value) {
      activeAutoName = value is String ? value : '';
      if (selectedAutoName.isEmpty && activeAutoName.isNotEmpty) {
        selectedAutoName = activeAutoName;
      }
      notifyListeners();
    });
    _subscribe('/SmartDashboard/DriverDashboard/Game Piece Camera Connected',
        0.25, (value) {
      gamePieceCameraConnected = value == true;
      notifyListeners();
    });
    _subscribe('/SmartDashboard/DriverDashboard/Sees Balls', 0.25, (value) {
      seesBalls = value == true;
      notifyListeners();
    });
    _subscribe('/SmartDashboard/DriverDashboard/Detected Groups', 0.25,
        (value) {
      detectedGroups = _numberValue(value).round();
      notifyListeners();
    });
    _subscribe('/SmartDashboard/DriverDashboard/Selected Ball Count', 0.25,
        (value) {
      selectedBallCount = _numberValue(value).round();
      notifyListeners();
    });
    _subscribe('/SmartDashboard/DriverDashboard/Selected Distance Meters',
        0.25, (value) {
      selectedDistanceMeters = _numberValue(value);
      notifyListeners();
    });
    _subscribe('/SmartDashboard/DriverDashboard/Selected Yaw Degrees', 0.25,
        (value) {
      selectedYawDegrees = _numberValue(value);
      notifyListeners();
    });
    _subscribe('/SmartDashboard/DriverDashboard/Selected Kept Balls', 0.25,
        (value) {
      selectedKeptBalls = _numberValue(value);
      notifyListeners();
    });
    _subscribe('/SmartDashboard/DriverDashboard/Selected Shape', 0.5, (value) {
      selectedShape = value is String ? value : '';
      notifyListeners();
    });
    _subscribe('/SmartDashboard/DriverDashboard/Selected Reason', 0.5,
        (value) {
      selectedReason = value is String ? value : '';
      notifyListeners();
    });
    _subscribe('/SmartDashboard/DriverDashboard/Biggest Ball Count', 0.25,
        (value) {
      biggestBallCount = _numberValue(value).round();
      notifyListeners();
    });
    _subscribe('/SmartDashboard/DriverDashboard/Biggest Distance Meters', 0.25,
        (value) {
      biggestDistanceMeters = _numberValue(value);
      notifyListeners();
    });
    _subscribe('/SmartDashboard/DriverDashboard/Closest Ball Count', 0.25,
        (value) {
      closestBallCount = _numberValue(value).round();
      notifyListeners();
    });
    _subscribe('/SmartDashboard/DriverDashboard/Closest Distance Meters', 0.25,
        (value) {
      closestDistanceMeters = _numberValue(value);
      notifyListeners();
    });
    _subscribe('/SmartDashboard/DriverDashboard/Front Camera Video URL', 1.0,
        (value) {
      frontCameraVideoUrl = value is String ? value : frontCameraVideoUrl;
      notifyListeners();
    });
    _subscribe('/SmartDashboard/DriverDashboard/Rear Camera Video URL', 1.0,
        (value) {
      rearCameraVideoUrl = value is String ? value : rearCameraVideoUrl;
      notifyListeners();
    });
    _subscribe('/SmartDashboard/DriverDashboard/Front Camera Dashboard URL',
        1.0, (value) {
      frontCameraDashboardUrl =
          value is String ? value : frontCameraDashboardUrl;
      notifyListeners();
    });
    _subscribe('/SmartDashboard/DriverDashboard/Rear Camera Dashboard URL',
        1.0, (value) {
      rearCameraDashboardUrl = value is String ? value : rearCameraDashboardUrl;
      notifyListeners();
    });

    _sendAll();
    Timer.periodic(const Duration(seconds: 1), (_) {
      if (connected) {
        _sendAll();
      }
    });
  }

  static const String robotAddress = String.fromEnvironment(
    'ROBOT_ADDRESS',
    defaultValue: kDebugMode ? '127.0.0.1' : '10.83.24.2',
  );
  static const String _clickedPoseTopic = '/AutoDrive/ClickedPose';
  static const String _requestIdTopic = '/AutoDrive/RequestId';
  static const String _killRequestIdTopic = '/AutoDrive/KillRequestId';
  static const String _autoChooserSelectedTopic =
      '/SmartDashboard/Auto Choices/selected';

  late final NT4Client _client;
  late final Map<String, NT4Topic> _publishedTopics;
  final List<StreamSubscription<dynamic>> _streamSubscriptions = <StreamSubscription<dynamic>>[];

  bool connected = false;
  bool isRedAlliance = false;
  bool robotEnabled = false;
  bool autoDriveActive = false;
  String robotMode = 'Unknown';
  String matchTime = '--:--.-';
  List<String> autoOptions = const <String>[];
  String selectedAutoName = '';
  String activeAutoName = '';
  int lastAcceptedRequestId = -1;
  int lastKillRequestId = -1;
  String lastRejectedReason = '';
  bool gamePieceCameraConnected = false;
  bool seesBalls = false;
  int detectedGroups = 0;
  int selectedBallCount = 0;
  int biggestBallCount = 0;
  int closestBallCount = 0;
  double selectedDistanceMeters = 0.0;
  double selectedYawDegrees = 0.0;
  double selectedKeptBalls = 0.0;
  double biggestDistanceMeters = 0.0;
  double closestDistanceMeters = 0.0;
  String selectedShape = '';
  String selectedReason = '';
  String frontCameraVideoUrl = 'http://10.83.24.11:5800/video';
  String rearCameraVideoUrl = 'http://10.83.24.12:5800/video';
  String frontCameraDashboardUrl = 'http://10.83.24.11:5800/';
  String rearCameraDashboardUrl = 'http://10.83.24.12:5800/';
  int _requestId = 0;
  int _killRequestId = 0;
  List<double> _lastClickedPose = const <double>[0.0, 0.0, 0.0];

  void driveToPose(double xMeters, double yMeters, double headingDegrees) {
    _requestId += 1;
    _lastClickedPose = <double>[xMeters, yMeters, headingDegrees];
    _sendAll();
    notifyListeners();
  }

  void killRobot() {
    _killRequestId += 1;
    _sendAll();
    notifyListeners();
  }

  void selectAuto(String autoName) {
    selectedAutoName = autoName;
    _client.addSample(_publishedTopics[_autoChooserSelectedTopic]!, autoName);
    notifyListeners();
  }

  void _sendAll() {
    _client.addSample(_publishedTopics[_clickedPoseTopic]!, _lastClickedPose);
    _client.addSample(_publishedTopics[_requestIdTopic]!, _requestId);
    _client.addSample(_publishedTopics[_killRequestIdTopic]!, _killRequestId);
    if (selectedAutoName.isNotEmpty) {
      _client.addSample(
          _publishedTopics[_autoChooserSelectedTopic]!, selectedAutoName);
    }
  }

  void _subscribe(String topic, double periodSeconds, void Function(dynamic value) listener) {
    final subscription = _client.subscribePeriodic(topic, periodSeconds);
    _streamSubscriptions.add(subscription.stream(yieldAll: true).listen(listener));
  }

  double _numberValue(dynamic value) {
    return value is num ? value.toDouble() : 0.0;
  }

  List<String> _stringListValue(dynamic value) {
    if (value is List) {
      return value.whereType<String>().toList(growable: false);
    }

    return const <String>[];
  }

  @override
  void dispose() {
    for (final subscription in _streamSubscriptions) {
      subscription.cancel();
    }
    super.dispose();
  }
}
