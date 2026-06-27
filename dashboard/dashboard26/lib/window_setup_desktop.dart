import 'package:flutter/material.dart';
import 'package:window_manager/window_manager.dart';

Future<void> configureAppWindow() async {
  await windowManager.ensureInitialized();
  await windowManager.waitUntilReadyToShow(
    const WindowOptions(
      size: Size(1310, 720),
      minimumSize: Size(1024, 576),
      center: true,
      title: 'Team 8324 Dashboard',
    ),
    () async {
      await windowManager.show();
      await windowManager.focus();
    },
  );
}
