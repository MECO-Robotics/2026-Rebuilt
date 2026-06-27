import 'package:flutter/material.dart';

import 'pages/dashboard_page.dart';
import 'services/dashboard_state.dart';
import 'window_setup.dart';

Future<void> main() async {
  WidgetsFlutterBinding.ensureInitialized();
  await configureAppWindow();

  runApp(const DashboardApp());
}

class DashboardApp extends StatefulWidget {
  const DashboardApp({super.key});

  @override
  State<DashboardApp> createState() => _DashboardAppState();
}

class _DashboardAppState extends State<DashboardApp> {
  late final DashboardState dashboardState;

  @override
  void initState() {
    super.initState();
    dashboardState = DashboardState();
  }

  @override
  void dispose() {
    dashboardState.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      debugShowCheckedModeBanner: false,
      title: '8324 Dashboard',
      theme: ThemeData(
        brightness: Brightness.dark,
        colorScheme: ColorScheme.fromSeed(
          seedColor: const Color(0xfffa315f),
          brightness: Brightness.dark,
        ),
        fontFamily: 'Segoe UI',
        useMaterial3: true,
      ),
      home: DashboardPage(dashboardState: dashboardState),
    );
  }
}
