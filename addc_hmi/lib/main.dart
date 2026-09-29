import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'providers/drone_state_provider.dart';
import 'ui/settings_screen.dart';
import 'ui/dashboard_screen.dart';

void main() {
  runApp(
    MultiProvider(
      providers: [
        ChangeNotifierProvider(create: (_) => DroneStateProvider()),
      ],
      child: const AddcHmiApp(),
    ),
  );
}

class AddcHmiApp extends StatelessWidget {
  const AddcHmiApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'ADDC HMI',
      theme: ThemeData.dark().copyWith(
        primaryColor: Colors.cyanAccent,
        scaffoldBackgroundColor: const Color(0xFF121212),
        colorScheme: const ColorScheme.dark(
          primary: Colors.cyanAccent,
          secondary: Colors.cyan,
        ),
      ),
      initialRoute: '/',
      routes: {
        '/': (context) => const SettingsScreen(),
        '/dashboard': (context) => const DashboardScreen(),
      },
    );
  }
}
