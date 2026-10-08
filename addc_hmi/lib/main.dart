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
      debugShowCheckedModeBanner: false,
      theme: ThemeData(
        brightness: Brightness.light,
        primaryColor: Colors.blueAccent,
        scaffoldBackgroundColor: const Color(0xFFF3F4F6),
        colorScheme: const ColorScheme.light(
          primary: Colors.blueAccent,
          secondary: Colors.blue,
          surface: Colors.white,
        ),
        fontFamily: 'Roboto', // Default fallback that looks clean
      ),
      initialRoute: '/',
      routes: {
        '/': (context) => const SettingsScreen(),
        '/dashboard': (context) => const DashboardScreen(),
      },
    );
  }
}
