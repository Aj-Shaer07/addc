import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../providers/drone_state_provider.dart';

class SettingsScreen extends StatefulWidget {
  const SettingsScreen({super.key});

  @override
  State<SettingsScreen> createState() => _SettingsScreenState();
}

class _SettingsScreenState extends State<SettingsScreen> {
  final _ipController = TextEditingController();

  @override
  void initState() {
    super.initState();
    final provider = context.read<DroneStateProvider>();
    _ipController.text = provider.ipAddress;
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('ADDC HMI Setup')),
      body: Padding(
        padding: const EdgeInsets.all(24.0),
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            const Icon(Icons.flight_takeoff, size: 80, color: Colors.cyanAccent),
            const SizedBox(height: 32),
            TextField(
              controller: _ipController,
              decoration: const InputDecoration(
                labelText: 'Tailscale IP:Port',
                border: OutlineInputBorder(),
                hintText: '100.x.y.z:5000',
              ),
            ),
            const SizedBox(height: 24),
            ElevatedButton(
              style: ElevatedButton.styleFrom(
                minimumSize: const Size.fromHeight(50),
              ),
              onPressed: () {
                context.read<DroneStateProvider>().saveIpAndConnect(_ipController.text);
                Navigator.pushReplacementNamed(context, '/dashboard');
              },
              child: const Text('Connect to UAV'),
            ),
          ],
        ),
      ),
    );
  }
}
