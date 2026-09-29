import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../providers/drone_state_provider.dart';

class DashboardScreen extends StatelessWidget {
  const DashboardScreen({super.key});

  @override
  Widget build(BuildContext context) {
    final provider = context.watch<DroneStateProvider>();

    return Scaffold(
      appBar: AppBar(
        title: const Text('Operative Dashboard'),
        actions: [
          Icon(
            provider.isConnected ? Icons.wifi : Icons.wifi_off,
            color: provider.isConnected ? Colors.green : Colors.red,
          ),
          const SizedBox(width: 16),
        ],
      ),
      body: SingleChildScrollView(
        padding: const EdgeInsets.all(16.0),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: [
            _buildStatusCard(provider),
            const SizedBox(height: 24),
            const Text(
              'Dispatch Priority Sector',
              style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
              textAlign: TextAlign.center,
            ),
            const SizedBox(height: 16),
            _buildSectorGrid(context, provider),
            const SizedBox(height: 24),
            _buildActionLogs(provider),
          ],
        ),
      ),
    );
  }

  Widget _buildStatusCard(DroneStateProvider provider) {
    return Card(
      elevation: 4,
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
      child: Padding(
        padding: const EdgeInsets.all(20.0),
        child: Column(
          children: [
            const Text('DECODED CACHE ACCESS CODE', style: TextStyle(color: Colors.grey)),
            const SizedBox(height: 8),
            Text(
              provider.decodedDigits,
              style: const TextStyle(
                fontSize: 56, 
                fontWeight: FontWeight.bold, 
                color: Colors.greenAccent,
                letterSpacing: 8,
              ),
            ),
            const SizedBox(height: 8),
            Text('Drone Status: ${provider.droneStatus}'),
          ],
        ),
      ),
    );
  }

  Widget _buildSectorGrid(BuildContext context, DroneStateProvider provider) {
    return GridView.count(
      crossAxisCount: 2,
      shrinkWrap: true,
      physics: const NeverScrollableScrollPhysics(),
      crossAxisSpacing: 12,
      mainAxisSpacing: 12,
      childAspectRatio: 1.5,
      children: [
        _SectorButton(
          title: 'SECTOR 1 (NW)',
          subtitle: 'X[8-14], Y[0-6]',
          onTap: () => provider.dispatchRoi([8, 14, 0, 6], 'Sector 1 (NW)'),
        ),
        _SectorButton(
          title: 'SECTOR 2 (NE)',
          subtitle: 'X[14-20], Y[0-6]',
          onTap: () => provider.dispatchRoi([14, 20, 0, 6], 'Sector 2 (NE)'),
        ),
        _SectorButton(
          title: 'SECTOR 3 (SW)',
          subtitle: 'X[8-14], Y[-6-0]',
          onTap: () => provider.dispatchRoi([8, 14, -6, 0], 'Sector 3 (SW)'),
        ),
        _SectorButton(
          title: 'SECTOR 4 (SE)',
          subtitle: 'X[14-20], Y[-6-0]',
          onTap: () => provider.dispatchRoi([14, 20, -6, 0], 'Sector 4 (SE)'),
        ),
      ],
    );
  }

  Widget _buildActionLogs(DroneStateProvider provider) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.stretch,
      children: provider.actionLogs.map((log) {
        return Padding(
          padding: const EdgeInsets.symmetric(vertical: 4.0),
          child: Text(
            log,
            style: TextStyle(
              color: log.contains('❌') ? Colors.redAccent : Colors.orangeAccent,
              fontSize: 14,
            ),
            textAlign: TextAlign.center,
          ),
        );
      }).toList(),
    );
  }
}

class _SectorButton extends StatelessWidget {
  final String title;
  final String subtitle;
  final VoidCallback onTap;

  const _SectorButton({
    required this.title,
    required this.subtitle,
    required this.onTap,
  });

  @override
  Widget build(BuildContext context) {
    return ElevatedButton(
      style: ElevatedButton.styleFrom(
        backgroundColor: Colors.blueGrey[900],
        shape: RoundedRectangleBorder(
          borderRadius: BorderRadius.circular(10),
          side: const BorderSide(color: Colors.cyanAccent),
        ),
      ),
      onPressed: onTap,
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          Text(title, style: const TextStyle(fontWeight: FontWeight.bold, color: Colors.white)),
          const SizedBox(height: 4),
          Text(subtitle, style: const TextStyle(fontSize: 10, color: Colors.grey)),
        ],
      ),
    );
  }
}
