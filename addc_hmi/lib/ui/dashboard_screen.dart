import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../providers/drone_state_provider.dart';

class DashboardScreen extends StatelessWidget {
  const DashboardScreen({super.key});

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: const Color(0xFFF3F4F6),
      appBar: AppBar(
        backgroundColor: Colors.white,
        elevation: 1,
        title: const Text(
          'OPERATIVE DASHBOARD',
          style: TextStyle(color: Colors.black87, fontWeight: FontWeight.bold),
        ),
        actions: [
          Consumer<DroneStateProvider>(
            builder: (context, provider, child) {
              return Padding(
                padding: const EdgeInsets.symmetric(horizontal: 16.0),
                child: Center(
                  child: Container(
                    padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
                    decoration: BoxDecoration(
                      color: provider.isConnected ? Colors.green[50] : Colors.red[50],
                      borderRadius: BorderRadius.circular(20),
                      border: Border.all(
                        color: provider.isConnected ? Colors.green : Colors.red,
                      ),
                    ),
                    child: Row(
                      children: [
                        Icon(
                          provider.isConnected ? Icons.wifi : Icons.wifi_off,
                          color: provider.isConnected ? Colors.green : Colors.red,
                          size: 16,
                        ),
                        const SizedBox(width: 8),
                        Text(
                          provider.isConnected ? 'CONNECTED' : 'DISCONNECTED',
                          style: TextStyle(
                            color: provider.isConnected ? Colors.green : Colors.red,
                            fontSize: 12,
                            fontWeight: FontWeight.bold,
                          ),
                        ),
                      ],
                    ),
                  ),
                ),
              );
            },
          ),
        ],
      ),
      body: SingleChildScrollView(
        padding: const EdgeInsets.all(24.0),
        child: Consumer<DroneStateProvider>(
          builder: (context, provider, child) {
            return Column(
              crossAxisAlignment: CrossAxisAlignment.stretch,
              children: [
                _buildStatusCard(provider, context),
                const SizedBox(height: 24),
                _buildCodeCombiner(provider),
                const SizedBox(height: 24),
                _buildRunnerTracking(provider),
                const SizedBox(height: 24),
                _buildSectorGrid(provider),
                const SizedBox(height: 24),
                _buildActionLogs(provider),
              ],
            );
          },
        ),
      ),
    );
  }

  Widget _buildStatusCard(DroneStateProvider provider, BuildContext context) {
    return Container(
      padding: const EdgeInsets.all(24),
      decoration: BoxDecoration(
        color: Colors.white,
        borderRadius: BorderRadius.circular(16),
        boxShadow: [
          BoxShadow(
            color: Colors.black.withOpacity(0.05),
            blurRadius: 10,
            offset: const Offset(0, 5),
          ),
        ],
      ),
      child: Column(
        children: [
          const Text(
            'UAV STATUS',
            style: TextStyle(
              color: Colors.black54,
              fontWeight: FontWeight.bold,
              letterSpacing: 1.5,
            ),
          ),
          const SizedBox(height: 16),
          Text(
            provider.droneStatus.toUpperCase(),
            style: const TextStyle(
              fontSize: 32,
              fontWeight: FontWeight.w900,
              color: Colors.blueAccent,
              letterSpacing: 2,
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildCodeCombiner(DroneStateProvider provider) {
    return Container(
      padding: const EdgeInsets.all(24),
      decoration: BoxDecoration(
        color: Colors.white,
        borderRadius: BorderRadius.circular(16),
        boxShadow: [
          BoxShadow(
            color: Colors.black.withOpacity(0.05),
            blurRadius: 10,
            offset: const Offset(0, 5),
          ),
        ],
      ),
      child: Column(
        children: [
          const Text(
            'ACCESS CODE DECODER',
            style: TextStyle(
              color: Colors.black54,
              fontWeight: FontWeight.bold,
              letterSpacing: 1.5,
            ),
          ),
          const SizedBox(height: 24),
          Row(
            mainAxisAlignment: MainAxisAlignment.spaceEvenly,
            children: [
              _buildCodeInputBlock('DRONE', provider.decodedDigits),
              Column(
                children: [
                  IconButton(
                    icon: const Icon(Icons.swap_horiz, size: 32, color: Colors.blueAccent),
                    onPressed: provider.toggleDigitOrder,
                    tooltip: 'Swap order',
                  ),
                  Text(
                    provider.droneDigitsFirst ? 'D + R' : 'R + D',
                    style: const TextStyle(color: Colors.black54, fontWeight: FontWeight.bold),
                  ),
                ],
              ),
              _buildCodeEditableBlock('RUNNER', provider),
            ],
          ),
          const SizedBox(height: 24),
          const Divider(),
          const SizedBox(height: 16),
          const Text('FINAL 4-DIGIT CODE', style: TextStyle(color: Colors.black54, fontWeight: FontWeight.bold)),
          const SizedBox(height: 8),
          Text(
            provider.combinedCode,
            style: const TextStyle(
              fontSize: 48,
              fontWeight: FontWeight.w900,
              color: Colors.blueAccent,
              letterSpacing: 16,
              fontFamily: 'monospace',
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildCodeInputBlock(String title, String value) {
    return Column(
      children: [
        Text(title, style: const TextStyle(fontWeight: FontWeight.bold, color: Colors.black54)),
        const SizedBox(height: 8),
        Container(
          padding: const EdgeInsets.symmetric(horizontal: 24, vertical: 16),
          decoration: BoxDecoration(
            color: Colors.grey[100],
            borderRadius: BorderRadius.circular(12),
            border: Border.all(color: Colors.grey[300]!),
          ),
          child: Text(
            value.padRight(2, '-'),
            style: const TextStyle(fontSize: 32, fontWeight: FontWeight.bold, fontFamily: 'monospace'),
          ),
        ),
      ],
    );
  }

  Widget _buildCodeEditableBlock(String title, DroneStateProvider provider) {
    return Column(
      children: [
        Text(title, style: const TextStyle(fontWeight: FontWeight.bold, color: Colors.black54)),
        const SizedBox(height: 8),
        SizedBox(
          width: 100,
          child: TextField(
            textAlign: TextAlign.center,
            maxLength: 2,
            keyboardType: TextInputType.number,
            style: const TextStyle(fontSize: 32, fontWeight: FontWeight.bold, fontFamily: 'monospace'),
            decoration: InputDecoration(
              counterText: '',
              filled: true,
              fillColor: Colors.blue[50],
              border: OutlineInputBorder(
                borderRadius: BorderRadius.circular(12),
                borderSide: BorderSide.none,
              ),
            ),
            onChanged: provider.setRunnerDigits,
          ),
        ),
      ],
    );
  }

  Widget _buildRunnerTracking(DroneStateProvider provider) {
    return Container(
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: Colors.white,
        borderRadius: BorderRadius.circular(16),
      ),
      child: Row(
        children: [
          const Icon(Icons.gps_fixed, color: Colors.blueAccent),
          const SizedBox(width: 16),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                const Text('RUNNER LOCATION', style: TextStyle(fontWeight: FontWeight.bold, color: Colors.black54)),
                Text(provider.currentSector, style: const TextStyle(fontWeight: FontWeight.bold, fontSize: 16)),
              ],
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildSectorGrid(DroneStateProvider provider) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        const Text(
          'DISPATCH ROI',
          style: TextStyle(
            fontWeight: FontWeight.bold,
            color: Colors.black54,
            letterSpacing: 1.5,
          ),
        ),
        const SizedBox(height: 16),
        GridView.builder(
          shrinkWrap: true,
          physics: const NeverScrollableScrollPhysics(),
          gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(
            crossAxisCount: 2,
            crossAxisSpacing: 16,
            mainAxisSpacing: 16,
            childAspectRatio: 1.5,
          ),
          itemCount: provider.sectors.length,
          itemBuilder: (context, index) {
            final sector = provider.sectors[index];
            return _SectorButton(
              title: sector.name,
              subtitle: sector.description,
              onTap: () => provider.dispatchRoi(sector.roi, sector.name),
            );
          },
        ),
      ],
    );
  }

  Widget _buildActionLogs(DroneStateProvider provider) {
    if (provider.actionLogs.isEmpty) return const SizedBox.shrink();
    
    return Container(
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: Colors.white,
        borderRadius: BorderRadius.circular(16),
        border: Border.all(color: Colors.grey[200]!),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.stretch,
        children: [
          const Text(
            'ACTIVITY LOG',
            style: TextStyle(
              color: Colors.black54,
              fontWeight: FontWeight.bold,
              letterSpacing: 1.5,
            ),
          ),
          const SizedBox(height: 12),
          ...provider.actionLogs.map((log) {
            final isError = log.contains('❌');
            return Padding(
              padding: const EdgeInsets.symmetric(vertical: 4.0),
              child: Row(
                children: [
                  Icon(
                    isError ? Icons.error_outline : Icons.check_circle_outline,
                    size: 16,
                    color: isError ? Colors.red : Colors.green,
                  ),
                  const SizedBox(width: 8),
                  Expanded(
                    child: Text(
                      log,
                      style: TextStyle(color: Colors.grey[800], fontSize: 13),
                    ),
                  ),
                ],
              ),
            );
          }),
        ],
      ),
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
        backgroundColor: Colors.white,
        foregroundColor: Colors.blueAccent,
        elevation: 0,
        shape: RoundedRectangleBorder(
          borderRadius: BorderRadius.circular(12),
          side: BorderSide(color: Colors.grey[300]!),
        ),
      ),
      onPressed: onTap,
      child: Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: [
          Text(
            title,
            style: const TextStyle(fontWeight: FontWeight.bold, fontSize: 16),
          ),
          const SizedBox(height: 4),
          Text(
            subtitle,
            style: TextStyle(
              color: Colors.grey[600],
              fontSize: 12,
            ),
            textAlign: TextAlign.center,
          ),
        ],
      ),
    );
  }
}
