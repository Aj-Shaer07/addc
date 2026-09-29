import 'dart:async';
import 'package:flutter/material.dart';
import 'package:shared_preferences/shared_preferences.dart';
import '../services/api_service.dart';

class DroneStateProvider extends ChangeNotifier {
  String _ipAddress = '';
  String get ipAddress => _ipAddress;

  bool _isConnected = false;
  bool get isConnected => _isConnected;

  String _decodedDigits = '--';
  String get decodedDigits => _decodedDigits;

  String _droneStatus = 'IDLE';
  String get droneStatus => _droneStatus;

  List<String> _actionLogs = [];
  List<String> get actionLogs => _actionLogs;

  HmiApiService? _apiService;
  Timer? _pollingTimer;

  DroneStateProvider() {
    _loadIpAddress();
  }

  Future<void> _loadIpAddress() async {
    final prefs = await SharedPreferences.getInstance();
    _ipAddress = prefs.getString('tailscale_ip') ?? '100.x.y.z:5000';
    notifyListeners();
  }

  Future<void> saveIpAndConnect(String ip) async {
    final prefs = await SharedPreferences.getInstance();
    await prefs.setString('tailscale_ip', ip);
    _ipAddress = ip;
    _apiService = HmiApiService(ip);
    
    _startPolling();
    notifyListeners();
  }

  void _startPolling() {
    _pollingTimer?.cancel();
    _pollingTimer = Timer.periodic(const Duration(seconds: 1), (_) async {
      if (_apiService == null) return;
      try {
        final status = await _apiService!.fetchStatus();
        _isConnected = true;
        _decodedDigits = status['decoded_digits'] ?? '--';
        _droneStatus = status['drone_status'] ?? 'IDLE';
        notifyListeners();
      } catch (e) {
        _isConnected = false;
        notifyListeners();
      }
    });
  }

  Future<void> dispatchRoi(List<double> roi, String name) async {
    if (_apiService == null) return;
    
    _logAction("Transmitting priority to UAV: $name...");
    final success = await _apiService!.sendRoi(roi);
    
    if (success) {
      _logAction("✓ UAV Acknowledged: Prioritizing $name");
    } else {
      _logAction("❌ Transmission failed. Check connection.");
    }
  }

  void _logAction(String msg) {
    _actionLogs.insert(0, msg);
    if (_actionLogs.length > 5) {
      _actionLogs.removeLast();
    }
    notifyListeners();
  }

  @override
  void dispose() {
    _pollingTimer?.cancel();
    super.dispose();
  }
}
