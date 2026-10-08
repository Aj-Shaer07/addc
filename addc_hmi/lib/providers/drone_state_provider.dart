import 'dart:async';
import 'package:flutter/material.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:geolocator/geolocator.dart';
import '../services/api_service.dart';

class Sector {
  final String name;
  final String description;
  final List<double> roi;

  Sector({required this.name, required this.description, required this.roi});
}

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

  String _runnerDigits = '';
  String get runnerDigits => _runnerDigits;
  
  bool _droneDigitsFirst = true;
  bool get droneDigitsFirst => _droneDigitsFirst;

  Position? _runnerPosition;
  Position? get runnerPosition => _runnerPosition;
  StreamSubscription<Position>? _positionStream;
  String _currentSector = 'Unknown';
  String get currentSector => _currentSector;

  List<Sector> _sectors = [
    Sector(name: 'Sector 1 (NW)', description: 'X[8-14], Y[0-6]', roi: [8, 14, 0, 6]),
    Sector(name: 'Sector 2 (NE)', description: 'X[14-20], Y[0-6]', roi: [14, 20, 0, 6]),
    Sector(name: 'Sector 3 (SW)', description: 'X[8-14], Y[-6-0]', roi: [8, 14, -6, 0]),
    Sector(name: 'Sector 4 (SE)', description: 'X[14-20], Y[-6-0]', roi: [14, 20, -6, 0]),
  ];
  List<Sector> get sectors => _sectors;

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

  Future<bool> saveIpAndConnect(String ip) async {
    final prefs = await SharedPreferences.getInstance();
    await prefs.setString('tailscale_ip', ip);
    _ipAddress = ip;
    _apiService = HmiApiService(ip);
    
    try {
      final status = await _apiService!.fetchStatus();
      _isConnected = true;
      _decodedDigits = status['decoded_digits'] ?? '--';
      _droneStatus = status['drone_status'] ?? 'IDLE';
      _startPolling();
      _startLocationTracking();
      notifyListeners();
      return true;
    } catch (e) {
      _isConnected = false;
      notifyListeners();
      return false;
    }
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

  void _startLocationTracking() async {
    bool serviceEnabled;
    LocationPermission permission;

    serviceEnabled = await Geolocator.isLocationServiceEnabled();
    if (!serviceEnabled) return;

    permission = await Geolocator.checkPermission();
    if (permission == LocationPermission.denied) {
      permission = await Geolocator.requestPermission();
      if (permission == LocationPermission.denied) return;
    }
    
    if (permission == LocationPermission.deniedForever) return;

    _positionStream = Geolocator.getPositionStream(
      locationSettings: const LocationSettings(
        accuracy: LocationAccuracy.high,
        distanceFilter: 10,
      )
    ).listen((Position position) {
      _runnerPosition = position;
      _updateCurrentSector();
      notifyListeners();
    });
  }
  
  void _updateCurrentSector() {
    if (_runnerPosition == null) return;
    _currentSector = 'Tracking (${_runnerPosition!.latitude.toStringAsFixed(4)}, ${_runnerPosition!.longitude.toStringAsFixed(4)})';
  }

  void setRunnerDigits(String digits) {
    if (digits.length <= 2) {
      _runnerDigits = digits;
      notifyListeners();
    }
  }
  
  void toggleDigitOrder() {
    _droneDigitsFirst = !_droneDigitsFirst;
    notifyListeners();
  }
  
  String get combinedCode {
    String d = (_decodedDigits == '--' || _decodedDigits.isEmpty) ? '__' : _decodedDigits;
    String r = _runnerDigits.padRight(2, '_');
    return _droneDigitsFirst ? '$d$r' : '$r$d';
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
    _positionStream?.cancel();
    super.dispose();
  }
}
