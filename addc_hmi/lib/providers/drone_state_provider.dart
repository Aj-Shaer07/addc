import 'dart:async';
import 'package:flutter/material.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:geolocator/geolocator.dart';
import 'dart:math' as math;
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

  double _dronePosX = 0.0;
  double get dronePosX => _dronePosX;

  double _dronePosY = 0.0;
  double get dronePosY => _dronePosY;

  String get droneLocationString {
    String sectorName = "Unknown Sector";
    for (var s in _sectors) {
      if (_dronePosX >= s.roi[0] && _dronePosX <= s.roi[1] &&
          _dronePosY >= s.roi[2] && _dronePosY <= s.roi[3]) {
        sectorName = s.name;
        break;
      }
    }
    return "X: ${_dronePosX.toStringAsFixed(1)}, Y: ${_dronePosY.toStringAsFixed(1)} ($sectorName)";
  }

  List<String> _actionLogs = [];
  List<String> get actionLogs => _actionLogs;

  String _runnerDigits = '';
  String get runnerDigits => _runnerDigits;
  
  bool _droneDigitsFirst = true;
  bool get droneDigitsFirst => _droneDigitsFirst;

  Position? _runnerPosition;
  Position? get runnerPosition => _runnerPosition;
  StreamSubscription<Position>? _positionStream;

  double _homeLat = 0.0;
  double _homeLon = 0.0;

  double _operativePosX = 0.0;
  double _operativePosY = 0.0;

  String get operativeLocationString {
    if (_runnerPosition == null) return "Waiting for GPS...";
    if (_homeLat == 0.0 && _homeLon == 0.0) return "GPS Active (Waiting for Drone Home)";

    // Haversine Flat-Earth Approximation (Same as ROS 2 ENU)
    double r = 6378137.0;
    double dlat = (_runnerPosition!.latitude - _homeLat) * (math.pi / 180.0);
    double dlon = (_runnerPosition!.longitude - _homeLon) * (math.pi / 180.0);
    double refLatRad = _homeLat * (math.pi / 180.0);

    _operativePosX = dlon * r * math.cos(refLatRad);
    _operativePosY = dlat * r;

    String sectorName = "Outside Grid";
    for (var s in _sectors) {
      if (_operativePosX >= s.roi[0] && _operativePosX <= s.roi[1] &&
          _operativePosY >= s.roi[2] && _operativePosY <= s.roi[3]) {
        sectorName = s.name;
        break;
      }
    }
    return "X: ${_operativePosX.toStringAsFixed(1)}, Y: ${_operativePosY.toStringAsFixed(1)} ($sectorName)";
  }

  bool isOperativeInSector(String sectorName) {
    if (_runnerPosition == null || _homeLat == 0.0) return false;
    for (var s in _sectors) {
      if (s.name == sectorName) {
        return _operativePosX >= s.roi[0] && _operativePosX <= s.roi[1] &&
               _operativePosY >= s.roi[2] && _operativePosY <= s.roi[3];
      }
    }
    return false;
  }

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

  void _updateStatus(Map<String, dynamic> status) {
    _isConnected = true;
    _decodedDigits = status['decoded_digits'] ?? '--';
    _droneStatus = status['drone_status'] ?? 'IDLE';
    _dronePosX = (status['drone_pos_x'] ?? 0.0).toDouble();
    _dronePosY = (status['drone_pos_y'] ?? 0.0).toDouble();
    _homeLat = (status['home_lat'] ?? 0.0).toDouble();
    _homeLon = (status['home_lon'] ?? 0.0).toDouble();

    if (status['sectors'] != null) {
      List<dynamic> rawSectors = status['sectors'];
      _sectors = rawSectors.map((s) => Sector(
        name: s['name'],
        description: s['description'],
        roi: (s['roi'] as List).map((e) => (e as num).toDouble()).toList()
      )).toList();
    }
  }

  Future<bool> saveIpAndConnect(String ip) async {
    final prefs = await SharedPreferences.getInstance();
    await prefs.setString('tailscale_ip', ip);
    _ipAddress = ip;
    _apiService = HmiApiService(ip);
    
    try {
      final status = await _apiService!.fetchStatus();
      _updateStatus(status);
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
        _updateStatus(status);
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
