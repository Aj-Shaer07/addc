import 'dart:convert';
import 'package:http/http.dart' as http;

class HmiApiService {
  final String baseUrl;

  HmiApiService(this.baseUrl);

  Future<Map<String, dynamic>> fetchStatus() async {
    try {
      final response = await http.get(Uri.parse('http://$baseUrl/status'));
      if (response.statusCode == 200) {
        return json.decode(response.body);
      }
    } catch (e) {
      throw Exception('Failed to connect to drone: $e');
    }
    throw Exception('Failed to fetch status');
  }

  Future<bool> sendRoi(List<double> roi) async {
    try {
      final response = await http.post(
        Uri.parse('http://$baseUrl/set_roi'),
        headers: {'Content-Type': 'application/json'},
        body: json.encode({'roi': roi}),
      );
      return response.statusCode == 200;
    } catch (e) {
      return false;
    }
  }
}
