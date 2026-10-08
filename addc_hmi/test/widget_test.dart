import 'package:flutter_test/flutter_test.dart';
import 'package:provider/provider.dart';
import 'package:addc_hmi/main.dart';
import 'package:addc_hmi/providers/drone_state_provider.dart';

void main() {
  testWidgets('Settings screen loads correctly', (WidgetTester tester) async {
    // Build our app and trigger a frame.
    await tester.pumpWidget(
      MultiProvider(
        providers: [
          ChangeNotifierProvider(create: (_) => DroneStateProvider()),
        ],
        child: const AddcHmiApp(),
      ),
    );

    // Verify that the IP text field is present
    expect(find.text('Tailscale IP:Port'), findsOneWidget);
    
    // Verify that the connect button is present
    expect(find.text('INITIALIZE CONNECTION'), findsOneWidget);
  });
}
