# SAEISS ADDC - Ground Operative HMI App

This is the Flutter-based Human-Machine Interface (HMI) application for the ground operatives (runners) participating in the SAEISS Autonomous Drone Development Challenge.

The app communicates over a Tailscale mesh VPN with the drone's companion computer (running `hmi_bridge_node.py`), allowing the operative to monitor the mission status, receive the decoded intelligence cache digits, and dispatch priority Regions of Interest (ROIs).

---

## 🎨 UI/UX Features
- **Modern Dark Theme**: Sleek glassmorphism and gradient design tailored for high-contrast visibility.
- **Micro-Animations**: Hover effects, press states, and glowing text for an immersive, tech-forward experience.
- **Real-Time Dashboard**: Monitor connection status and drone status simultaneously.

## 🛠 Prerequisites

Before you can run or build this app, ensure you have the following installed on your development machine:
1. **Flutter SDK**: [Install Flutter](https://docs.flutter.dev/get-started/install)
2. **Android Studio** (for Android builds/emulation) or **Xcode** (for iOS builds/simulation).

---

## 🚀 Step 1: Initial Setup

To generate the required platform folders and download dependencies, run the following commands in your terminal:

```bash
# 1. Navigate into the HMI app directory
cd addc_hmi

# 2. Generate the native platform scaffolding
flutter create . --org com.saeiss.addc

# 3. Fetch dependencies
flutter pub get
```

---

## 💻 Step 2: Running & Testing Locally

You can test the app on your computer using a simulator or your web browser before deploying it to a phone.

1. **Start the Mock Backend (No ROS required):**
   To test the app's UI without needing a full ROS 2 installation on your laptop, you can run the provided mock server:
   ```bash
   cd addc_hmi
   python3 mock_server.py
   ```
   *(This will start a dummy HTTP server on port 5000 that mimics the drone).*

2. **Run the Flutter App:**
   ```bash
   # Run the app locally on a web browser
   flutter run -d web-server
   ```

3. **Connecting the App:**
   - When the app launches, it will ask for the Tailscale IP. 
   - If testing locally, enter: `127.0.0.1:5000` or `localhost:5000`.
   - If connecting to the actual drone, enter the Raspberry Pi's Tailscale IP (e.g., `100.10.20.30:5000`).

---

## 📦 Step 3: Building for Production

When you are ready to install the app permanently on the runner's smartphone, you need to build the production application.

### 🌐 Build for Web (Recommended for Hackathons)
If you prefer the runner to access the app via a URL on their phone's browser instead of installing an app:
```bash
flutter build web --release
```
- **Output location:** `build/web/`
- **Installation:** Host the contents of this folder on any web server.

### 🤖 Build for Android (APK)
This is the easiest method for Android phones.
```bash
flutter build apk --release
```
- **Output location:** `build/app/outputs/flutter-apk/app-release.apk`
- **Installation:** Transfer this file to your Android phone via USB, Google Drive, or email, and open it to install.

### 🍎 Build for iOS (IPA)
*Note: Requires a Mac and Xcode.*
```bash
flutter build ipa --release
```
- **Output location:** `build/ios/archive/Runner.xcarchive` (and subsequently the `.ipa` file).
- **Installation:** You will need to deploy this to your iPhone using Xcode or Apple Configurator.

---

## 🏗 Architecture Overview

- **`lib/main.dart`**: Application entry point and theme configuration.
- **`lib/services/api_service.dart`**: Handles the HTTP `GET` and `POST` requests to the drone over the Tailscale network.
- **`lib/providers/drone_state_provider.dart`**: Core logic. Manages the 1-second polling loop and updates the UI when the drone state changes.
- **`lib/ui/settings_screen.dart`**: The connection screen where the Tailscale IP is configured. Featuring a modern glassmorphic card design.
- **`lib/ui/dashboard_screen.dart`**: The main operative dashboard displaying the access code and sector dispatch buttons with interactive animations.
