# ADDC HMI App

Flutter application for the Ground Operative (runner) in the SAEISS ADDC. 

## Setup

Since this was initialized without platform folders, please run:
```bash
flutter create .
```
to generate the Android/iOS/Web platform runners.

## Running

```bash
flutter run
```

## Architecture

- `lib/services`: HTTP communication with `hmi_bridge_node`.
- `lib/providers`: State management for connection status and drone data.
- `lib/ui`: Screens and widgets (Settings, Dashboard).
