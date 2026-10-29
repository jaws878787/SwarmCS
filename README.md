# SwarmCS

C++ ground control station for ArduPilot and PX4 multicopters.

The first milestone provides a full-window OpenStreetMap map, pan/zoom, and a top Connect button. Connect binds IPv4 UDP port 14550 and discovers multicopters from valid MAVLink 1/2 heartbeats. Known vehicles receive a GCS heartbeat once per second; they expire after five seconds without a heartbeat. Disconnect closes the socket. Bind failures appear in the top bar.

## Build

Install CMake 3.24+, a C++17 compiler, and Qt 6.8+ with Qt Quick, Quick Controls 2, Network, Location (including the OSM plugin), and Positioning. The initial configure downloads the official MAVLink C headers, so internet access and Git are required.

```powershell
cmake -S . -B build -DCMAKE_PREFIX_PATH="C:/Qt/6.8.3/msvc2022_64"
cmake --build build --config Debug
./build/bin/Debug/SwarmCS.exe
```

Use your installed Qt kit path and its matching compiler/runtime. On single-configuration generators the executable is directly in `build/bin/`. Deploy Qt libraries and QML plugins with your Qt kit's `windeployqt --qmldir . build/bin/Debug/SwarmCS.exe` when running outside the Qt development environment.

Run the UDP integration tests with `ctest --test-dir build -C Debug --output-on-failure`. Port 14550 must be free.

## Try discovery

Configure ArduPilot SITL, PX4 SITL, or your telemetry router to send MAVLink UDP to this computer's IP on port **14550**, then click **Connect**. UDP discovery listens for incoming traffic; it does not scan network addresses. Permit incoming UDP 14550 through the firewall, and close any other GCS using that port. The map needs internet access for tiles; OpenStreetMap attribution remains visible.

This milestone discovers vehicles only. Flight commands, arming, mission planning, position markers, and connection settings will be added separately. Signed MAVLink authentication is not configured yet. No vehicle commands are sent.
