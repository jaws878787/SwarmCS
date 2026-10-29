# SwarmCS

C++ ground control station for ArduPilot and PX4 multicopters.

The map defaults to Esri World Imagery satellite tiles. The bottom-right button switches between satellite and OpenStreetMap street tiles while preserving center and zoom. Each layer uses a separate local tile cache. Maps require internet access and retain provider attribution.

The map initially centers on the computer's location when available through Qt Positioning or the Windows location service. Without a position or permission it keeps the Indianapolis fallback. A first fix centers once; manually moving the map prevents a late fix from interrupting navigation. The Windows fallback reads available location reports for up to 15 seconds without changing system privacy settings.

The top gear opens a preview of connection options; those fields do not change the connection behavior yet. Connect binds IPv4 UDP port 14550 and discovers multicopters from valid MAVLink 1/2 heartbeats. While searching, the status cycles through zero to three dots. Known vehicles receive a GCS heartbeat once per second; they expire after five seconds without a heartbeat. Disconnect closes the socket. Bind failures appear in the top bar.

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

## Local three-vehicle SITL

With the official Windows SITL binary and its Cygwin DLLs installed in `.tools/sitl/bin/` (binary named `ArduCopter.exe`), run `./scripts/Start-Sitl.ps1`. The setup used here comes from [ArduPilot's CopterStable Windows simulator builds](https://firmware.ardupilot.org/Tools/MissionPlanner/sitl/CopterStable/).

This starts three real ArduCopter quadrotor SITL processes with system IDs 1, 2, and 3, separate state/log folders, and homes approximately 20 metres apart. All send MAVLink directly to `127.0.0.1:14550`. Click Connect in SwarmCS to discover them. The script does not arm or fly the vehicles. Use `./scripts/Stop-Sitl.ps1` to stop only the recorded simulator processes. Runtime files and logs remain in the ignored `.tools/sitl/` directory.
