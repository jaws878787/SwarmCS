# SwarmCS

C++ ground control station for ArduPilot and PX4 multicopters.

The map defaults to Esri World Imagery satellite tiles. The bottom-right button switches between satellite and OpenStreetMap street tiles while preserving center and zoom. Each layer uses a separate local tile cache. Maps require internet access and retain provider attribution.

The right-side zoom slider has 20 steps, from 1 (closest) at the top to 20 (farthest) at the bottom, with the selected number above it. It starts at 5 and remembers the selected level across restarts. Mouse-wheel scrolling and touchpad/touchscreen pinch gestures update the same zoom setting around the pointer or gesture center.

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

Connected vehicles appear in the left Vehicles tree, initially stacked under Ungrouped. Cards use a 75% scale with centered, evenly spaced values. Each shows its name, firmware type and reported version, armed state, active flight mode, altitude relative to home (metres), and horizontal ground speed labelled Velocity. The version is requested from the flight controller on discovery; until reported, only the firmware name appears. Left-click Velocity to switch every card between kph and mph; the unit preference is saved. Missing or stale telemetry displays a dash. SwarmCS requests position and ground-speed messages at 2 Hz when a vehicle is discovered.

Each card's gear opens vehicle settings. Names save as you type, with collisions automatically numbered in system-ID order (including saved offline identities). The input keeps the requested base name while the displayed name includes any necessary suffix. Icon choices also persist by sysID: ArduCopter/PX4 offer quadcopter, hexacopter, octocopter, and bullet drone silhouettes; ArduPlane offers fixed wing, delta wing, and wing silhouettes. A saved name labels each valid, fresh map position. Clicking the card centers on its position while preserving the selected zoom.

Right-click a vehicle for Arm/Disarm or the Add to group submenu. Names, icons, groups, and memberships are saved locally with Qt settings; vehicle identity uses the MAVLink system ID, which must be unique across connected vehicles. Right-click the Vehicles title to clear saved names, including offline vehicles, restoring default Vehicle N names. Icons and group assignments remain intact.

Click a card's flight mode to choose another mode. SwarmCS queries the MAVLink available-modes service and uses its complete mode list when supported, otherwise displaying firmware defaults. Unsupported or non-selectable modes cannot be forced. Requests use the firmware's ArduPilot/PX4 mode encoding, and arm/disarm always leaves force flags unset. Pending commands are confirmed by heartbeat/current-mode telemetry; rejection and timeout messages appear over the map. Only commands selected by the user are sent; no automatic arming or mode changes occur.

Mission planning and functional connection settings will be added separately. Signed MAVLink authentication is not configured yet.

## Local three-vehicle SITL

With the official Windows SITL binary and its Cygwin DLLs installed in `.tools/sitl/bin/` (binary named `ArduCopter.exe`), run `./scripts/Start-Sitl.ps1`. The setup used here comes from [ArduPilot's CopterStable Windows simulator builds](https://firmware.ardupilot.org/Tools/MissionPlanner/sitl/CopterStable/).

This starts three real ArduCopter quadrotor SITL processes with system IDs 1, 2, and 3, separate state/log folders, and homes approximately 20 metres apart. All send MAVLink directly to `127.0.0.1:14550`. Click Connect in SwarmCS to discover them. The script does not arm or fly the vehicles. Use `./scripts/Stop-Sitl.ps1` to stop only the recorded simulator processes. Runtime files and logs remain in the ignored `.tools/sitl/` directory.
