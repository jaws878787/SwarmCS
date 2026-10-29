# Parameter reference snapshots

The JSON files contain descriptions, raw units, documented ranges and enumerated values from upstream firmware metadata. They are bundled for offline hover help. Each file records its source and retrieval date; the PX4 source is pinned to a QGroundControl commit. Run `scripts/Update-ParameterMetadata.py` with Python to refresh them.

- `copter.json`, `plane.json`: generated ArduPilot parameter documentation from https://autotest.ardupilot.org/Parameters/ (ArduPilot contributors; upstream `ARDUPILOT-COPYING.txt`).
- `px4.json`: PX4 reference metadata distributed by QGroundControl, https://github.com/mavlink/qgroundcontrol (PX4/QGroundControl contributors; upstream `QGC-LICENSE-GPL.txt` and `QGC-LICENSE-APACHE.txt`).

These are reference descriptions rather than metadata reported by the connected firmware. Firmware versions, vehicle types and build options can differ. When upstream lists multiple variants of the same parameter, their distinct ranges are retained and labelled as variants. Unknown metadata is displayed as unavailable; no units or bounds are invented. The reference ranges are informational; existing protocol type/range checks and flight-controller acceptance still govern writes.
