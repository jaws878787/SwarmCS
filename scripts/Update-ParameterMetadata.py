"""Refresh the bundled, offline parameter reference from upstream metadata."""
import datetime
import json
import pathlib
import urllib.request
import xml.etree.ElementTree as ET

ROOT = pathlib.Path(__file__).resolve().parents[1] / "assets" / "parameters"
ROOT.mkdir(parents=True, exist_ok=True)

def fetch(url):
    with urllib.request.urlopen(url, timeout=60) as response:
        return response.read()

def write(name, source, parameters):
    data = {"source": source, "retrieved": datetime.date.today().isoformat(), "parameters": parameters}
    (ROOT / (name + ".json")).write_text(json.dumps(data, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
    print(name, len(parameters), "parameter descriptions")

for name, vehicle in [("copter", "ArduCopter"), ("plane", "ArduPlane")]:
    source = f"https://autotest.ardupilot.org/Parameters/{vehicle}/apm.pdef.xml"
    result = {}
    for param in ET.fromstring(fetch(source)).iter("param"):
        key = param.get("name", "").split(":")[-1]
        if not key:
            continue
        fields = {field.get("name"): (field.text or "").strip() for field in param.findall("field")}
        entry = result.setdefault(key, {"summary": param.get("humanName", ""),
            "description": param.get("documentation", ""), "units": fields.get("Units", ""), "ranges": [], "choices": []})
        valid = fields.get("Range", "")
        parts = valid.split()
        if len(parts) == 2:
            try:
                float(parts[0]); float(parts[1])
                valid = parts[0] + " … " + parts[1]
            except ValueError:
                pass
        if valid and valid not in entry["ranges"]:
            entry["ranges"].append(valid)
        for value in param.findall("values/value"):
            choice = value.get("code", "") + ": " + (value.text or "")
            if choice not in entry["choices"]:
                entry["choices"].append(choice)
        if fields.get("Bitmask"):
            choice = "Bitmask: " + fields["Bitmask"]
            if choice not in entry["choices"]:
                entry["choices"].append(choice)
    write(name, source, result)

commit = json.loads(fetch("https://api.github.com/repos/mavlink/qgroundcontrol/commits/master"))["sha"]
source = f"https://raw.githubusercontent.com/mavlink/qgroundcontrol/{commit}/src/FirmwarePlugin/PX4/PX4ParameterFactMetaData.json"
result = {}
for param in json.loads(fetch(source))["parameters"]:
    minimum, maximum = param.get("min"), param.get("max")
    valid = f"{minimum} … {maximum}" if minimum is not None and maximum is not None else (
        f"≥ {minimum}" if minimum is not None else f"≤ {maximum}" if maximum is not None else "")
    result[param["name"]] = {"summary": param.get("shortDesc", ""), "description": param.get("longDesc", ""),
        "units": param.get("units", ""), "ranges": [valid] if valid else [],
        "choices": [str(value["value"]) + ": " + value.get("description", "") for value in param.get("values", [])]}
    bits = param.get("bitmask", [])
    if bits:
        result[param["name"]]["choices"].append("Bitmask: " + ", ".join(str(bit["index"]) + ": " + bit.get("description", "") for bit in bits))
write("px4", source, result)
