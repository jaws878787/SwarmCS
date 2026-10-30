#include "VehicleLink.h"

namespace {
QString modeKey(QString name, int standardMode = 0) {
    if (standardMode == MAV_STANDARD_MODE_SAFE_RECOVERY) return "rtl";
    if (standardMode == MAV_STANDARD_MODE_MISSION) return "mission";
    name = name.toCaseFolded().remove(' ').remove('_').remove('-');
    if (name == "auto") return "mission";
    if (name == "stabilized") return "stabilize";
    if (name == "return" || name == "returntolaunch" || name == "returntohome" || name == "saferecovery") return "rtl";
    return name;
}
}

QVariantList VehicleLink::groupVehicles(const QString &group) const {
    QVariantList result;
    for (const auto &peer : m_peers)
        if (group == "__selected__" ? m_selection.contains(peer.systemId) : groupForSystem(peer.systemId) == group)
            result.append(QVariantMap{{"systemId",peer.systemId},{"name",nameForSystem(peer.systemId)}});
    return result;
}

QVariantList VehicleLink::groupModes(const QString &group) const {
    const auto vehicles = groupVehicles(group);
    if (vehicles.isEmpty()) return {};
    QVariantList result;
    const auto first = availableModes(vehicles.first().toMap().value("systemId").toInt());
    for (const auto &entry : first) {
        const auto candidate = entry.toMap();
        if (!candidate.value("enabled").toBool()) continue;
        const auto key = modeKey(candidate.value("name").toString(),candidate.value("standardMode").toInt());
        bool common = true;
        for (const auto &vehicle : vehicles) {
            bool found = false;
            for (const auto &mode : availableModes(vehicle.toMap().value("systemId").toInt())) {
                const auto option = mode.toMap();
                if (option.value("enabled").toBool() && modeKey(option.value("name").toString(),option.value("standardMode").toInt()) == key)
                    found = true;
            }
            if (!found) { common = false; break; }
        }
        if (common) result.append(QVariantMap{{"key",key},{"name",candidate.value("name")}});
    }
    return result;
}

void VehicleLink::groupAction(const QString &group, const QString &action, const QString &mode) {
    const auto vehicles = groupVehicles(group);
    if (action=="mission") {
        if (!m_missionStartBatch.isEmpty()) { emit notice("Mission start already pending."); return; }
        for (const auto &v:vehicles) {
            const int id=v.toMap().value("systemId").toInt(); const auto &p=m_peers[rowForSystem(id)];
            if (p.pendingCommand || p.missionCount<=0) { failVehicle(m_peers[rowForSystem(id)],"Mission unavailable or command pending."); return; }
        }
        for (const auto &v:vehicles) m_missionStartBatch.insert(v.toMap().value("systemId").toInt());
    }
    QStringList skipped;
    int sent = 0;
    for (const auto &vehicle : vehicles) {
        const auto item = vehicle.toMap();
        const int id = item.value("systemId").toInt();
        auto &peer = m_peers[rowForSystem(id)];
        bool success = false;
        if (action == "arm" || action == "disarm") success = setArmed(id, action == "arm");
        else if (action == "takeoff") success = takeoff(id);
        else if (action == "land") success = land(id);
        else if (action == "mode" || action == "rtl") {
            const auto requested = action == "rtl" ? QString("rtl") : modeKey(mode);
            for (const auto &entry : availableModes(id)) {
                const auto option = entry.toMap();
                if (option.value("enabled").toBool() && modeKey(option.value("name").toString(),option.value("standardMode").toInt()) == requested) {
                    if (action == "rtl" && peer.pendingCommand) finishCommand(peer,"Command replaced.");
                    success = setFlightMode(id, option.value("id").toUInt(), option.value("standardMode").toInt());
                    break;
                }
            }
        } else if (action == "mission") success = startMission(id);
        if (success) ++sent;
        else skipped.append(item.value("name").toString());
        if (action=="mission" && !success) { cancelMissionStarts(); break; }
    }
    if (action=="mission") advanceMissionStarts();
    const QString label = action == "arm" ? "Arm" : action == "disarm" ? "Disarm" : action == "takeoff" ? "Takeoff" : action == "land" ? "Land"
        : action == "mission" ? "Mission" : action == "rtl" ? "RTL" : "Mode";
    QString report = QString("%1 · %2: %3 requested").arg(group,label).arg(sent);
    if (!skipped.isEmpty()) report += QString(", %1 skipped").arg(skipped.size());
    if (vehicles.isEmpty()) report = group + ": no vehicles.";
    emit notice(report);
}
