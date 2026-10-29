#include "VehicleLink.h"
#include <cmath>
#include <QGeoCoordinate>

QVariantList VehicleLink::navigationTargets() const {
    QVariantList result;
    for (auto it = m_navigationTargets.cbegin(); it != m_navigationTargets.cend(); ++it) {
        const int row = rowForSystem(it.key());
        if (row < 0 || !positionValid(m_peers[row])) continue;
        const auto &peer = m_peers[row];
        result.append(QVariantMap{{"systemId",it.key()},{"name",nameForSystem(it.key())},
            {"latitude",it->latitude},{"longitude",it->longitude},
            {"fromLatitude",peer.latitude},{"fromLongitude",peer.longitude}});
    }
    return result;
}

void VehicleLink::clearNavigation(int systemId) {
    if (m_navigationTargets.remove(systemId)) emit navigationChanged();
}

QVariantList VehicleLink::navigationCenters() const {
    QVariantList result;
    QSet<QString> seen;
    for (const auto &target : m_navigationTargets) {
        const auto key = QString::number(target.centerLatitude,'f',7) + "," + QString::number(target.centerLongitude,'f',7);
        if (seen.contains(key)) continue;
        seen.insert(key);
        result.append(QVariantMap{{"latitude",target.centerLatitude},{"longitude",target.centerLongitude}});
    }
    return result;
}

void VehicleLink::updateNavigation() {
    if (m_navigationTargets.isEmpty()) return;
    for (auto it = m_navigationTargets.begin(); it != m_navigationTargets.end();) {
        const int row = rowForSystem(it.key());
        bool done = row < 0;
        if (!done) {
            const auto &peer = m_peers[row];
            done = !peer.armed || (positionValid(peer) && peer.altitudeTime >= 0
                && m_clock.elapsed() - peer.altitudeTime < 5000
                && QGeoCoordinate(peer.latitude,peer.longitude).distanceTo(QGeoCoordinate(it->latitude,it->longitude)) <= 2.0
                && std::abs(peer.altitude - it->altitude) <= 2.0);
        }
        if (done) it = m_navigationTargets.erase(it); else ++it;
    }
    emit navigationChanged();
}

void VehicleLink::selectGroup(const QString &group) {
    selectVehicleGroup(group,false);
}

QVariantList VehicleLink::selectedIds() const {
    QVariantList result;
    for (const auto &peer : m_peers) if (m_selection.contains(peer.systemId)) result.append(peer.systemId);
    return result;
}

void VehicleLink::resolveSelection() {
    for (auto it = m_selection.begin(); it != m_selection.end();)
        if (rowForSystem(*it) < 0) it = m_selection.erase(it); else ++it;
    m_selectedSystemId = m_selection.size() == 1 ? *m_selection.cbegin() : 0;
    m_selectedGroup.clear();
    if (m_selection.size() > 1) {
        m_selectedGroup = "__selected__";
        for (const auto &group : m_groups) {
            QSet<int> members;
            for (const auto &peer : m_peers) if (groupForSystem(peer.systemId) == group) members.insert(peer.systemId);
            if (members == m_selection) { m_selectedGroup = group; break; }
        }
    }
    emit selectionChanged();
}

void VehicleLink::selectVehicle(int systemId, bool additive) {
    if (rowForSystem(systemId) < 0 || m_planActive) return;
    if (!additive) m_selection.clear();
    if (additive && m_selection.contains(systemId)) m_selection.remove(systemId); else m_selection.insert(systemId);
    resolveSelection();
    if (m_selection.size()>1) emit groupCenterRequested("__selected__");
    else if (m_selection.size()==1) {
        const int selected=*m_selection.cbegin();
        const auto &peer=m_peers[rowForSystem(selected)];
        if (positionValid(peer)) emit centerRequested(selected,peer.latitude,peer.longitude);
    }
}

void VehicleLink::selectVehicleGroup(const QString &group, bool additive) {
    if ((!m_groups.contains(group) && group != "__selected__") || m_planActive) return;
    const auto members = groupVehicles(group);
    bool all = !members.isEmpty();
    for (const auto &member : members) if (!m_selection.contains(member.toMap().value("systemId").toInt())) all = false;
    if (!additive) m_selection.clear();
    for (const auto &member : members) {
        const int id = member.toMap().value("systemId").toInt();
        if (additive && all) m_selection.remove(id); else m_selection.insert(id);
    }
    resolveSelection();
    if (!additive && group != "__selected__" && members.size() == 1) { m_selectedGroup = group; m_selectedSystemId = 0; emit selectionChanged(); }
    bool any = false;
    int missing = 0;
    for (const auto &peer : m_peers) {
        if (!m_selection.contains(peer.systemId)) continue;
        if (positionValid(peer)) any = true;
        else ++missing;
    }
    if (any) emit groupCenterRequested("__selected__");
    else emit notice(group + ": no positions available.");
    if (any && missing) emit notice(QString("%1: %2 position(s) unavailable.").arg(group).arg(missing));
}

bool VehicleLink::land(int systemId) {
    const int row = rowForSystem(systemId);
    if (row < 0) { emit notice("Vehicle disconnected."); return false; }
    auto &peer = m_peers[row];
    for (const auto &entry : availableModes(systemId)) {
        const auto mode = entry.toMap();
        if (!mode.value("enabled").toBool() || (mode.value("standardMode").toInt() != MAV_STANDARD_MODE_LAND
            && mode.value("name").toString().compare("Land",Qt::CaseInsensitive) != 0)) continue;
        // Landing interrupts a queued arm/mode/takeoff chain.
        if (peer.pendingCommand) finishCommand(peer,"Command replaced.");
        return setFlightMode(systemId,mode.value("id").toUInt(),mode.value("standardMode").toInt());
    }
    return failVehicle(peer,"Land unavailable.");
}

bool VehicleLink::goTo(int systemId, double latitude, double longitude) {
    const int row = rowForSystem(systemId);
    if (row < 0) { emit notice("Vehicle disconnected."); return false; }
    auto &peer = m_peers[row];
    if (!std::isfinite(latitude) || !std::isfinite(longitude) || latitude < -90 || latitude > 90
        || longitude < -180 || longitude > 180) return failVehicle(peer,"Go Here: invalid location.");
    if (isPlane(peer)) return failVehicle(peer,"Go Here: multicopters only.");
    if (!peer.armed || !positionValid(peer) || peer.altitudeTime < 0 || m_clock.elapsed() - peer.altitudeTime > 5000
        || !std::isfinite(peer.altitude) || peer.altitude < 0.5)
        return failVehicle(peer,"Go Here: take off with valid telemetry first.");
    const bool px4 = peer.autopilot == MAV_AUTOPILOT_PX4;
    if (px4 && !std::isfinite(peer.homeAltitude)) return failVehicle(peer,"Go Here: altitude unavailable.");
    if (peer.pendingCommand) finishCommand(peer,"Command replaced.");
    mavlink_message_t message{};
    mavlink_status_t status{};
    status.current_tx_seq = m_sequence++;
    mavlink_msg_command_int_pack_status(255,MAV_COMP_ID_MISSIONPLANNER,&status,&message,
        peer.systemId,peer.componentId,px4 ? MAV_FRAME_GLOBAL : MAV_FRAME_GLOBAL_RELATIVE_ALT,
        MAV_CMD_DO_REPOSITION,0,0,-1,MAV_DO_REPOSITION_FLAGS_CHANGE_MODE,0,
        std::numeric_limits<float>::quiet_NaN(),qint32(std::llround(latitude * 1e7)),qint32(std::llround(longitude * 1e7)),
        float(peer.altitude + (px4 ? peer.homeAltitude : 0)));
    if (!send(message,peer)) return failVehicle(peer,"Go Here not sent.");
    m_navigationTargets.insert(systemId,{latitude,longitude,peer.altitude,latitude,longitude});
    emit navigationChanged();
    peer.pendingCommand = MAV_CMD_DO_REPOSITION;
    peer.commandDeadline = m_clock.elapsed() + 10000;
    peer.commandStatus = "Going to location…";
    emit dataChanged(index(row),index(row),{CommandPendingRole,CommandStatusRole});
    emit notice(nameForSystem(systemId) + ": Go Here requested.");
    return true;
}

void VehicleLink::goToGroup(const QString &group, double latitude, double longitude) {
    const auto vehicles = groupVehicles(group);
    const QGeoCoordinate destination(latitude,longitude);
    if (!destination.isValid()) { emit notice("Go Here: invalid location."); return; }
    // Spherical centroid avoids longitude wraparound at the date line.
    QList<int> members;
    double x = 0, y = 0, z = 0;
    constexpr double radians = 3.14159265358979323846 / 180.0;
    for (const auto &entry : vehicles) {
        const int id = entry.toMap().value("systemId").toInt();
        const auto &peer = m_peers[rowForSystem(id)];
        if (isPlane(peer) || !peer.armed || !positionValid(peer) || peer.altitudeTime < 0
            || m_clock.elapsed() - peer.altitudeTime > 5000 || !std::isfinite(peer.altitude) || peer.altitude < 0.5
            || (peer.autopilot == MAV_AUTOPILOT_PX4 && !std::isfinite(peer.homeAltitude))) continue;
        members.append(id);
        x += std::cos(peer.latitude*radians)*std::cos(peer.longitude*radians);
        y += std::cos(peer.latitude*radians)*std::sin(peer.longitude*radians);
        z += std::sin(peer.latitude*radians);
    }
    const QGeoCoordinate center(std::atan2(z,std::hypot(x,y))/radians,std::atan2(y,x)/radians);
    int requested = 0;
    for (const int id : members) {
        const auto &peer = m_peers[rowForSystem(id)];
        const QGeoCoordinate position(peer.latitude,peer.longitude);
        const auto target = destination.atDistanceAndAzimuth(center.distanceTo(position),center.azimuthTo(position));
        if (goTo(id,target.latitude(),target.longitude())) {
            ++requested;
            m_navigationTargets[id].centerLatitude = latitude;
            m_navigationTargets[id].centerLongitude = longitude;
        }
    }
    emit navigationChanged();
    emit notice(QString("%1 · Go Here: %2 requested, %3 skipped").arg(group).arg(requested).arg(vehicles.size() - requested));
}
