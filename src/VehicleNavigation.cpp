#include "VehicleLink.h"
#include <cmath>

void VehicleLink::selectGroup(const QString &group) {
    if (!m_groups.contains(group)) return;
    m_selectedGroup = group;
    m_selectedSystemId = 0;
    emit selectionChanged();
    bool any = false;
    int missing = 0;
    for (const auto &peer : m_peers) {
        if (groupForSystem(peer.systemId) != group) continue;
        if (positionValid(peer)) any = true;
        else ++missing;
    }
    if (any) emit groupCenterRequested(group);
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
    peer.pendingCommand = MAV_CMD_DO_REPOSITION;
    peer.commandDeadline = m_clock.elapsed() + 10000;
    peer.commandStatus = "Going to location…";
    emit dataChanged(index(row),index(row),{CommandPendingRole,CommandStatusRole});
    emit notice(nameForSystem(systemId) + ": Go Here requested.");
    return true;
}

void VehicleLink::goToGroup(const QString &group, double latitude, double longitude) {
    const auto vehicles = groupVehicles(group);
    int requested = 0;
    for (const auto &entry : vehicles)
        if (goTo(entry.toMap().value("systemId").toInt(),latitude,longitude)) ++requested;
    emit notice(QString("%1 · Go Here: %2 requested, %3 skipped").arg(group).arg(requested).arg(vehicles.size() - requested));
}
