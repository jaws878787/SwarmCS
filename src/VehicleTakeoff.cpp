#include "VehicleLink.h"
#include <cmath>

void VehicleLink::setTakeoffAltitude(double value) {
    const double meters = m_takeoffUnit == "ft" ? value * 0.3048 : value;
    if (!std::isfinite(meters) || meters <= 0 || meters > 10000) return;
    m_takeoffMeters = meters;
    m_settings->setValue("takeoff/altitudeMeters",meters);
    m_settings->sync();
    emit takeoffSettingsChanged();
}

void VehicleLink::toggleTakeoffUnit() {
    m_takeoffUnit = m_takeoffUnit == "ft" ? "m" : "ft";
    m_settings->setValue("takeoff/unit",m_takeoffUnit);
    m_settings->sync();
    emit takeoffSettingsChanged();
}

bool VehicleLink::takeoff(int systemId) {
    if (m_missionStartBatch.contains(systemId)) cancelMissionStarts();
    const int row = rowForSystem(systemId);
    if (row < 0) { emit notice("Vehicle is disconnected."); return false; }
    auto &peer = m_peers[row];
    if (isPlane(peer)) return failVehicle(peer,"Takeoff: multicopters only.");
    if (peer.pendingCommand) { emit notice(nameForSystem(systemId) + ": command pending."); return false; }
    const bool mission = peer.standardMode == MAV_STANDARD_MODE_MISSION
        || (peer.autopilot == MAV_AUTOPILOT_ARDUPILOTMEGA && peer.customMode == 3)
        || (peer.autopilot == MAV_AUTOPILOT_PX4 && peer.customMode == ((4u << 16) | (4u << 24)));
    if (mission && peer.missionState == MISSION_STATE_ACTIVE && peer.armed) {
        // Leave mission items, altitude frames and takeoff heights entirely with the autopilot.
        emit notice(nameForSystem(systemId) + ": using mission altitude.");
        return true;
    }
    if (mission && peer.missionCount < 0) return failVehicle(peer,"Takeoff: mission status unavailable.");
    peer.missionTakeoff = mission && peer.missionCount > 0;
    return advanceTakeoff(peer,m_takeoffMeters);
}

bool VehicleLink::advanceTakeoff(Peer &peer, double altitude) {
    const int systemId = peer.systemId;
    const int row = rowForSystem(systemId);
    if (peer.missionTakeoff) {
        if (!peer.armed) {
            if (!setArmed(systemId,true)) return false;
            peer.queuedTakeoff = altitude;
            return true;
        }
        if (!sendCommand(peer,MAV_CMD_MISSION_START,0,0)) return failVehicle(peer,"Mission not sent.");
        peer.pendingCommand = MAV_CMD_MISSION_START;
        peer.commandDeadline = m_clock.elapsed() + 10000;
        peer.commandStatus = "Starting mission takeoff…";
        emit dataChanged(index(row),index(row),{CommandPendingRole,CommandStatusRole});
        emit notice(nameForSystem(systemId) + ": using mission altitude.");
        return true;
    }
    if (peer.autopilot == MAV_AUTOPILOT_PX4 && (!std::isfinite(peer.homeAltitude) || peer.altitudeTime < 0
        || m_clock.elapsed() - peer.altitudeTime > 5000)) return failVehicle(peer,"Takeoff: altitude unavailable.");
    if (peer.autopilot == MAV_AUTOPILOT_ARDUPILOTMEGA && peer.customMode != 4) {
        if (!setFlightMode(systemId,4)) return false;
        peer.queuedTakeoff = altitude;
        peer.commandStatus = "Entering Guided…";
        emit dataChanged(index(row),index(row),{CommandStatusRole});
        return true;
    }
    if (!peer.armed) {
        if (!setArmed(systemId,true)) return false;
        peer.queuedTakeoff = altitude;
        return true;
    }
    return sendTakeoff(peer,altitude);
}

bool VehicleLink::sendTakeoff(Peer &peer, double altitude) {
    if (!peer.armed || isPlane(peer) || !std::isfinite(altitude) || altitude <= 0) return false;
    mavlink_message_t message{};
    mavlink_status_t status{};
    status.current_tx_seq = m_sequence++;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    if (peer.autopilot == MAV_AUTOPILOT_ARDUPILOTMEGA) {
        if (peer.customMode != 4) return false;
        mavlink_msg_command_int_pack_status(255,MAV_COMP_ID_MISSIONPLANNER,&status,&message,
            peer.systemId,peer.componentId,MAV_FRAME_GLOBAL_RELATIVE_ALT,MAV_CMD_NAV_TAKEOFF,0,0,
            0,0,0,nan,0,0,float(altitude));
    } else {
        if (!std::isfinite(peer.homeAltitude) || peer.altitudeTime < 0 || m_clock.elapsed() - peer.altitudeTime > 5000) {
            return failVehicle(peer,"Takeoff: altitude unavailable.");
        }
        mavlink_msg_command_long_pack_status(255,MAV_COMP_ID_MISSIONPLANNER,&status,&message,
            peer.systemId,peer.componentId,MAV_CMD_NAV_TAKEOFF,0,0,0,0,nan,nan,nan,float(peer.homeAltitude + altitude));
    }
    if (!send(message,peer)) return failVehicle(peer,"Takeoff command not sent.");
    peer.pendingCommand = MAV_CMD_NAV_TAKEOFF;
    peer.takeoffTarget = altitude;
    peer.commandDeadline = m_clock.elapsed() + 60000;
    peer.commandStatus = QString("Takeoff: %1 m…").arg(altitude,0,'f',1);
    const int row = rowForSystem(peer.systemId);
    emit dataChanged(index(row),index(row),{CommandPendingRole,CommandStatusRole});
    emit notice(nameForSystem(peer.systemId) + ": " + peer.commandStatus);
    return true;
}
