#include "VehicleLink.h"
#include <QGeoCoordinate>
#include <algorithm>
#include <cmath>

bool VehicleLink::missionMode(const Peer &p) const {
    return p.standardMode==MAV_STANDARD_MODE_MISSION
        || (p.autopilot==MAV_AUTOPILOT_ARDUPILOTMEGA && p.customMode==3)
        || (p.autopilot==MAV_AUTOPILOT_PX4 && p.customMode==((4u<<16)|(4u<<24)));
}

bool VehicleLink::startMission(int id) {
    const int row=rowForSystem(id); if (row<0) return false;
    auto &p=m_peers[row];
    if (p.pendingCommand) return failVehicle(p,"Mission: command pending.");
    if (p.missionCount<=0) return failVehicle(p,"Mission unavailable.");
    if (!p.armed) {
        if (!setArmed(id,true)) return false;
        p.queuedMissionStart=true;
        return true;
    }
    if (m_missionStartBatch.contains(id)) { p.queuedMissionStart=true; return true; }
    if (!sendCommand(p,MAV_CMD_MISSION_START,0,0)) return failVehicle(p,"Mission not sent.");
    p.pendingCommand=MAV_CMD_MISSION_START; p.commandDeadline=m_clock.elapsed()+10000;
    p.commandStatus="Starting mission…";
    emit dataChanged(index(row),index(row),{CommandPendingRole,CommandStatusRole});
    return true;
}

void VehicleLink::cancelMissionStarts() {
    if (m_missionStartBatch.isEmpty()) return;
    for (auto &p:m_peers) if (m_missionStartBatch.contains(p.systemId)) p.queuedMissionStart=false;
    m_missionStartBatch.clear();
    emit notice("Group mission start cancelled.");
}

void VehicleLink::advanceMissionStarts() {
    if (m_missionStartBatch.isEmpty()) return;
    for (int id:m_missionStartBatch) {
        const int row=rowForSystem(id);
        if (row<0 || m_peers[row].missionCount<=0) { cancelMissionStarts(); return; }
        const auto &p=m_peers[row];
        if (!p.armed || p.pendingCommand) return;
    }
    const auto ids=m_missionStartBatch; m_missionStartBatch.clear();
    for (int id:ids) { m_peers[rowForSystem(id)].queuedMissionStart=false; startMission(id); }
}

QVariantMap VehicleLink::sharedFlightPlan() const {
    if (m_planActive || m_selection.size()<2) return {};
    m_settings->beginGroup("missionPlans"); const auto keys=m_settings->childKeys(); m_settings->endGroup();
    for (const auto &key:keys) {
        const auto saved=m_settings->value("missionPlans/"+key).toMap();
        if (saved.value("version").toInt()!=2 || saved.value("geometryVersion").toInt()<4 || saved.value("points").toList().size()<2) continue;
        const auto members=saved.value("vehicles").toList(); bool match=true;
        for (int id:m_selection) {
            const int row=rowForSystem(id);
            if (row<0 || isPlane(m_peers[row])) { match=false; break; }
            bool found=false;
            for (const auto &v:members) if (v.toMap().value("id").toInt()==id) found=true;
            if (!found || rowForSystem(id)<0 || m_flightMissions.value(id).isEmpty()
                || !missionMatchesMetadata(id,saved,m_flightMissions.value(id))) { match=false; break; }
        }
        if (match) return saved;
    }
    return {};
}

bool VehicleLink::liveAdjustmentAvailable() const { return !sharedFlightPlan().isEmpty(); }

void VehicleLink::setLiveAdjustmentEnabled(bool enabled) {
    if (!enabled) { stopLiveAdjustment("Live adjustments off."); return; }
    if (m_liveEnabled || !liveAdjustmentAvailable()) return;
    m_liveIds=m_selection; m_liveSessions.clear(); m_liveSpeeds.clear(); m_liveFlying=false;
    for (int id:m_liveIds) m_liveSessions[id]=m_peers[rowForSystem(id)].session;
    m_liveEnabled=true; m_liveStatus="Waiting for mission flight"; emit liveAdjustmentChanged();
}

void VehicleLink::stopLiveAdjustment(const QString &reason,bool restore) {
    if (!m_liveEnabled) return;
    m_liveEnabled=false;
    if (restore) for (auto it=m_liveSpeeds.cbegin();it!=m_liveSpeeds.cend();++it) {
        const int row=rowForSystem(it.key()); if (row<0) continue;
        auto &p=m_peers[row];
        if (p.session==m_liveSessions.value(it.key()) && p.armed && missionMode(p) && !p.pendingCommand
            && p.missionTime>=0 && m_clock.elapsed()-p.missionTime<=3000 && it->baseline>0) {
            double baseline=it->baseline;
            for (const auto &item:m_flightMissions.value(it.key())) {
                if (item.seq>p.missionSequence) break;
                if (item.command==MAV_CMD_DO_CHANGE_SPEED && item.param1==1) baseline=item.param2;
            }
            sendCommand(p,MAV_CMD_DO_CHANGE_SPEED,1,baseline,-1);
        }
    }
    m_liveIds.clear(); m_liveSessions.clear(); m_liveSpeeds.clear(); m_liveFlying=false;
    m_liveStatus=reason; emit notice(reason); emit liveAdjustmentChanged();
}

bool VehicleLink::receiveLiveSpeedAck(Peer &p,const mavlink_command_ack_t &ack) {
    if (ack.command!=MAV_CMD_DO_CHANGE_SPEED || !m_liveEnabled || !m_liveSpeeds.contains(p.systemId)
        || (ack.target_system && ack.target_system!=255)
        || (ack.target_component && ack.target_component!=MAV_COMP_ID_MISSIONPLANNER)) return false;
    auto &speed=m_liveSpeeds[p.systemId];
    if (ack.result==MAV_RESULT_IN_PROGRESS) speed.awaiting=m_clock.elapsed();
    else if (ack.result==MAV_RESULT_ACCEPTED) speed.awaiting=-1;
    else stopLiveAdjustment(nameForSystem(p.systemId)+": speed adjustment rejected.");
    return true;
}

void VehicleLink::liveAdjustmentTick() {
    emit liveAdjustmentChanged();
    if (!m_liveEnabled) return;
    if (m_selection!=m_liveIds || !liveAdjustmentAvailable()) { stopLiveAdjustment("Live adjustments: selection or mission changed."); return; }
    struct Flight { int id,stage; double progress,baseline; };
    QList<Flight> flights; bool ready=true;
    for (int id:m_liveIds) {
        const int row=rowForSystem(id);
        if (row<0 || m_peers[row].session!=m_liveSessions.value(id)) { stopLiveAdjustment("Live adjustments: vehicle disconnected."); return; }
        const auto &p=m_peers[row];
        if (m_liveFlying && p.missionState==MISSION_STATE_COMPLETE) { stopLiveAdjustment("Live adjustments: mission completed."); return; }
        if (!p.armed || !missionMode(p) || p.pendingCommand) { ready=false; continue; }
        if (!positionValid(p) || m_clock.elapsed()-p.positionTime>1500 || p.missionTime<0 || m_clock.elapsed()-p.missionTime>3000) {
            stopLiveAdjustment("Live adjustments: telemetry stale."); return;
        }
        if (m_liveSpeeds.contains(id) && m_liveSpeeds[id].awaiting>=0 && m_clock.elapsed()-m_liveSpeeds[id].awaiting>3000) {
            stopLiveAdjustment("Live adjustments: speed command timed out."); return;
        }
        const auto mission=m_flightMissions.value(id);
        int stage=-1; double speed=0; QGeoCoordinate previous; bool found=false;
        for (const auto &i:mission) {
            if (i.seq==0 && p.autopilot==MAV_AUTOPILOT_ARDUPILOTMEGA && i.command==MAV_CMD_NAV_WAYPOINT) continue;
            if (i.command==MAV_CMD_DO_CHANGE_SPEED && i.param1==1) speed=i.param2;
            if (i.command!=MAV_CMD_NAV_WAYPOINT && i.command!=MAV_CMD_NAV_TAKEOFF && i.command!=MAV_CMD_NAV_DELAY) continue;
            ++stage;
            const auto target=i.command==MAV_CMD_NAV_DELAY ? previous : QGeoCoordinate(i.x/1e7,i.y/1e7);
            if (i.seq>=p.missionSequence) {
                const double length=previous.isValid() ? previous.distanceTo(target) : 0;
                if (i.command!=MAV_CMD_NAV_WAYPOINT || length<2 || speed<0.1 || speed>30) { ready=false; found=true; break; }
                const QGeoCoordinate current(p.latitude,p.longitude);
                const double angle=(previous.azimuthTo(current)-previous.azimuthTo(target))*3.141592653589793/180;
                const double distance=previous.distanceTo(current);
                if (std::abs(distance*std::sin(angle))>20) { stopLiveAdjustment("Live adjustments: vehicle off route."); return; }
                flights.append({id,stage,stage+std::clamp(distance*std::cos(angle)/length,0.0,1.0),speed});
                found=true; break;
            }
            previous=target;
        }
        if (!found) ready=false;
    }
    if (!ready || flights.size()!=m_liveIds.size()) {
        if (m_liveFlying) {
            for (int id:m_liveIds) { const auto &p=m_peers[rowForSystem(id)];
                if (!p.armed || !missionMode(p) || p.missionState==MISSION_STATE_COMPLETE) {
                    stopLiveAdjustment("Live adjustments: mission flight ended."); return;
                }
            }
        }
        m_liveStatus=m_liveFlying ? "Waiting for horizontal mission legs" : "Waiting for mission flight";
        return;
    }
    m_liveFlying=true; double slowest=flights.first().progress;
    for (const auto &f:flights) slowest=std::min(slowest,f.progress);
    for (const auto &f:flights) {
        auto &p=m_peers[rowForSystem(f.id)]; auto &last=m_liveSpeeds[f.id];
        const double lead=std::max(0.0,f.progress-slowest-0.02);
        double desired=std::max(std::min(0.5,f.baseline),f.baseline*std::clamp(1-2*lead,0.0,1.0));
        const bool changedStage=last.stage!=f.stage;
        if (!changedStage) desired=std::clamp(desired,last.requested-1,last.requested+1);
        const bool sendUpdate=changedStage || std::abs(desired-last.requested)>=0.2;
        last.baseline=f.baseline;
        if (last.awaiting>=0 || !sendUpdate) continue;
        if (!sendCommand(p,MAV_CMD_DO_CHANGE_SPEED,1,desired,-1)) { stopLiveAdjustment("Live speed command failed."); return; }
        last.requested=desired; last.stage=f.stage; last.awaiting=m_clock.elapsed();
    }
    m_liveStatus="Coordinating mission speeds";
}
