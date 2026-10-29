#include "VehicleLink.h"
#include <QGeoCoordinate>
#include <cmath>
#include <algorithm>

namespace {
constexpr double pi = 3.14159265358979323846;
struct Vec { double x=0,y=0,z=0; Vec operator-(Vec b) const { return {x-b.x,y-b.y,z-b.z}; } };
double dot(Vec a,Vec b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
Vec local(const QGeoCoordinate &origin,const QGeoCoordinate &point,double altitude=0) {
    const auto distance=origin.distanceTo(point), bearing=origin.azimuthTo(point)*pi/180;
    return {distance*std::sin(bearing),distance*std::cos(bearing),altitude};
}
QGeoCoordinate geographic(const QGeoCoordinate &origin,Vec point) {
    return origin.atDistanceAndAzimuth(std::hypot(point.x,point.y),std::atan2(point.x,point.y)*180/pi);
}
mavlink_mission_item_int_t item(int command,double latitude=0,double longitude=0,double altitude=0) {
    mavlink_mission_item_int_t result{};
    result.command=command; result.frame=MAV_FRAME_GLOBAL_RELATIVE_ALT_INT; result.autocontinue=1;
    result.x=qint32(std::llround(latitude*1e7)); result.y=qint32(std::llround(longitude*1e7)); result.z=altitude;
    result.mission_type=MAV_MISSION_TYPE_MISSION;
    return result;
}
bool navigation(const mavlink_mission_item_int_t &i) {
    return i.command==MAV_CMD_NAV_WAYPOINT || i.command==MAV_CMD_NAV_TAKEOFF || i.command==MAV_CMD_NAV_LAND;
}
}

bool VehicleLink::missionBusy() const {
    for (const auto &transfer:m_missionTransfers)
        if (transfer.operation!=MissionTransfer::Done && transfer.operation!=MissionTransfer::Failed) return true;
    return false;
}
QVariantList VehicleLink::planVehicles() const {
    QVariantList result;
    for (const auto &v:m_planVehicles) result.append(QVariantMap{{"systemId",v.id},{"name",nameForSystem(v.id)}});
    return result;
}
QVariantList VehicleLink::planItems() const {
    QVariantList result;
    if (m_loadedPlan) {
        for (const auto &v:m_planVehicles) {
            const auto mission=m_plannedMissions.value(v.id);
            for (int seq=0;seq<mission.size();++seq) {
                const auto &i=mission[seq];
                const int row=rowForSystem(v.id);
                if (seq==0 && row>=0 && m_peers[row].autopilot==MAV_AUTOPILOT_ARDUPILOTMEGA && i.command==MAV_CMD_NAV_WAYPOINT) continue;
                const QString command=i.command==MAV_CMD_NAV_TAKEOFF ? "Takeoff" : i.command==MAV_CMD_NAV_WAYPOINT ? "Waypoint"
                    : i.command==MAV_CMD_DO_CHANGE_SPEED ? "Speed" : i.command==MAV_CMD_NAV_LAND ? "Land" : QString("Command %1").arg(i.command);
                result.append(QVariantMap{{"title",QString("%1 · %2 · %3").arg(nameForSystem(v.id)).arg(seq).arg(command)},
                    {"altitude",double(i.z)},{"speed",i.command==MAV_CMD_DO_CHANGE_SPEED ? double(i.param2) : 0.0},
                    {"editable",false},{"frame",int(i.frame)}});
            }
        }
    } else for (int n=0;n<m_planPoints.size();++n) {
        const auto &p=m_planPoints[n];
        result.append(QVariantMap{{"title",n==0 ? QString("Takeoff") : QString("Waypoint %1").arg(n)},
            {"altitude",p.altitude},{"speed",p.speed},{"editable",true},{"frame",int(MAV_FRAME_GLOBAL_RELATIVE_ALT_INT)}});
    }
    return result;
}
QVariantList VehicleLink::planCenterPath() const {
    QVariantList result;
    for (const auto &p:m_planPoints) result.append(QVariantMap{{"latitude",p.latitude},{"longitude",p.longitude}});
    return result;
}
QVariantList VehicleLink::planPaths() const {
    QVariantList result;
    for (const auto &v:m_planVehicles) {
        QVariantList path;
        for (const auto &i:m_plannedMissions.value(v.id)) if (navigation(i)) {
            // ArduPilot sequence zero is its home placeholder, not a flown waypoint.
            const int row=rowForSystem(v.id);
            if (i.seq==0 && row>=0 && m_peers[row].autopilot==MAV_AUTOPILOT_ARDUPILOTMEGA && i.command==MAV_CMD_NAV_WAYPOINT) continue;
            const bool atStart=i.command==MAV_CMD_NAV_TAKEOFF && !i.x && !i.y;
            path.append(QVariantMap{{"latitude",atStart ? v.latitude : i.x/1e7},{"longitude",atStart ? v.longitude : i.y/1e7},{"altitude",double(i.z)}});
        }
        result.append(QVariantMap{{"systemId",v.id},{"name",nameForSystem(v.id)},{"path",path}});
    }
    return result;
}

void VehicleLink::togglePlan() {
    if (missionBusy()) { emit notice("Wait for mission transfer to finish."); return; }
    if (m_planActive) { m_planActive=false; emit planChanged(); return; }
    if (m_selection.isEmpty()) { emit notice("Select vehicles to plan for."); return; }
    QList<PlanVehicle> vehicles;
    for (const auto &peer:m_peers) if (m_selection.contains(peer.systemId)) {
        if (isPlane(peer) || !positionValid(peer) || !std::isfinite(peer.homeAltitude)) {
            emit notice("Planning needs multicopters with fresh position and home altitude."); return;
        }
        const bool home=QGeoCoordinate(peer.homeLatitude,peer.homeLongitude).isValid();
        if (peer.armed && !home) { emit notice("Planning needs takeoff/home positions for airborne vehicles."); return; }
        vehicles.append({peer.systemId,peer.session,home ? peer.homeLatitude : peer.latitude,home ? peer.homeLongitude : peer.longitude,peer.homeAltitude});
    }
    if (vehicles.isEmpty()) return;
    m_planVehicles=vehicles; m_planPoints.clear(); m_plannedMissions.clear(); m_missionTransfers.clear();
    m_planActive=true; m_planValid=false; m_loadedPlan=false;
    m_planStatus="Loading missions from selected vehicles…";
    for (const auto &v:m_planVehicles) { m_missionTransfers.insert(v.id,{}); sendMissionTransfer(v.id); }
    emit planChanged();
}

void VehicleLink::setMinimumSeparation(double meters) {
    if (!std::isfinite(meters) || meters<0.3 || meters>1000 || missionBusy()) return;
    m_minimumSeparation=meters;
    if (!m_loadedPlan) rebuildPlan(); else { m_planValid=false; m_planStatus="Loaded missions are shown unchanged. Clear to design a checked swarm route."; emit planChanged(); }
}
void VehicleLink::addPlanWaypoint(double latitude,double longitude) {
    if (!m_planActive || missionBusy()) return;
    if (m_loadedPlan) { emit notice("Clear loaded missions before creating a new swarm route."); return; }
    if (!QGeoCoordinate(latitude,longitude).isValid() || m_planPoints.size()>=100) return;
    const auto previous=m_planPoints.isEmpty() ? PlanPoint{} : m_planPoints.last();
    m_planPoints.append({latitude,longitude,previous.altitude,previous.speed}); rebuildPlan();
}
void VehicleLink::editPlanItem(int row,double altitudeFeet,double speed) {
    const double altitude=altitudeFeet*0.3048;
    if (m_loadedPlan || missionBusy() || row<0 || row>=m_planPoints.size() || !std::isfinite(altitude)
        || altitude<0.3 || altitude>500 || !std::isfinite(speed) || speed<0.1 || speed>30) return;
    m_planPoints[row].altitude=altitude; m_planPoints[row].speed=speed; rebuildPlan();
}
void VehicleLink::removePlanItem(int row) {
    if (m_loadedPlan || missionBusy() || row<=0 || row>=m_planPoints.size()) return;
    m_planPoints.removeAt(row); rebuildPlan();
}

void VehicleLink::rebuildPlan() {
    m_plannedMissions.clear(); m_planValid=false;
    if (m_planVehicles.isEmpty()) { emit planChanged(); return; }
    // One local metric frame keeps all formation translations parallel, including across the date line.
    const QGeoCoordinate origin(m_planVehicles.first().latitude,m_planVehicles.first().longitude);
    Vec centroid;
    QList<Vec> offsets;
    for (const auto &v:m_planVehicles) {
        const auto p=local(origin,QGeoCoordinate(v.latitude,v.longitude));
        offsets.append(p); centroid.x+=p.x/m_planVehicles.size(); centroid.y+=p.y/m_planVehicles.size();
    }
    if (m_planPoints.isEmpty()) {
        const auto center=geographic(origin,centroid);
        m_planPoints.append({center.latitude(),center.longitude(),15.24,5});
    }
    for (auto &p:offsets) { p.x-=centroid.x; p.y-=centroid.y; }
    QString error;
    double closestPair=std::numeric_limits<double>::infinity();
    for (int a=0;a<offsets.size();++a) for (int b=a+1;b<offsets.size();++b) {
        const double distance=std::hypot(offsets[a].x-offsets[b].x,offsets[a].y-offsets[b].y);
        closestPair=std::min(closestPair,distance);
        if (distance<m_minimumSeparation+0.1)
            error="Starting/landing formation is closer than minimum separation. Move vehicles apart or reduce separation.";
    }
    const double compactScale=closestPair>0 ? std::min(1.0,(m_minimumSeparation+1)/closestPair) : 1;
    QList<QList<Vec>> stages;
    QList<double> stageSpeeds;
    auto formation=[&](int pointIndex,bool layered) {
        const auto &point=m_planPoints[pointIndex];
        const double scale=pointIndex>0 && pointIndex<m_planPoints.size()-1 ? compactScale : 1;
        const auto center=local(origin,QGeoCoordinate(point.latitude,point.longitude));
        QList<Vec> nodes;
        for (int n=0;n<offsets.size();++n) nodes.append({center.x+offsets[n].x*scale,center.y+offsets[n].y*scale,
            m_planVehicles[n].homeAltitude+point.altitude+(layered ? n*(m_minimumSeparation+1) : 0)});
        return nodes;
    };
    stages.append(formation(0,false)); stageSpeeds.append(5);
    for (int p=1;p<m_planPoints.size();++p) {
        // Separate cruise lanes vertically. Transition at formation slots, then restore takeoff formation at every stop.
        if (offsets.size()>1) { stages.append(formation(p-1,true)); stageSpeeds.append(m_planPoints[p].speed); }
        stages.append(formation(p,offsets.size()>1)); stageSpeeds.append(m_planPoints[p].speed);
        if (offsets.size()>1) { stages.append(formation(p,false)); stageSpeeds.append(m_planPoints[p].speed); }
    }
    double totalSeconds=0;
    QList<double> durations{0};
    for (int s=1;s<stages.size();++s) {
        double duration=0;
        for (int n=0;n<offsets.size();++n) {
            const auto d=stages[s][n]-stages[s-1][n];
            duration=std::max(duration,std::max(std::hypot(d.x,d.y)/stageSpeeds[s],std::abs(d.z)/2.0));
        }
        duration=std::max(1.0,duration); durations.append(duration); totalSeconds+=duration;
        for (int a=0;a<offsets.size();++a) for (int b=a+1;b<offsets.size();++b) {
            const auto relative=stages[s-1][a]-stages[s-1][b];
            const auto delta=(stages[s][a]-stages[s-1][a])-(stages[s][b]-stages[s-1][b]);
            const double t=dot(delta,delta)>1e-12 ? std::clamp(-dot(relative,delta)/dot(delta,delta),0.0,1.0) : 0;
            const Vec nearest{relative.x+t*delta.x,relative.y+t*delta.y,relative.z+t*delta.z};
            if (std::sqrt(dot(nearest,nearest))<m_minimumSeparation) error="Unable to resolve a path separation conflict.";
        }
    }
    for (int n=0;n<m_planVehicles.size();++n) {
        QList<mavlink_mission_item_int_t> mission;
        const auto &vehicle=m_planVehicles[n];
        const auto row=rowForSystem(vehicle.id);
        if (row<0 || m_peers[row].session!=vehicle.session) error="A planning vehicle disconnected. Return to Fly and select again.";
        if (row>=0 && m_peers[row].autopilot==MAV_AUTOPILOT_ARDUPILOTMEGA)
            mission.append(item(MAV_CMD_NAV_WAYPOINT,vehicle.latitude,vehicle.longitude));
        const auto first=geographic(origin,stages.first()[n]);
        mission.append(item(MAV_CMD_NAV_TAKEOFF,first.latitude(),first.longitude(),m_planPoints.first().altitude));
        for (int s=1;s<stages.size();++s) {
            const auto d=stages[s][n]-stages[s-1][n];
            const double horizontal=std::hypot(d.x,d.y), vertical=std::abs(d.z);
            if ((horizontal>0.01 && horizontal/durations[s]<0.1) || (vertical>0.01 && vertical/durations[s]<0.1))
                error="Nominal synchronization requires a speed below 0.1 m/s. Adjust the waypoint or incoming speed.";
            if (horizontal<0.01 && vertical<0.01) {
                auto wait=item(MAV_CMD_NAV_DELAY); wait.param1=durations[s]; mission.append(wait); continue;
            }
            if (horizontal>0.01) { auto speed=item(MAV_CMD_DO_CHANGE_SPEED); speed.param1=1; speed.param2=std::max(0.1,horizontal/durations[s]); speed.param3=-1; mission.append(speed); }
            if (vertical>0.01) { auto speed=item(MAV_CMD_DO_CHANGE_SPEED); speed.param1=d.z>0 ? 2 : 3; speed.param2=std::max(0.1,vertical/durations[s]); speed.param3=-1; mission.append(speed); }
            const auto coordinate=geographic(origin,stages[s][n]);
            const double altitude=stages[s][n].z-vehicle.homeAltitude;
            if (origin.distanceTo(coordinate)>25000 || altitude>500 || altitude<0.3) error="Route exceeds the 25 km local planning area or 500 m relative altitude limit.";
            auto waypoint=item(MAV_CMD_NAV_WAYPOINT,coordinate.latitude(),coordinate.longitude(),altitude);
            waypoint.param2=0.5; mission.append(waypoint);
        }
        for (int seq=0;seq<mission.size();++seq) { mission[seq].seq=seq; mission[seq].current=seq==0; }
        m_plannedMissions[vehicle.id]=mission;
    }
    m_planValid=error.isEmpty() && m_planPoints.size()>1;
    m_planStatus=!error.isEmpty() ? error : m_planPoints.size()==1 ? "Click the map to add a waypoint."
        : QString("Separation checked · nominal route %1 s · synchronized speed targets").arg(std::ceil(totalSeconds));
    emit planChanged();
}

void VehicleLink::sendMissionTransfer(int systemId) {
    const int row=rowForSystem(systemId); if (row<0) return;
    auto &transfer=m_missionTransfers[systemId]; const auto &peer=m_peers[row];
    mavlink_message_t message{}; mavlink_status_t tx{}; tx.current_tx_seq=m_sequence++;
    if (transfer.operation==MissionTransfer::Download) {
        if (transfer.count<0) mavlink_msg_mission_request_list_pack_status(255,MAV_COMP_ID_MISSIONPLANNER,&tx,&message,systemId,peer.componentId,MAV_MISSION_TYPE_MISSION);
        else mavlink_msg_mission_request_int_pack_status(255,MAV_COMP_ID_MISSIONPLANNER,&tx,&message,systemId,peer.componentId,transfer.next,MAV_MISSION_TYPE_MISSION);
    } else if (transfer.operation==MissionTransfer::Clear)
        mavlink_msg_mission_clear_all_pack_status(255,MAV_COMP_ID_MISSIONPLANNER,&tx,&message,systemId,peer.componentId,MAV_MISSION_TYPE_MISSION);
    else if (transfer.operation==MissionTransfer::Upload) {
        if (transfer.next<0) mavlink_msg_mission_count_pack_status(255,MAV_COMP_ID_MISSIONPLANNER,&tx,&message,systemId,peer.componentId,transfer.items.size(),MAV_MISSION_TYPE_MISSION,0);
        else {
            auto command=transfer.items[transfer.next]; command.target_system=systemId; command.target_component=peer.componentId;
            if (transfer.legacyRequest) {
                mavlink_mission_item_t legacy{};
                legacy.target_system=systemId; legacy.target_component=peer.componentId; legacy.seq=command.seq;
                legacy.command=command.command; legacy.current=command.current; legacy.autocontinue=command.autocontinue;
                legacy.frame=command.frame==MAV_FRAME_GLOBAL_RELATIVE_ALT_INT ? MAV_FRAME_GLOBAL_RELATIVE_ALT : command.frame;
                legacy.param1=command.param1; legacy.param2=command.param2; legacy.param3=command.param3; legacy.param4=command.param4;
                const bool global=command.frame==MAV_FRAME_GLOBAL || command.frame==MAV_FRAME_GLOBAL_INT
                    || command.frame==MAV_FRAME_GLOBAL_RELATIVE_ALT || command.frame==MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
                    || command.frame==MAV_FRAME_GLOBAL_TERRAIN_ALT || command.frame==MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;
                legacy.x=global ? command.x/1e7 : command.x; legacy.y=global ? command.y/1e7 : command.y;
                legacy.z=command.z; legacy.mission_type=MAV_MISSION_TYPE_MISSION;
                mavlink_msg_mission_item_encode_status(255,MAV_COMP_ID_MISSIONPLANNER,&tx,&message,&legacy);
            } else mavlink_msg_mission_item_int_encode_status(255,MAV_COMP_ID_MISSIONPLANNER,&tx,&message,&command);
        }
    } else return;
    if (!send(message,peer)) { transfer.operation=MissionTransfer::Failed; transfer.error="send failed"; }
    transfer.retryAt=m_clock.elapsed()+1500; ++transfer.attempts;
}

bool VehicleLink::receiveMission(Peer &peer,const mavlink_message_t &message) {
    if (!m_planActive || !m_missionTransfers.contains(peer.systemId)) return false;
    auto &transfer=m_missionTransfers[peer.systemId];
    if (transfer.operation==MissionTransfer::Done || transfer.operation==MissionTransfer::Failed) return false;
    auto addressed=[](int system,int component,int type) { return (!system || system==255) && (!component || component==MAV_COMP_ID_MISSIONPLANNER) && type==MAV_MISSION_TYPE_MISSION; };
    if (message.msgid==MAVLINK_MSG_ID_MISSION_COUNT && transfer.operation==MissionTransfer::Download) {
        mavlink_mission_count_t count{}; mavlink_msg_mission_count_decode(&message,&count);
        if (!addressed(count.target_system,count.target_component,count.mission_type)) return true;
        if (count.count>2000) { transfer.operation=MissionTransfer::Failed; transfer.error="mission too large"; }
        else { transfer.count=count.count; peer.missionCount=std::max(0,int(count.count)-(peer.autopilot==MAV_AUTOPILOT_ARDUPILOTMEGA ? 1 : 0)); transfer.next=0; transfer.attempts=0;
            if (!count.count) {
                transfer.operation=MissionTransfer::Done;
                mavlink_message_t reply{}; mavlink_status_t tx{}; tx.current_tx_seq=m_sequence++;
                mavlink_msg_mission_ack_pack_status(255,MAV_COMP_ID_MISSIONPLANNER,&tx,&reply,peer.systemId,peer.componentId,MAV_MISSION_ACCEPTED,MAV_MISSION_TYPE_MISSION,0); send(reply,peer);
            } else sendMissionTransfer(peer.systemId); }
    } else if ((message.msgid==MAVLINK_MSG_ID_MISSION_ITEM_INT || message.msgid==MAVLINK_MSG_ID_MISSION_ITEM) && transfer.operation==MissionTransfer::Download) {
        mavlink_mission_item_int_t command{};
        if (message.msgid==MAVLINK_MSG_ID_MISSION_ITEM_INT) mavlink_msg_mission_item_int_decode(&message,&command);
        else {
            mavlink_mission_item_t legacy{}; mavlink_msg_mission_item_decode(&message,&legacy);
            command.target_system=legacy.target_system; command.target_component=legacy.target_component; command.mission_type=legacy.mission_type;
            command.seq=legacy.seq; command.frame=legacy.frame; command.command=legacy.command; command.current=legacy.current; command.autocontinue=legacy.autocontinue;
            command.param1=legacy.param1; command.param2=legacy.param2; command.param3=legacy.param3; command.param4=legacy.param4;
            const bool global=legacy.frame==MAV_FRAME_GLOBAL || legacy.frame==MAV_FRAME_GLOBAL_INT
                || legacy.frame==MAV_FRAME_GLOBAL_RELATIVE_ALT || legacy.frame==MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
                || legacy.frame==MAV_FRAME_GLOBAL_TERRAIN_ALT || legacy.frame==MAV_FRAME_GLOBAL_TERRAIN_ALT_INT;
            command.x=qint32(std::llround(double(legacy.x)*(global ? 1e7 : 1)));
            command.y=qint32(std::llround(double(legacy.y)*(global ? 1e7 : 1))); command.z=legacy.z;
        }
        if (!addressed(command.target_system,command.target_component,command.mission_type) || command.seq!=transfer.next || transfer.count<0) return true;
        transfer.received.insert(command.seq,command); ++transfer.next; transfer.attempts=0;
        if (transfer.next==transfer.count) {
            transfer.items=transfer.received.values(); transfer.operation=MissionTransfer::Done;
            mavlink_message_t reply{}; mavlink_status_t tx{}; tx.current_tx_seq=m_sequence++;
            mavlink_msg_mission_ack_pack_status(255,MAV_COMP_ID_MISSIONPLANNER,&tx,&reply,peer.systemId,peer.componentId,MAV_MISSION_ACCEPTED,MAV_MISSION_TYPE_MISSION,0); send(reply,peer);
        } else sendMissionTransfer(peer.systemId);
    } else if ((message.msgid==MAVLINK_MSG_ID_MISSION_REQUEST_INT || message.msgid==MAVLINK_MSG_ID_MISSION_REQUEST) && transfer.operation==MissionTransfer::Upload) {
        mavlink_mission_request_int_t request{};
        if (message.msgid==MAVLINK_MSG_ID_MISSION_REQUEST_INT) mavlink_msg_mission_request_int_decode(&message,&request);
        else { mavlink_mission_request_t old{}; mavlink_msg_mission_request_decode(&message,&old); request.seq=old.seq; request.target_system=old.target_system; request.target_component=old.target_component; request.mission_type=old.mission_type; }
        if (!addressed(request.target_system,request.target_component,request.mission_type) || request.seq>=transfer.items.size()) return true;
        transfer.next=request.seq; transfer.received.insert(request.seq,transfer.items[request.seq]);
        transfer.legacyRequest=message.msgid==MAVLINK_MSG_ID_MISSION_REQUEST;
        transfer.attempts=0; sendMissionTransfer(peer.systemId);
    } else if (message.msgid==MAVLINK_MSG_ID_MISSION_ACK) {
        mavlink_mission_ack_t ack{}; mavlink_msg_mission_ack_decode(&message,&ack);
        if (!addressed(ack.target_system,ack.target_component,ack.mission_type)) return true;
        if (ack.type!=MAV_MISSION_ACCEPTED) { transfer.operation=MissionTransfer::Failed; transfer.error=QString("rejected (%1)").arg(ack.type); }
        else if (transfer.operation==MissionTransfer::Clear || (transfer.operation==MissionTransfer::Upload && transfer.received.size()==transfer.items.size())) {
            peer.missionCount=transfer.operation==MissionTransfer::Clear ? 0 : std::max(0,int(transfer.items.size())-(peer.autopilot==MAV_AUTOPILOT_ARDUPILOTMEGA ? 1 : 0)); transfer.operation=MissionTransfer::Done;
        }
    } else return false;
    if (!missionBusy()) finishMissionDownloads();
    emit planChanged(); return true;
}

void VehicleLink::finishMissionDownloads() {
    QStringList failed;
    for (auto it=m_missionTransfers.cbegin();it!=m_missionTransfers.cend();++it)
        if (it->operation==MissionTransfer::Failed) failed.append(nameForSystem(it.key())+": "+it->error);
    if (!failed.isEmpty()) {
        if (m_planPoints.isEmpty()) {
            for (auto it=m_missionTransfers.cbegin();it!=m_missionTransfers.cend();++it)
                if (it->operation==MissionTransfer::Done) m_plannedMissions[it.key()]=it->items;
            m_loadedPlan=true;
        }
        m_planValid=false; m_planStatus="Mission transfer failed · "+failed.join("; "); return;
    }
    if (m_planPoints.isEmpty()) {
        m_plannedMissions.clear();
        bool any=false;
        for (auto it=m_missionTransfers.cbegin();it!=m_missionTransfers.cend();++it) {
            m_plannedMissions[it.key()]=it->items;
            const int row=rowForSystem(it.key());
            for (const auto &i:it->items)
                if (!(i.seq==0 && i.command==MAV_CMD_NAV_WAYPOINT && row>=0 && m_peers[row].autopilot==MAV_AUTOPILOT_ARDUPILOTMEGA)) any=true;
        }
        m_loadedPlan=any;
        if (any) {
            // Display the center of corresponding navigation items without modifying downloaded missions.
            const auto paths=planPaths(); int count=2000;
            for (const auto &p:paths) count=std::min(count,int(p.toMap().value("path").toList().size()));
            for (int n=0;n<count;++n) {
                double x=0,y=0,z=0;
                for (const auto &p:paths) {
                    const auto point=p.toMap().value("path").toList()[n].toMap();
                    const double lat=point.value("latitude").toDouble()*pi/180,lon=point.value("longitude").toDouble()*pi/180;
                    x+=std::cos(lat)*std::cos(lon); y+=std::cos(lat)*std::sin(lon); z+=std::sin(lat);
                }
                m_planPoints.append({std::atan2(z,std::hypot(x,y))*180/pi,std::atan2(y,x)*180/pi});
            }
            m_planValid=false; m_planStatus="Loaded original missions unchanged. Clear to create a checked swarm route.";
        }
        else rebuildPlan();
    } else m_planStatus="Missions saved on all planning vehicles.";
}

void VehicleLink::missionTick() {
    if (!m_planActive) return;
    for (const auto &v:m_planVehicles) {
        const int row=rowForSystem(v.id);
        if ((row<0 || m_peers[row].session!=v.session) && m_planValid) {
            m_planValid=false; m_planStatus="A planning vehicle disconnected. Return to Fly and select again."; emit planChanged();
        }
    }
    if (!missionBusy()) return;
    for (auto it=m_missionTransfers.begin();it!=m_missionTransfers.end();++it) {
        if (it->operation==MissionTransfer::Done || it->operation==MissionTransfer::Failed) continue;
        const int row=rowForSystem(it.key());
        bool valid=row>=0;
        for (const auto &v:m_planVehicles) if (v.id==it.key() && valid) valid=m_peers[row].session==v.session;
        if (!valid || (m_clock.elapsed()>=it->retryAt && it->attempts>=5)) {
            it->operation=MissionTransfer::Failed; it->error=valid ? "timed out" : "disconnected"; emit planChanged();
        } else if (m_clock.elapsed()>=it->retryAt) sendMissionTransfer(it.key());
    }
    if (!missionBusy()) { finishMissionDownloads(); emit planChanged(); }
}

void VehicleLink::clearPlanMissions() {
    if (!m_planActive || missionBusy()) return;
    for (const auto &v:m_planVehicles) { const int row=rowForSystem(v.id); if (row<0 || m_peers[row].session!=v.session || m_peers[row].armed) { emit notice("Clear requires all planning vehicles connected and disarmed."); return; } }
    m_planPoints.clear(); m_plannedMissions.clear(); m_loadedPlan=false; m_planValid=false;
    m_planStatus="Clearing missions…"; m_missionTransfers.clear();
    for (const auto &v:m_planVehicles) { MissionTransfer t; t.operation=MissionTransfer::Clear; m_missionTransfers.insert(v.id,t); sendMissionTransfer(v.id); clearNavigation(v.id); }
    emit planChanged();
}
void VehicleLink::uploadPlanMissions() {
    if (!m_planActive || !m_planValid || m_loadedPlan || missionBusy()) return;
    for (const auto &v:m_planVehicles) { const int row=rowForSystem(v.id); if (row<0 || m_peers[row].session!=v.session || m_peers[row].armed) { emit notice("Upload requires all planning vehicles connected and disarmed."); return; } }
    m_missionTransfers.clear(); m_planStatus="Uploading missions…";
    for (const auto &v:m_planVehicles) { MissionTransfer t; t.operation=MissionTransfer::Upload; t.next=-1; t.items=m_plannedMissions.value(v.id); m_missionTransfers.insert(v.id,t); sendMissionTransfer(v.id); clearNavigation(v.id); }
    emit planChanged();
}
