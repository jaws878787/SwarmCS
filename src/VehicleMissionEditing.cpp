#include "VehicleLink.h"
#include <QGeoCoordinate>
#include <QCryptographicHash>
#include <QDataStream>
#include <cmath>
#include <algorithm>

namespace {
constexpr double pi=3.14159265358979323846;
bool navigation(const mavlink_mission_item_int_t &i) {
    return i.command==MAV_CMD_NAV_WAYPOINT || i.command==MAV_CMD_NAV_TAKEOFF || i.command==MAV_CMD_NAV_LAND;
}
}

void VehicleLink::setPlanVehicleId(int systemId) {
    for (const auto &v:m_planVehicles) if (v.id==systemId) {
        if (m_planVehicleId==systemId) return;
        m_planVehicleId=systemId;
        if (m_loadedPlan) m_selectedPlanItem=-1;
        emit planChanged(); emit planSelectionChanged(); return;
    }
}
void VehicleLink::selectPlanItem(int row) {
    if (!m_planActive || row<0 || row>=planItems().size()) return;
    m_selectedPlanItem=row; emit planSelectionChanged();
}
QVariantList VehicleLink::planMarkers() const {
    QVariantList result;
    if (!m_loadedPlan) {
        for (int n=0;n<m_planPoints.size();++n) result.append(QVariantMap{{"latitude",m_planPoints[n].latitude},
            {"longitude",m_planPoints[n].longitude},{"row",n},{"label",n==0 ? QString("T") : QString::number(n)},{"draggable",n>0}});
    } else {
        const auto rows=planItems();
        const auto mission=m_plannedMissions.value(m_planVehicleId);
        for (int n=0;n<rows.size();++n) {
            const auto &i=mission[rows[n].toMap().value("sequence").toInt()];
            if (!navigation(i)) continue;
            double lat=i.x/1e7,lon=i.y/1e7;
            if (i.command==MAV_CMD_NAV_TAKEOFF && !i.x && !i.y)
                for (const auto &v:m_planVehicles) if (v.id==m_planVehicleId) { lat=v.latitude; lon=v.longitude; }
            if (!QGeoCoordinate(lat,lon).isValid()) continue;
            result.append(QVariantMap{{"latitude",lat},{"longitude",lon},{"row",n},{"label",QString::number(n+1)},
                {"draggable",i.command!=MAV_CMD_NAV_TAKEOFF}});
        }
    }
    return result;
}
void VehicleLink::movePlanWaypoint(int row,double latitude,double longitude) {
    if (!m_planActive || missionBusy() || !QGeoCoordinate(latitude,longitude).isValid()) return;
    if (!m_loadedPlan) {
        if (row<=0 || row>=m_planPoints.size()) return;
        m_planPoints[row].latitude=latitude; m_planPoints[row].longitude=longitude;
        m_selectedPlanItem=row; rebuildPlan(); emit planSelectionChanged();
    } else {
        const auto rows=planItems(); if (row<0 || row>=rows.size()) return;
        auto &i=m_plannedMissions[m_planVehicleId][rows[row].toMap().value("sequence").toInt()];
        if (i.command!=MAV_CMD_NAV_WAYPOINT && i.command!=MAV_CMD_NAV_LAND) return;
        i.x=qint32(std::llround(latitude*1e7)); i.y=qint32(std::llround(longitude*1e7));
        m_selectedPlanItem=row; m_planValid=true; m_planStatus="Edited imported mission · swarm separation not checked.";
        emit planChanged(); emit planSelectionChanged();
    }
}
void VehicleLink::addPlanRtl() {
    if (!m_planActive || missionBusy()) return;
    if (!m_loadedPlan) {
        m_planRtl=true; m_selectedPlanItem=m_planPoints.size(); rebuildPlan(); emit planSelectionChanged();
    } else {
        auto &mission=m_plannedMissions[m_planVehicleId];
        if (mission.isEmpty() || mission.last().command!=MAV_CMD_NAV_RETURN_TO_LAUNCH) {
            mavlink_mission_item_int_t rtl{}; rtl.command=MAV_CMD_NAV_RETURN_TO_LAUNCH; rtl.frame=MAV_FRAME_MISSION;
            rtl.autocontinue=1; rtl.seq=mission.size(); mission.append(rtl);
        }
        m_planValid=true; m_planStatus="Edited imported mission · swarm separation not checked.";
        emit planChanged(); selectPlanItem(planItems().size()-1);
    }
}

QString VehicleLink::missionPlanKey() const {
    QStringList ids; for (const auto &v:m_planVehicles) ids.append(QString::number(v.id));
    return ids.join('-');
}
QString VehicleLink::missionSignature(int systemId,const QList<mavlink_mission_item_int_t> &items) const {
    QByteArray bytes; QDataStream stream(&bytes,QIODevice::WriteOnly);
    const int row=rowForSystem(systemId);
    auto number=[](double value) { return std::isfinite(value) && std::abs(value)<1e12 ? qint64(std::llround(value*1000)) : std::numeric_limits<qint64>::max(); };
    for (const auto &i:items) {
        if (i.seq==0 && i.command==MAV_CMD_NAV_WAYPOINT && row>=0 && m_peers[row].autopilot==MAV_AUTOPILOT_ARDUPILOTMEGA) continue;
        const quint8 frame=i.frame==MAV_FRAME_GLOBAL_INT ? MAV_FRAME_GLOBAL : i.frame==MAV_FRAME_GLOBAL_RELATIVE_ALT_INT ? MAV_FRAME_GLOBAL_RELATIVE_ALT
            : i.frame==MAV_FRAME_GLOBAL_TERRAIN_ALT_INT ? MAV_FRAME_GLOBAL_TERRAIN_ALT : i.frame;
        stream<<i.command<<frame<<i.autocontinue<<number(i.param1)<<number(i.param2)<<number(i.param3)<<number(i.param4)<<i.x<<i.y<<number(i.z);
    }
    return QString::fromLatin1(QCryptographicHash::hash(bytes,QCryptographicHash::Sha256).toHex());
}
void VehicleLink::savePlanMetadata() {
    QVariantList points,vehicles; QVariantMap signatures,sequences,missions;
    for (const auto &p:m_planPoints) points.append(QVariantMap{{"latitude",p.latitude},{"longitude",p.longitude},{"altitude",p.altitude},{"speed",p.speed}});
    for (const auto &v:m_planVehicles) {
        vehicles.append(QVariantMap{{"id",v.id},{"latitude",v.latitude},{"longitude",v.longitude},{"homeAltitude",v.homeAltitude}});
        signatures[QString::number(v.id)]=missionSignature(v.id,m_plannedMissions.value(v.id));
        QVariantList indexes; for (int seq:m_pointSequences.value(v.id)) indexes.append(seq);
        sequences[QString::number(v.id)]=indexes;
        QVariantList expected;
        for (const auto &i:m_plannedMissions.value(v.id)) expected.append(QVariantMap{{"command",int(i.command)},{"frame",int(i.frame)},
            {"x",i.x},{"y",i.y},{"z",double(i.z)},{"p1",double(i.param1)},{"p2",double(i.param2)},{"p3",double(i.param3)},{"p4",double(i.param4)}});
        missions[QString::number(v.id)]=expected;
    }
    m_settings->setValue("missionPlans/"+missionPlanKey(),QVariantMap{{"version",2},{"geometryVersion",m_planGeometryVersion},{"points",points},{"vehicles",vehicles},
        {"signatures",signatures},{"missions",missions},{"sequences",sequences},{"separation",m_minimumSeparation},{"rtl",m_planRtl}});
    m_settings->sync();
}
bool VehicleLink::restorePlanMetadata() {
    const auto saved=m_settings->value("missionPlans/"+missionPlanKey()).toMap();
    if ((saved.value("version").toInt()!=1 && saved.value("version").toInt()!=2) || saved.value("points").toList().isEmpty()) return false;
    const auto signatures=saved.value("signatures").toMap();
    auto frame=[](int value) { return value==MAV_FRAME_GLOBAL_INT ? MAV_FRAME_GLOBAL : value==MAV_FRAME_GLOBAL_RELATIVE_ALT_INT ? MAV_FRAME_GLOBAL_RELATIVE_ALT
        : value==MAV_FRAME_GLOBAL_TERRAIN_ALT_INT ? MAV_FRAME_GLOBAL_TERRAIN_ALT : value; };
    for (const auto &v:m_planVehicles) {
        if (signatures.value(QString::number(v.id)).toString()==missionSignature(v.id,m_plannedMissions.value(v.id))) continue;
        const auto expected=saved.value("missions").toMap().value(QString::number(v.id)).toList();
        const auto actual=m_plannedMissions.value(v.id);
        if (expected.isEmpty() || expected.size()!=actual.size()) return false;
        const int row=rowForSystem(v.id);
        const bool ardupilot=row>=0 && m_peers[row].autopilot==MAV_AUTOPILOT_ARDUPILOTMEGA;
        for (int seq=0;seq<actual.size();++seq) {
            const auto e=expected[seq].toMap(); const auto &a=actual[seq];
            if (ardupilot && seq==0 && a.command==MAV_CMD_NAV_WAYPOINT) continue;
            if (e.value("command").toInt()!=a.command || (navigation(a) && frame(e.value("frame").toInt())!=frame(a.frame))) return false;
            const double params[]={a.param1,a.param2,a.param3,a.param4,a.z};
            const QString keys[]={"p1","p2","p3","p4","z"};
            for (int n=0;n<5;++n) {
                if (ardupilot && a.command==MAV_CMD_NAV_WAYPOINT && n==1) continue; // Copter does not retain acceptance-radius param2.
                const double value=e.value(keys[n]).toDouble();
                if ((std::isfinite(value)!=std::isfinite(params[n])) || (std::isfinite(value) && std::abs(value-params[n])>0.001)) return false;
            }
            // Some firmware stores coordinates through float precision. Account for
            // that quantization while still requiring every command and parameter to match.
            if (navigation(a)) {
                if (QGeoCoordinate(e.value("x").toInt()/1e7,e.value("y").toInt()/1e7).distanceTo(QGeoCoordinate(a.x/1e7,a.y/1e7))>2) return false;
            } else if (e.value("x").toInt()!=a.x || e.value("y").toInt()!=a.y) return false;
        }
    }
    m_planPoints.clear();
    for (const auto &entry:saved.value("points").toList()) {
        const auto p=entry.toMap(); m_planPoints.append({p.value("latitude").toDouble(),p.value("longitude").toDouble(),p.value("altitude").toDouble(),p.value("speed").toDouble()});
    }
    for (auto &v:m_planVehicles) for (const auto &entry:saved.value("vehicles").toList()) {
        const auto start=entry.toMap(); if (start.value("id").toInt()!=v.id) continue;
        v.latitude=start.value("latitude").toDouble(); v.longitude=start.value("longitude").toDouble(); v.homeAltitude=start.value("homeAltitude").toDouble();
    }
    m_pointSequences.clear();
    for (const auto &v:m_planVehicles) for (const auto &seq:saved.value("sequences").toMap().value(QString::number(v.id)).toList()) m_pointSequences[v.id].append(seq.toInt());
    m_minimumSeparation=saved.value("separation",5).toDouble(); m_planRtl=saved.value("rtl").toBool(); m_planValid=true;
    m_planGeometryVersion=saved.value("geometryVersion",1).toInt();
    return true;
}

bool VehicleLink::recoverGeneratedPlan() {
    // Older uploads have no sidecar: align motion stages including explicit holds,
    // rather than pairing NAV_WAYPOINT indexes, which differ between vehicles.
    struct Stage { double lat,lon,alt,speed; int sequence; };
    QMap<int,QList<Stage>> stages; QMap<int,Stage> starts;
    bool rtl=false;
    for (const auto &v:m_planVehicles) {
        Stage last{}; double speed=5; bool takeoff=false;
        for (const auto &i:m_plannedMissions.value(v.id)) {
            const int row=rowForSystem(v.id);
            if (i.seq==0 && row>=0 && m_peers[row].autopilot==MAV_AUTOPILOT_ARDUPILOTMEGA && i.command==MAV_CMD_NAV_WAYPOINT) continue;
            if (i.command==MAV_CMD_NAV_TAKEOFF && !takeoff) {
                last={i.x ? i.x/1e7 : v.latitude,i.y ? i.y/1e7 : v.longitude,i.z,speed,int(i.seq)};
                starts[v.id]=last; takeoff=true;
            } else if (i.command==MAV_CMD_DO_CHANGE_SPEED) { if (i.param1==1) speed=i.param2; }
            else if (i.command==MAV_CMD_NAV_WAYPOINT && takeoff) { last={i.x/1e7,i.y/1e7,i.z,speed,int(i.seq)}; stages[v.id].append(last); }
            else if (i.command==MAV_CMD_NAV_DELAY && takeoff) stages[v.id].append(last);
            else if (i.command==MAV_CMD_NAV_RETURN_TO_LAUNCH) rtl=true;
            else return false;
        }
        if (!takeoff) return false;
    }
    const int count=stages.value(m_planVehicles.first().id).size();
    const bool oldGeometry=m_planVehicles.size()>1 && count%3==0;
    const bool laneGeometry=m_planVehicles.size()>1 && count>=5 && (count-2)%3==0;
    if (!count || (m_planVehicles.size()>1 && !oldGeometry && !laneGeometry)) return false;
    QList<int> indexes{-1};
    if (m_planVehicles.size()==1) for (int n=0;n<count;++n) indexes.append(n);
    else if (oldGeometry) for (int n=2;n<count;n+=3) indexes.append(n);
    else {
        const int points=(count-2)/3; int cursor=0;
        for (int p=0;p<points;++p) {
            ++cursor; if (p==0) ++cursor; ++cursor;
            if (p==points-1) ++cursor;
            indexes.append(cursor++);
        }
    }
    for (const auto &v:m_planVehicles) {
        const auto list=stages.value(v.id); if (list.size()!=count) return false;
        if (m_planVehicles.size()>1) for (int p=1;p<indexes.size();++p) {
            const int begin=indexes[p-1]+1,end=indexes[p];
            const auto prev=begin ? list[begin-1] : starts.value(v.id);
            const int transit=begin+1+(laneGeometry && p==1 ? 1 : 0);
            const int settleFrom=end-1;
            if (QGeoCoordinate(prev.lat,prev.lon).distanceTo(QGeoCoordinate(list[begin].lat,list[begin].lon))>0.5
                || QGeoCoordinate(list[settleFrom].lat,list[settleFrom].lon).distanceTo(QGeoCoordinate(list[end].lat,list[end].lon))>0.5
                || std::abs((list[begin].alt-prev.alt)-(list[transit].alt-list[end].alt))>0.2) return false;
        }
    }
    QList<PlanPoint> points; QMap<int,QList<int>> sequences;
    for (int index:indexes) {
        double x=0,y=0,z=0,alt=0,speed=0;
        for (const auto &v:m_planVehicles) {
            const auto s=index<0 ? starts.value(v.id) : stages.value(v.id)[index];
            const double lat=s.lat*pi/180,lon=s.lon*pi/180;
            x+=std::cos(lat)*std::cos(lon); y+=std::cos(lat)*std::sin(lon); z+=std::sin(lat);
            alt+=s.alt/m_planVehicles.size(); speed=std::max(speed,s.speed); sequences[v.id].append(s.sequence);
        }
        points.append({std::atan2(z,std::hypot(x,y))*180/pi,std::atan2(y,x)*180/pi,alt,speed});
    }
    for (auto &v:m_planVehicles) { const auto s=starts.value(v.id); v.latitude=s.lat; v.longitude=s.lon; }
    QList<double> layers;
    for (const auto &v:m_planVehicles) layers.append(stages.value(v.id).first().alt-starts.value(v.id).alt);
    std::sort(layers.begin(),layers.end());
    double spacing=std::numeric_limits<double>::infinity();
    for (int n=1;n<layers.size();++n) if (layers[n]-layers[n-1]>0.01) spacing=std::min(spacing,layers[n]-layers[n-1]);
    const double separation=spacing-(oldGeometry ? 1 : 3);
    if (separation>=0.3 && separation<=1000) m_minimumSeparation=separation;
    m_planPoints=points; m_pointSequences=sequences; m_planRtl=rtl; m_planValid=true;
    m_planGeometryVersion=oldGeometry ? 1 : 3;
    return true;
}
