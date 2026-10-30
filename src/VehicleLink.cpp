#include "VehicleLink.h"
#include <QNetworkDatagram>
#include <cmath>
#include <algorithm>
#include <QSet>

namespace {
QString commandResult(quint8 result) {
    switch (result) {
    case MAV_RESULT_TEMPORARILY_REJECTED: return "rejected";
    case MAV_RESULT_DENIED: return "denied";
    case MAV_RESULT_UNSUPPORTED: return "unsupported";
    case MAV_RESULT_FAILED: return "failed";
    case MAV_RESULT_CANCELLED: return "cancelled";
    default: return QString("rejected (%1)").arg(result);
    }
}
}

VehicleLink::VehicleLink(QObject *parent, const QString &settingsFile, quint16 port)
    : QAbstractListModel(parent),
      m_settings(settingsFile.isEmpty() ? std::make_unique<QSettings>()
          : std::make_unique<QSettings>(settingsFile, QSettings::IniFormat)), m_port(port) {
    m_resolvedNames = resolvedNames();
    const double savedPlanSpeed=m_settings->value("planning/defaultSpeedMetersPerSecond",8.9408).toDouble();
    if (std::isfinite(savedPlanSpeed) && savedPlanSpeed>=0.1 && savedPlanSpeed<=30) m_defaultPlanSpeed=savedPlanSpeed;
    const double savedAltitude = m_settings->value("takeoff/altitudeMeters",15.24).toDouble();
    if (std::isfinite(savedAltitude) && savedAltitude > 0 && savedAltitude <= 10000) m_takeoffMeters = savedAltitude;
    m_takeoffUnit = m_settings->value("takeoff/unit","ft").toString() == "m" ? "m" : "ft";
    m_velocityUnit = m_settings->value("display/velocityUnit", "kph").toString() == "mph" ? "mph" : "kph";
    m_groups = {"Ungrouped"};
    for (const auto &group : m_settings->value("vehicleGroups/list").toStringList())
        if (!group.trimmed().isEmpty() && !m_groups.contains(group, Qt::CaseInsensitive)) m_groups.append(group);
    m_clock.start();
    m_timer.setInterval(1000);
    connect(&m_socket, &QUdpSocket::readyRead, this, &VehicleLink::receive);
    connect(&m_timer, &QTimer::timeout, this, &VehicleLink::tick);
    m_parameterTimer.setInterval(250);
    connect(&m_parameterTimer, &QTimer::timeout, this, &VehicleLink::parameterTick);
    m_parameterTimer.start();
}

int VehicleLink::rowCount(const QModelIndex &parent) const { return parent.isValid() ? 0 : m_peers.size(); }

QHash<int, QByteArray> VehicleLink::roleNames() const {
    return {{SystemIdRole, "systemId"}, {VehicleNameRole, "vehicleName"}, {FirmwareRole, "firmware"},
        {ArmedRole, "armed"}, {FlightModeRole, "flightMode"}, {AltitudeRole, "altitude"},
        {VelocityRole, "velocity"}, {VehicleGroupRole, "vehicleGroup"},
        {LatitudeRole, "latitude"}, {LongitudeRole, "longitude"}, {HeadingRole, "heading"},
        {PositionValidRole, "positionValid"}, {IconIdRole, "iconId"},
        {CommandPendingRole, "commandPending"}, {CommandStatusRole, "commandStatus"}, {ErrorMessageRole,"errorMessage"}};
}

QVariant VehicleLink::data(const QModelIndex &index, int role) const {
    if (!index.isValid() || index.row() < 0 || index.row() >= m_peers.size()) return {};
    const auto &peer = m_peers[index.row()];
    switch (role) {
    case SystemIdRole: return peer.systemId;
    case VehicleNameRole: return nameForSystem(peer.systemId);
    case FirmwareRole: {
        const QString name = peer.autopilot == MAV_AUTOPILOT_PX4 ? "PX4" : isPlane(peer) ? "ArduPlane" : "ArduCopter";
        return peer.firmwareVersion.isEmpty() ? name : name + " " + peer.firmwareVersion;
    }
    case ArmedRole: return peer.armed;
    case FlightModeRole: return peer.flightMode;
    case AltitudeRole: return peer.altitudeTime >= 0 && m_clock.elapsed() - peer.altitudeTime <= 5000
        ? peer.altitude : std::numeric_limits<double>::quiet_NaN();
    case VelocityRole: return peer.velocityTime >= 0 && m_clock.elapsed() - peer.velocityTime <= 5000
        ? peer.velocity : std::numeric_limits<double>::quiet_NaN();
    case VehicleGroupRole: return groupForSystem(peer.systemId);
    case LatitudeRole: return peer.latitude;
    case LongitudeRole: return peer.longitude;
    case HeadingRole: return peer.heading;
    case PositionValidRole: return positionValid(peer);
    case IconIdRole: return iconForPeer(peer);
    case CommandPendingRole: return peer.pendingCommand != 0;
    case CommandStatusRole: return peer.commandStatus;
    case ErrorMessageRole: return peer.errorMessage;
    default: return {};
    }
}

int VehicleLink::rowForSystem(int systemId) const {
    for (int row = 0; row < m_peers.size(); ++row)
        if (m_peers[row].systemId == systemId) return row;
    return -1;
}

QString VehicleLink::baseNameForSystem(int systemId) const {
    const auto saved = m_settings->value(QString("vehicleNames/%1").arg(systemId)).toString().trimmed();
    return saved.isEmpty() ? QString("Vehicle %1").arg(systemId) : saved;
}

QMap<int, QString> VehicleLink::resolvedNames() const {
    QMap<int, QString> bases, resolved;
    QHash<QString,int> counts, suffixes;
    QSet<QString> reserved, used;
    // Include stored offline identities and default names so labels remain unique on reconnect.
    for (int id = 1; id <= 255; ++id) {
        const auto base = baseNameForSystem(id);
        bases[id] = base;
        ++counts[base.toCaseFolded()];
        reserved.insert(base.toCaseFolded());
    }
    for (auto it = bases.cbegin(); it != bases.cend(); ++it) {
        const auto key = it.value().toCaseFolded();
        auto name = it.value();
        if (counts[key] > 1) {
            do { name = it.value() + " " + QString::number(++suffixes[key]); }
            while (reserved.contains(name.toCaseFolded()) || used.contains(name.toCaseFolded()));
        }
        used.insert(name.toCaseFolded());
        resolved[it.key()] = name;
    }
    return resolved;
}

QString VehicleLink::nameForSystem(int systemId) const {
    return m_resolvedNames.value(systemId, QString("Vehicle %1").arg(systemId));
}

bool VehicleLink::isPlane(const Peer &peer) const {
    return peer.autopilot == MAV_AUTOPILOT_ARDUPILOTMEGA &&
        (peer.vehicleType == MAV_TYPE_FIXED_WING || (peer.vehicleType >= 19 && peer.vehicleType <= 25));
}

bool VehicleLink::positionValid(const Peer &peer) const {
    return peer.positionTime >= 0 && m_clock.elapsed() - peer.positionTime <= 5000
        && std::isfinite(peer.latitude) && std::isfinite(peer.longitude);
}

QString VehicleLink::iconForPeer(const Peer &peer) const {
    const auto saved = m_settings->value(QString("vehicleIcons/%1").arg(peer.systemId)).toString();
    const QStringList allowed = isPlane(peer) ? QStringList{"fixed","delta","wing"}
        : QStringList{"quad","hex","octo","bullet"};
    return allowed.contains(saved) ? saved : allowed.first();
}

QString VehicleLink::modeName(const Peer &peer) const {
    if (peer.standardMode) {
        for (const auto &mode : peer.reportedModes)
            if (mode.standardMode == peer.standardMode) return mode.name;
    }
    for (const auto &mode : peer.reportedModes)
        if (mode.id == peer.customMode && (!mode.standardMode || mode.id) && !mode.name.isEmpty()) return mode.name;
    for (const auto &mode : firmwareModes(peer.autopilot, isPlane(peer)))
        if (mode.id == peer.customMode) return mode.name;
    return peer.standardMode ? standardModeName(peer.standardMode) : QString("Mode %1").arg(peer.customMode);
}

bool VehicleLink::reportedModesComplete(const Peer &peer) const {
    return peer.expectedModes > 0 && peer.reportedModes.size() == peer.expectedModes;
}

QVariantMap VehicleLink::vehicleDetails(int systemId) const {
    const int row = rowForSystem(systemId);
    QVariantMap result{{"connected", row >= 0}, {"nameInput", baseNameForSystem(systemId)}};
    if (row < 0) return result;
    const auto roles = roleNames();
    for (auto it = roles.cbegin(); it != roles.cend(); ++it)
        result.insert(QString::fromUtf8(it.value()), data(index(row), it.key()));
    result.insert("reportedModes", reportedModesComplete(m_peers[row]));
    result.insert("customMode", m_peers[row].customMode);
    result.insert("standardMode", m_peers[row].standardMode);
    return result;
}

QVariantList VehicleLink::iconOptions(int systemId) const {
    const int row = rowForSystem(systemId);
    if (row < 0) return {};
    const QList<QPair<QString,QString>> choices = isPlane(m_peers[row])
        ? QList<QPair<QString,QString>>{{"fixed","Fixed wing"},{"delta","Delta wing"},{"wing","Wing"}}
        : QList<QPair<QString,QString>>{{"quad","Quadcopter"},{"hex","Hexacopter"},{"octo","Octocopter"},{"bullet","Bullet drone"}};
    QVariantList result;
    for (const auto &choice : choices) result.append(QVariantMap{{"id",choice.first},{"name",choice.second}});
    return result;
}

bool VehicleLink::setVehicleIcon(int systemId, const QString &icon) {
    const auto choices = iconOptions(systemId);
    const bool valid = std::any_of(choices.cbegin(),choices.cend(),
        [&](const QVariant &entry) { return entry.toMap().value("id").toString() == icon; });
    if (!valid) return false;
    m_settings->setValue(QString("vehicleIcons/%1").arg(systemId), icon);
    m_settings->sync();
    const int row = rowForSystem(systemId);
    emit dataChanged(index(row),index(row),{IconIdRole});
    return m_settings->status() == QSettings::NoError;
}

QVariantList VehicleLink::availableModes(int systemId) const {
    const int row = rowForSystem(systemId);
    if (row < 0) return {};
    const auto &peer = m_peers[row];
    const auto options = reportedModesComplete(peer) ? peer.reportedModes.values()
        : firmwareModes(peer.autopilot,isPlane(peer));
    QVariantList result;
    for (const auto &option : options) result.append(QVariantMap{{"id",option.id},{"name",option.name},
        {"standardMode",option.standardMode},{"enabled",option.enabled}});
    return result;
}

void VehicleLink::refreshModes(int systemId) {
    const int row = rowForSystem(systemId);
    if (row < 0) return;
    m_peers[row].modeRequests = 0;
    requestModes(m_peers[row]);
}

void VehicleLink::requestModes(Peer &peer) {
    sendCommand(peer,MAV_CMD_REQUEST_MESSAGE,MAVLINK_MSG_ID_AVAILABLE_MODES,0);
    peer.modesRequestedAt = m_clock.elapsed();
    ++peer.modeRequests;
}

void VehicleLink::centerVehicle(int systemId) {
    selectVehicle(systemId,false);
}

void VehicleLink::toggleVelocityUnit() {
    m_velocityUnit = m_velocityUnit == "kph" ? "mph" : "kph";
    m_settings->setValue("display/velocityUnit",m_velocityUnit);
    m_settings->sync();
    emit velocityUnitChanged();
}

bool VehicleLink::sendCommand(Peer &peer, quint16 command, float p1, float p2, float p3) {
    mavlink_message_t message{};
    mavlink_status_t status{};
    status.current_tx_seq = m_sequence++;
    mavlink_msg_command_long_pack_status(255,MAV_COMP_ID_MISSIONPLANNER,&status,&message,
        peer.systemId,peer.componentId,command,0,p1,p2,p3,0,0,0,0);
    return send(message,peer);
}

bool VehicleLink::setArmed(int systemId, bool armed) {
    const int row = rowForSystem(systemId);
    if (row < 0) { emit notice("Vehicle is disconnected."); return false; }
    auto &peer = m_peers[row];
    if (peer.pendingCommand) { emit notice(nameForSystem(systemId) + ": command pending."); return false; }
    if (peer.armed == armed) return true;
    // param2 remains zero: never bypass flight-controller arming/disarming checks.
    if (!sendCommand(peer,MAV_CMD_COMPONENT_ARM_DISARM,armed ? 1 : 0,0)) return failVehicle(peer,"Arm command not sent.");
    peer.pendingCommand = MAV_CMD_COMPONENT_ARM_DISARM;
    peer.desiredArmed = armed;
    peer.commandDeadline = m_clock.elapsed() + 10000;
    peer.commandStatus = armed ? "Arming…" : "Disarming…";
    emit dataChanged(index(row),index(row),{CommandPendingRole,CommandStatusRole});
    emit notice(nameForSystem(systemId) + ": " + peer.commandStatus);
    return true;
}

bool VehicleLink::setFlightMode(int systemId, quint32 mode, int standardMode) {
    const int row = rowForSystem(systemId);
    if (row < 0) { emit notice("Vehicle is disconnected."); return false; }
    auto &peer = m_peers[row];
    if (peer.pendingCommand) { emit notice(nameForSystem(systemId) + ": command pending."); return false; }
    const auto options = availableModes(systemId);
    const bool valid = std::any_of(options.cbegin(),options.cend(),[&](const QVariant &entry) {
        const auto item = entry.toMap();
        return item.value("id").toUInt() == mode && item.value("standardMode").toInt() == standardMode
            && item.value("enabled").toBool();
    });
    if (!valid) return failVehicle(peer,"Mode unavailable.");
    if ((!standardMode && peer.customMode == mode) || (standardMode && peer.standardMode == standardMode)) return true;
    const quint16 command = standardMode ? MAV_CMD_DO_SET_STANDARD_MODE : MAV_CMD_DO_SET_MODE;
    const float p1 = standardMode ? standardMode : MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    const float p2 = standardMode ? 0 : peer.autopilot == MAV_AUTOPILOT_PX4 ? (mode >> 16) & 0xff : mode;
    const float p3 = !standardMode && peer.autopilot == MAV_AUTOPILOT_PX4 ? (mode >> 24) & 0xff : 0;
    if (!sendCommand(peer,command,p1,p2,p3)) return failVehicle(peer,"Mode command not sent.");
    peer.pendingCommand = command;
    peer.desiredMode = mode;
    peer.desiredStandardMode = standardMode;
    peer.commandDeadline = m_clock.elapsed() + 10000;
    peer.commandStatus = "Changing mode…";
    emit dataChanged(index(row),index(row),{CommandPendingRole,CommandStatusRole});
    emit notice(nameForSystem(systemId) + ": " + peer.commandStatus);
    return true;
}

void VehicleLink::finishCommand(Peer &peer, const QString &message, bool error) {
    const auto command = peer.pendingCommand;
    peer.pendingCommand = 0;
    peer.queuedTakeoff = std::numeric_limits<double>::quiet_NaN();
    peer.missionTakeoff = false;
    peer.commandStatus = message;
    const int row = rowForSystem(peer.systemId);
    emit dataChanged(index(row),index(row),{CommandPendingRole,CommandStatusRole});
    if (error) {
        if (command == MAV_CMD_DO_REPOSITION) clearNavigation(peer.systemId);
        if (command == MAV_CMD_COMPONENT_ARM_DISARM && peer.preArmError && m_clock.elapsed() - peer.errorTime < 10000)
            emit notice(nameForSystem(peer.systemId) + ": pre-arm check failed.");
        else failVehicle(peer,message);
    } else {
        if (peer.commandError || (peer.preArmError && peer.armed)) clearVehicleError(peer);
        emit notice(nameForSystem(peer.systemId) + ": " + message);
    }
}

void VehicleLink::confirmState(Peer &peer) {
    if (peer.pendingCommand == MAV_CMD_COMPONENT_ARM_DISARM && peer.armed == peer.desiredArmed) {
        const double queued = peer.queuedTakeoff;
        const bool mission = peer.missionTakeoff;
        finishCommand(peer,peer.armed ? "Armed." : "Disarmed.");
        if (std::isfinite(queued)) peer.missionTakeoff = mission;
        if (std::isfinite(queued) && !advanceTakeoff(peer,queued)) failVehicle(peer,"Takeoff cancelled.");
    }
    else if ((peer.pendingCommand == MAV_CMD_DO_SET_MODE && peer.customMode == peer.desiredMode)
        || (peer.pendingCommand == MAV_CMD_DO_SET_STANDARD_MODE && peer.standardMode == peer.desiredStandardMode))
    {
        const double queued = peer.queuedTakeoff;
        const bool mission = peer.missionTakeoff;
        finishCommand(peer,"Mode: " + peer.flightMode + ".");
        if (std::isfinite(queued)) peer.missionTakeoff = mission;
        if (std::isfinite(queued) && !advanceTakeoff(peer,queued))
            failVehicle(peer,"Takeoff cancelled.");
    }
    else if (peer.pendingCommand == MAV_CMD_NAV_TAKEOFF && peer.armed && peer.altitudeTime >= 0
        && m_clock.elapsed() - peer.altitudeTime <= 5000 && peer.altitude >= peer.takeoffTarget - 0.5)
        finishCommand(peer,"Takeoff altitude reached.");
}

bool VehicleLink::failVehicle(Peer &peer, const QString &message, bool preArm) {
    peer.errorMessage = message;
    peer.preArmError = preArm;
    peer.commandError = !preArm;
    peer.errorTime = m_clock.elapsed();
    const int row = rowForSystem(peer.systemId);
    emit dataChanged(index(row),index(row),{ErrorMessageRole});
    emit notice(nameForSystem(peer.systemId) + ": " + (preArm ? QString("Pre-arm check failed.") : message.left(90)));
    return false;
}

void VehicleLink::clearVehicleError(Peer &peer) {
    peer.errorMessage.clear();
    peer.preArmError = false;
    peer.commandError = false;
    const int row = rowForSystem(peer.systemId);
    emit dataChanged(index(row),index(row),{ErrorMessageRole});
}

void VehicleLink::receiveStatusText(Peer &peer, const mavlink_message_t &message) {
    mavlink_statustext_t status{};
    mavlink_msg_statustext_decode(&message,&status);
    const QByteArray bytes(status.text,sizeof(status.text));
    const int terminator = bytes.indexOf('\0');
    const auto part = terminator < 0 ? bytes : bytes.left(terminator);
    QByteArray complete;
    quint8 severity = status.severity;
    if (!status.id) complete = part;
    else {
        if (status.chunk_seq == 0) {
            peer.statusTextBuffer.clear();
            peer.statusTextId = status.id;
            peer.statusTextNext = 0;
            peer.statusTextSeverity = status.severity;
        }
        if (peer.statusTextId != status.id || status.chunk_seq != peer.statusTextNext
            || (status.chunk_seq && m_clock.elapsed() - peer.statusTextTime > 3000)) return;
        peer.statusTextBuffer += part;
        peer.statusTextTime = m_clock.elapsed();
        ++peer.statusTextNext;
        if (peer.statusTextBuffer.size() > 1000) { peer.statusTextId = 0; peer.statusTextBuffer.clear(); return; }
        if (terminator < 0) return;
        complete = peer.statusTextBuffer;
        severity = peer.statusTextSeverity;
        peer.statusTextBuffer.clear();
        peer.statusTextId = 0;
    }
    const auto text = QString::fromUtf8(complete).simplified();
    if (text.isEmpty()) return;
    const bool preArm = text.contains("prearm",Qt::CaseInsensitive) || text.contains("pre-arm",Qt::CaseInsensitive)
        || text.contains("preflight",Qt::CaseInsensitive) || text.startsWith("Arm:",Qt::CaseInsensitive)
        || text.startsWith("Arming denied",Qt::CaseInsensitive);
    if (severity <= MAV_SEVERITY_WARNING || preArm) {
        failVehicle(peer,text,preArm);
        peer.commandError = false; // A later successful mode change does not resolve an autopilot fault.
    }
}

QString VehicleLink::groupForSystem(int systemId) const {
    const auto group = m_settings->value(QString("vehicleGroups/members/%1").arg(systemId)).toString();
    return m_groups.contains(group) ? group : "Ungrouped";
}

QVariantMap VehicleLink::groupCounts() const {
    QVariantMap result;
    for (const auto &group : m_groups) result.insert(group, 0);
    for (const auto &peer : m_peers) {
        const auto group = groupForSystem(peer.systemId);
        result[group] = result.value(group).toInt() + 1;
    }
    return result;
}

bool VehicleLink::renameVehicle(int systemId, const QString &name) {
    const auto trimmed = name.trimmed();
    if (systemId < 1 || systemId > 255 || trimmed.isEmpty() || trimmed.size() > 64) return false;
    m_settings->setValue(QString("vehicleNames/%1").arg(systemId), trimmed);
    m_settings->sync();
    m_resolvedNames = resolvedNames();
    if (!m_peers.isEmpty()) emit dataChanged(index(0),index(m_peers.size()-1),{VehicleNameRole});
    return m_settings->status() == QSettings::NoError;
}

void VehicleLink::clearVehicleNames() {
    m_settings->remove("vehicleNames");
    m_resolvedNames = resolvedNames();
    m_settings->sync();
    if (!m_peers.isEmpty()) emit dataChanged(index(0), index(m_peers.size() - 1), {VehicleNameRole});
}

QString VehicleLink::createGroup(const QString &name) {
    const auto trimmed = name.trimmed();
    if (trimmed.isEmpty() || trimmed.size() > 48 || trimmed == "__selected__") return {};
    for (const auto &group : m_groups)
        if (group.compare(trimmed, Qt::CaseInsensitive) == 0) return group;
    auto next = m_groups;
    next.append(trimmed);
    m_settings->setValue("vehicleGroups/list", next.mid(1));
    m_settings->sync();
    if (m_settings->status() != QSettings::NoError) return {};
    m_groups = next;
    emit groupsChanged();
    return trimmed;
}

void VehicleLink::moveToGroup(int systemId, const QString &group) {
    if (systemId < 1 || systemId > 255 || !m_groups.contains(group)) return;
    m_settings->setValue(QString("vehicleGroups/members/%1").arg(systemId), group);
    m_settings->sync();
    const int row = rowForSystem(systemId);
    if (row >= 0) emit dataChanged(index(row), index(row), {VehicleGroupRole});
    resolveSelection();
    emit groupsChanged();
}

void VehicleLink::toggle() {
    if (listening()) {
        m_timer.stop();
        m_socket.close();
        beginResetModel();
        m_peers.clear();
        m_navigationTargets.clear();
        emit navigationChanged();
        m_selectedSystemId = 0;
        m_selectedGroup.clear();
        m_selection.clear();
        m_planActive=false;
        m_missionTransfers.clear();
        m_flightMissions.clear();
        emit planChanged();
        emit selectionChanged();
        endResetModel();
        m_status = "Disconnected";
        emit groupsChanged();
    } else if (!m_socket.bind(QHostAddress::AnyIPv4, m_port, QAbstractSocket::DontShareAddress)) {
        m_status = QString("Cannot listen on UDP %1: ").arg(m_port) + m_socket.errorString();
    } else {
        m_status = QString("Searching for UDP on %1").arg(m_socket.localPort());
        m_timer.start();
    }
    emit changed();
}

void VehicleLink::receive() {
    bool structureChanged = false;
    while (m_socket.hasPendingDatagrams()) {
        const auto datagram = m_socket.receiveDatagram();
        mavlink_message_t message{}, buffer{};
        mavlink_status_t result{}, parser{};
        for (const auto byte : datagram.data()) {
            if (mavlink_frame_char_buffer(&buffer, &parser, static_cast<uint8_t>(byte),
                                         &message, &result) != MAVLINK_FRAMING_OK || message.sysid == 0) continue;
            int row = rowForSystem(message.sysid);
            if (message.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                mavlink_heartbeat_t heartbeat{};
                mavlink_msg_heartbeat_decode(&message, &heartbeat);
                const bool copter = heartbeat.type == MAV_TYPE_QUADROTOR || heartbeat.type == MAV_TYPE_COAXIAL
                    || heartbeat.type == MAV_TYPE_HELICOPTER || heartbeat.type == MAV_TYPE_HEXAROTOR
                    || heartbeat.type == MAV_TYPE_OCTOROTOR || heartbeat.type == MAV_TYPE_TRICOPTER
                    || heartbeat.type == MAV_TYPE_DODECAROTOR || heartbeat.type == MAV_TYPE_DECAROTOR;
                const bool aircraft = copter || heartbeat.type == MAV_TYPE_FIXED_WING
                    || (heartbeat.type >= 19 && heartbeat.type <= 25);
                if (!aircraft || (heartbeat.autopilot != MAV_AUTOPILOT_ARDUPILOTMEGA
                    && heartbeat.autopilot != MAV_AUTOPILOT_PX4)) continue;
                const bool added = row < 0;
                if (added) {
                    row = 0;
                    while (row < m_peers.size() && m_peers[row].systemId < message.sysid) ++row;
                    beginInsertRows({}, row, row);
                    Peer peer;
                    peer.systemId = message.sysid;
                    peer.session = ++m_nextSession;
                    m_peers.insert(row, peer);
                }
                auto &peer = m_peers[row];
                peer.componentId = message.compid;
                peer.address = datagram.senderAddress();
                peer.port = static_cast<quint16>(datagram.senderPort());
                peer.lastSeen = m_clock.elapsed();
                peer.autopilot = heartbeat.autopilot;
                if (added) peer.parameterBytewise = heartbeat.autopilot == MAV_AUTOPILOT_PX4;
                peer.vehicleType = heartbeat.type;
                peer.armed = heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED;
                if (peer.armed && peer.preArmError) clearVehicleError(peer);
                if (peer.customMode != heartbeat.custom_mode) peer.standardMode = 0;
                peer.customMode = heartbeat.custom_mode;
                peer.flightMode = heartbeat.base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED ? modeName(peer) : "Unknown";
                if (added) {
                    endInsertRows();
                    requestTelemetry(peer);
                    sendCommand(peer, MAV_CMD_REQUEST_MESSAGE, MAVLINK_MSG_ID_AUTOPILOT_VERSION);
                    sendCommand(peer, MAV_CMD_REQUEST_MESSAGE, MAVLINK_MSG_ID_HOME_POSITION);
                    sendCommand(peer, MAV_CMD_REQUEST_MESSAGE, MAVLINK_MSG_ID_STATUSTEXT);
                    sendCommand(peer, MAV_CMD_RUN_PREARM_CHECKS);
                    m_flightMissions.remove(peer.systemId);
                    if (!m_planActive) { m_missionTransfers.insert(peer.systemId,{}); sendMissionTransfer(peer.systemId); }
                    requestModes(peer);
                    structureChanged = true;
                } else {
                    confirmState(peer);
                    emit dataChanged(index(row),index(row),{FirmwareRole,ArmedRole,FlightModeRole,IconIdRole});
                }
            } else if (row >= 0) {
                auto &peer = m_peers[row];
                // Only accept telemetry from the discovered flight controller endpoint/component.
                if (message.compid != peer.componentId || datagram.senderAddress() != peer.address
                    || datagram.senderPort() != peer.port) continue;
                if (receiveMission(peer,message)) continue;
                if (message.msgid == MAVLINK_MSG_ID_MISSION_COUNT) {
                    mavlink_mission_count_t count{};
                    mavlink_msg_mission_count_decode(&message,&count);
                    if (count.mission_type == MAV_MISSION_TYPE_MISSION && (!count.target_system || count.target_system == 255)
                        && (!count.target_component || count.target_component == MAV_COMP_ID_MISSIONPLANNER)) {
                        peer.missionCount = std::max(0,int(count.count)-(peer.autopilot==MAV_AUTOPILOT_ARDUPILOTMEGA ? 1 : 0));
                        mavlink_message_t reply{};
                        mavlink_status_t tx{};
                        tx.current_tx_seq = m_sequence++;
                        mavlink_msg_mission_ack_pack_status(255,MAV_COMP_ID_MISSIONPLANNER,&tx,&reply,
                            peer.systemId,peer.componentId,MAV_MISSION_ACCEPTED,MAV_MISSION_TYPE_MISSION,0);
                        send(reply,peer);
                    }
                } else if (message.msgid == MAVLINK_MSG_ID_MISSION_CURRENT) {
                    mavlink_mission_current_t current{};
                    mavlink_msg_mission_current_decode(&message,&current);
                    peer.missionState = current.mission_state;
                    if (current.mission_state == MISSION_STATE_NO_MISSION) peer.missionCount = 0;
                    else if (current.total && current.total != UINT16_MAX) peer.missionCount = std::max(0,int(current.total)-(peer.autopilot==MAV_AUTOPILOT_ARDUPILOTMEGA ? 1 : 0));
                } else if (message.msgid == MAVLINK_MSG_ID_STATUSTEXT) {
                    receiveStatusText(peer,message);
                } else if (message.msgid == MAVLINK_MSG_ID_SYS_STATUS) {
                    mavlink_sys_status_t state{};
                    mavlink_msg_sys_status_decode(&message,&state);
                    if (peer.preArmError && (state.onboard_control_sensors_present & MAV_SYS_STATUS_PREARM_CHECK)
                        && (state.onboard_control_sensors_enabled & MAV_SYS_STATUS_PREARM_CHECK)
                        && (state.onboard_control_sensors_health & MAV_SYS_STATUS_PREARM_CHECK)) clearVehicleError(peer);
                } else if (message.msgid == MAVLINK_MSG_ID_PARAM_VALUE) {
                    receiveParameter(peer, message);
                } else if (message.msgid == MAVLINK_MSG_ID_AUTOPILOT_VERSION) {
                    mavlink_autopilot_version_t version{};
                    mavlink_msg_autopilot_version_decode(&message, &version);
                    if (version.capabilities & MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE)
                        peer.parameterBytewise = true;
                    else if (version.capabilities & MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_C_CAST)
                        peer.parameterBytewise = false;
                    if (version.flight_sw_version != 0) {
                        peer.firmwareVersion = QString("%1.%2.%3")
                            .arg((version.flight_sw_version >> 24) & 0xff)
                            .arg((version.flight_sw_version >> 16) & 0xff)
                            .arg((version.flight_sw_version >> 8) & 0xff);
                        emit dataChanged(index(row), index(row), {FirmwareRole});
                    }
                } else if (message.msgid == MAVLINK_MSG_ID_HOME_POSITION) {
                    mavlink_home_position_t home{};
                    mavlink_msg_home_position_decode(&message,&home);
                    peer.homeAltitude = home.altitude / 1000.0;
                    peer.homeLatitude = home.latitude / 1e7;
                    peer.homeLongitude = home.longitude / 1e7;
                    if (m_flightMissions.contains(peer.systemId)) emit planChanged();
                } else if (message.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
                    mavlink_global_position_int_t position{};
                    mavlink_msg_global_position_int_decode(&message, &position);
                    const double latitude = position.lat / 1e7, longitude = position.lon / 1e7;
                    if (latitude >= -90 && latitude <= 90 && longitude >= -180 && longitude <= 180) {
                        peer.latitude = latitude;
                        peer.longitude = longitude;
                        peer.positionTime = m_clock.elapsed();
                    }
                    if (position.hdg != UINT16_MAX) peer.heading = position.hdg / 100.0;
                    peer.altitude = position.relative_alt / 1000.0;
                    peer.homeAltitude = (double(position.alt) - double(position.relative_alt)) / 1000.0;
                    peer.altitudeTime = m_clock.elapsed();
                    peer.velocity = std::hypot(double(position.vx), double(position.vy)) / 100.0;
                    peer.velocityTime = m_clock.elapsed();
                    confirmState(peer);
                    emit dataChanged(index(row),index(row),{AltitudeRole,VelocityRole,LatitudeRole,LongitudeRole,HeadingRole,PositionValidRole});
                } else if (message.msgid == MAVLINK_MSG_ID_VFR_HUD) {
                    mavlink_vfr_hud_t hud{};
                    mavlink_msg_vfr_hud_decode(&message, &hud);
                    if (std::isfinite(hud.groundspeed) && hud.groundspeed >= 0) {
                        peer.velocity = hud.groundspeed;
                        peer.velocityTime = m_clock.elapsed();
                        emit dataChanged(index(row), index(row), {VelocityRole});
                    }
                } else if (message.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
                    mavlink_command_ack_t ack{};
                    mavlink_msg_command_ack_decode(&message,&ack);
                    if (!peer.pendingCommand || ack.command != peer.pendingCommand
                        || (ack.target_system && ack.target_system != 255)
                        || (ack.target_component && ack.target_component != MAV_COMP_ID_MISSIONPLANNER)) continue;
                    if (ack.result == MAV_RESULT_ACCEPTED || ack.result == MAV_RESULT_IN_PROGRESS) {
                        if (ack.command == MAV_CMD_MISSION_START && ack.result == MAV_RESULT_ACCEPTED) {
                            finishCommand(peer, "Mission started.");
                            continue;
                        }
                        if (ack.command == MAV_CMD_DO_REPOSITION && ack.result == MAV_RESULT_ACCEPTED) {
                            finishCommand(peer,"Go Here accepted.");
                            continue;
                        }
                        peer.commandStatus = ack.result == MAV_RESULT_ACCEPTED
                            ? "Accepted…" : "In progress…";
                        if (ack.command == MAV_CMD_NAV_TAKEOFF) peer.commandDeadline = m_clock.elapsed() + 60000;
                        if (ack.result == MAV_RESULT_IN_PROGRESS) peer.commandDeadline = m_clock.elapsed() + 15000;
                        emit dataChanged(index(row),index(row),{CommandStatusRole});
                        if (peer.pendingCommand == MAV_CMD_DO_SET_STANDARD_MODE)
                            sendCommand(peer,MAV_CMD_REQUEST_MESSAGE,MAVLINK_MSG_ID_CURRENT_MODE);
                    } else {
                        const QString action = ack.command == MAV_CMD_COMPONENT_ARM_DISARM ? (peer.desiredArmed ? "Arm" : "Disarm")
                            : ack.command == MAV_CMD_NAV_TAKEOFF ? "Takeoff" : ack.command == MAV_CMD_DO_REPOSITION ? "Go Here"
                            : ack.command == MAV_CMD_MISSION_START ? "Mission" : "Mode";
                        finishCommand(peer,action + ": " + commandResult(ack.result) + ".",true);
                        if (ack.command == MAV_CMD_COMPONENT_ARM_DISARM && peer.desiredArmed)
                            sendCommand(peer,MAV_CMD_RUN_PREARM_CHECKS);
                    }
                } else if (message.msgid == MAVLINK_MSG_ID_AVAILABLE_MODES) {
                    mavlink_available_modes_t mode{};
                    mavlink_msg_available_modes_decode(&message,&mode);
                    if (!mode.number_modes || !mode.mode_index || mode.mode_index > mode.number_modes) continue;
                    if ((mode.seq && peer.modeSequence && mode.seq != peer.modeSequence)
                        || (peer.expectedModes && peer.expectedModes != mode.number_modes))
                        peer.reportedModes.clear();
                    peer.expectedModes = mode.number_modes;
                    if (mode.seq) peer.modeSequence = mode.seq;
                    const auto bytes = QByteArray(mode.mode_name,sizeof(mode.mode_name));
                    QString name = QString::fromUtf8(bytes.split('\0').first()).trimmed();
                    if (name.isEmpty()) {
                        Peer temporary = peer;
                        temporary.customMode = mode.custom_mode;
                        temporary.standardMode = mode.standard_mode;
                        temporary.reportedModes.clear();
                        name = mode.standard_mode ? standardModeName(mode.standard_mode) : modeName(temporary);
                    }
                    peer.reportedModes[mode.mode_index] = {mode.custom_mode,name,mode.standard_mode,
                        !(mode.properties & MAV_MODE_PROPERTY_NOT_USER_SELECTABLE)};
                    if (reportedModesComplete(peer)) {
                        peer.flightMode = modeName(peer);
                        emit dataChanged(index(row),index(row),{FlightModeRole});
                        emit modeListChanged(peer.systemId);
                    }
                } else if (message.msgid == MAVLINK_MSG_ID_AVAILABLE_MODES_MONITOR) {
                    mavlink_available_modes_monitor_t monitor{};
                    mavlink_msg_available_modes_monitor_decode(&message,&monitor);
                    if (monitor.seq && monitor.seq != peer.modeSequence) {
                        peer.modeSequence = monitor.seq;
                        peer.reportedModes.clear();
                        peer.expectedModes = 0;
                        peer.modeRequests = 0;
                        requestModes(peer);
                        emit modeListChanged(peer.systemId);
                    }
                } else if (message.msgid == MAVLINK_MSG_ID_CURRENT_MODE) {
                    mavlink_current_mode_t mode{};
                    mavlink_msg_current_mode_decode(&message,&mode);
                    peer.customMode = mode.custom_mode;
                    peer.standardMode = mode.standard_mode;
                    peer.flightMode = modeName(peer);
                    confirmState(peer);
                    emit dataChanged(index(row),index(row),{FlightModeRole});
                }
            }
        }
    }
    if (structureChanged) {
        resolveSelection();
        emit groupsChanged();
        emit changed();
    }
    updateStatus();
}

bool VehicleLink::send(const mavlink_message_t &message, const Peer &peer) {
    uint8_t bytes[MAVLINK_MAX_PACKET_LEN];
    const auto length = mavlink_msg_to_send_buffer(bytes, &message);
    const bool sent = m_socket.writeDatagram(reinterpret_cast<const char *>(bytes), length, peer.address, peer.port) == length;
    quint16 command = 0;
    if (message.msgid == MAVLINK_MSG_ID_COMMAND_LONG) command = mavlink_msg_command_long_get_command(&message);
    else if (message.msgid == MAVLINK_MSG_ID_COMMAND_INT) command = mavlink_msg_command_int_get_command(&message);
    if (sent && (command == MAV_CMD_COMPONENT_ARM_DISARM || command == MAV_CMD_DO_SET_MODE
        || command == MAV_CMD_DO_SET_STANDARD_MODE || command == MAV_CMD_NAV_TAKEOFF
        || command == MAV_CMD_MISSION_START || command == MAV_CMD_DO_REPOSITION)) clearNavigation(peer.systemId);
    return sent;
}

void VehicleLink::requestTelemetry(const Peer &peer) {
    for (const auto messageId : {MAVLINK_MSG_ID_GLOBAL_POSITION_INT, MAVLINK_MSG_ID_VFR_HUD}) {
        mavlink_message_t message{};
        mavlink_status_t status{};
        status.current_tx_seq = m_sequence++;
        mavlink_msg_command_long_pack_status(255, MAV_COMP_ID_MISSIONPLANNER, &status, &message,
            peer.systemId, peer.componentId, MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            messageId, 500000, 0, 0, 0, 0, 0);
        send(message, peer);
    }
}

void VehicleLink::tick() {
    bool removed = false;
    for (int row = m_peers.size() - 1; row >= 0; --row) {
        if (m_clock.elapsed() - m_peers[row].lastSeen > 5000) {
            if (m_selectedSystemId == m_peers[row].systemId) { m_selectedSystemId = 0; emit selectionChanged(); }
            if (m_peers[row].pendingCommand)
                emit notice(nameForSystem(m_peers[row].systemId) + ": disconnected before state confirmation.");
            beginRemoveRows({}, row, row);
            m_peers.removeAt(row);
            endRemoveRows();
            removed = true;
        }
    }
    mavlink_message_t message{};
    mavlink_status_t tx{};
    tx.current_tx_seq = m_sequence++;
    mavlink_msg_heartbeat_pack_status(255, MAV_COMP_ID_MISSIONPLANNER, &tx, &message,
        MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
    for (auto &peer : m_peers) {
        send(message,peer);
        if (peer.pendingCommand && m_clock.elapsed() >= peer.commandDeadline)
            finishCommand(peer,"Command timed out.",true);
        if (!reportedModesComplete(peer) && peer.modeRequests < 3
            && m_clock.elapsed() - peer.modesRequestedAt > 3000) requestModes(peer);
    }
    if (!m_peers.isEmpty()) emit dataChanged(index(0), index(m_peers.size() - 1), {AltitudeRole, VelocityRole, PositionValidRole});
    if (removed) {
        resolveSelection();
        emit groupsChanged();
        emit changed();
    }
    missionTick();
    updateNavigation();
    updateStatus();
}

void VehicleLink::updateStatus() {
    const auto next = m_peers.isEmpty() ? QString("Searching for UDP on %1").arg(m_socket.localPort())
        : QString("Connected");
    if (m_status != next) { m_status = next; emit changed(); }
}
