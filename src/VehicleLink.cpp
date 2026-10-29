#include "VehicleLink.h"
#include <QNetworkDatagram>
#include <cmath>
#include <algorithm>

namespace {
QString flightMode(quint8 autopilot, quint8 baseMode, quint32 customMode) {
    if (!(baseMode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)) return "Unknown";
    if (autopilot == MAV_AUTOPILOT_ARDUPILOTMEGA) {
        // ArduCopter mode numbers: ArduPilot/ArduCopter/mode.h.
        static const QHash<quint32, QString> modes = {
            {0, "Stabilize"}, {1, "Acro"}, {2, "Alt Hold"}, {3, "Auto"}, {4, "Guided"},
            {5, "Loiter"}, {6, "RTL"}, {7, "Circle"}, {9, "Land"}, {11, "Drift"},
            {13, "Sport"}, {14, "Flip"}, {15, "Autotune"}, {16, "PosHold"}, {17, "Brake"},
            {18, "Throw"}, {19, "Avoid ADS-B"}, {20, "Guided No GPS"}, {21, "Smart RTL"},
            {22, "FlowHold"}, {23, "Follow"}, {24, "ZigZag"}, {25, "System ID"},
            {26, "Autorotate"}, {27, "Auto RTL"}, {28, "Turtle"}
        };
        return modes.value(customMode, QString("Mode %1").arg(customMode));
    }
    // PX4 custom_mode packs main_mode in bits 16..23 and sub_mode in bits 24..31.
    const auto main = (customMode >> 16) & 0xff;
    const auto sub = (customMode >> 24) & 0xff;
    if (main == 4) {
        static const QHash<quint32, QString> modes = {
            {1, "Ready"}, {2, "Takeoff"}, {3, "Hold"}, {4, "Mission"}, {5, "RTL"},
            {6, "Land"}, {8, "Follow"}, {9, "Precision Land"}, {10, "VTOL Takeoff"},
            {19, "Guided Course"}, {20, "Descend"}
        };
        if (sub >= 11 && sub <= 18) return QString("External %1").arg(sub - 10);
        return modes.value(sub, QString("Auto (%1)").arg(sub));
    }
    if (main == 3 && sub == 1) return "Orbit";
    if (main == 3 && sub == 2) return "Position Slow";
    static const QHash<quint32, QString> modes = {
        {1, "Manual"}, {2, "Altitude"}, {3, "Position"}, {5, "Acro"}, {6, "Offboard"},
        {7, "Stabilized"}, {8, "Rattitude"}, {9, "Simple"}, {10, "Termination"},
        {11, "Altitude Cruise"}, {12, "Manual Parking"}
    };
    return modes.value(main, QString("Mode %1/%2").arg(main).arg(sub));
}
}

VehicleLink::VehicleLink(QObject *parent, const QString &settingsFile, quint16 port)
    : QAbstractListModel(parent),
      m_settings(settingsFile.isEmpty() ? std::make_unique<QSettings>()
          : std::make_unique<QSettings>(settingsFile, QSettings::IniFormat)), m_port(port) {
    m_groups = {"Ungrouped"};
    for (const auto &group : m_settings->value("vehicleGroups/list").toStringList())
        if (!group.trimmed().isEmpty() && !m_groups.contains(group, Qt::CaseInsensitive)) m_groups.append(group);
    m_clock.start();
    m_timer.setInterval(1000);
    connect(&m_socket, &QUdpSocket::readyRead, this, &VehicleLink::receive);
    connect(&m_timer, &QTimer::timeout, this, &VehicleLink::tick);
}

int VehicleLink::rowCount(const QModelIndex &parent) const { return parent.isValid() ? 0 : m_peers.size(); }

QHash<int, QByteArray> VehicleLink::roleNames() const {
    return {{SystemIdRole, "systemId"}, {VehicleNameRole, "vehicleName"}, {FirmwareRole, "firmware"},
        {ArmedRole, "armed"}, {FlightModeRole, "flightMode"}, {AltitudeRole, "altitude"},
        {VelocityRole, "velocity"}, {VehicleGroupRole, "vehicleGroup"}};
}

QVariant VehicleLink::data(const QModelIndex &index, int role) const {
    if (!index.isValid() || index.row() < 0 || index.row() >= m_peers.size()) return {};
    const auto &peer = m_peers[index.row()];
    switch (role) {
    case SystemIdRole: return peer.systemId;
    case VehicleNameRole: return nameForSystem(peer.systemId);
    case FirmwareRole: return peer.autopilot == MAV_AUTOPILOT_PX4 ? "PX4" : "ArduCopter";
    case ArmedRole: return peer.armed;
    case FlightModeRole: return peer.flightMode;
    case AltitudeRole: return peer.altitudeTime >= 0 && m_clock.elapsed() - peer.altitudeTime <= 5000
        ? peer.altitude : std::numeric_limits<double>::quiet_NaN();
    case VelocityRole: return peer.velocityTime >= 0 && m_clock.elapsed() - peer.velocityTime <= 5000
        ? peer.velocity : std::numeric_limits<double>::quiet_NaN();
    case VehicleGroupRole: return groupForSystem(peer.systemId);
    default: return {};
    }
}

int VehicleLink::rowForSystem(int systemId) const {
    for (int row = 0; row < m_peers.size(); ++row)
        if (m_peers[row].systemId == systemId) return row;
    return -1;
}

QString VehicleLink::nameForSystem(int systemId) const {
    const auto saved = m_settings->value(QString("vehicleNames/%1").arg(systemId)).toString().trimmed();
    return saved.isEmpty() ? QString("Vehicle %1").arg(systemId) : saved;
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
    const int row = rowForSystem(systemId);
    if (row >= 0) emit dataChanged(index(row), index(row), {VehicleNameRole});
    return m_settings->status() == QSettings::NoError;
}

void VehicleLink::clearVehicleNames() {
    m_settings->remove("vehicleNames");
    m_settings->sync();
    if (!m_peers.isEmpty()) emit dataChanged(index(0), index(m_peers.size() - 1), {VehicleNameRole});
}

QString VehicleLink::createGroup(const QString &name) {
    const auto trimmed = name.trimmed();
    if (trimmed.isEmpty() || trimmed.size() > 48) return {};
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
    emit groupsChanged();
}

void VehicleLink::toggle() {
    if (listening()) {
        m_timer.stop();
        m_socket.close();
        beginResetModel();
        m_peers.clear();
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
                if (!copter || (heartbeat.autopilot != MAV_AUTOPILOT_ARDUPILOTMEGA
                    && heartbeat.autopilot != MAV_AUTOPILOT_PX4)) continue;
                const bool added = row < 0;
                if (added) {
                    row = 0;
                    while (row < m_peers.size() && m_peers[row].systemId < message.sysid) ++row;
                    beginInsertRows({}, row, row);
                    Peer peer;
                    peer.systemId = message.sysid;
                    m_peers.insert(row, peer);
                }
                auto &peer = m_peers[row];
                peer.componentId = message.compid;
                peer.address = datagram.senderAddress();
                peer.port = static_cast<quint16>(datagram.senderPort());
                peer.lastSeen = m_clock.elapsed();
                peer.autopilot = heartbeat.autopilot;
                peer.armed = heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED;
                peer.flightMode = flightMode(heartbeat.autopilot, heartbeat.base_mode, heartbeat.custom_mode);
                if (added) {
                    endInsertRows();
                    requestTelemetry(peer);
                    structureChanged = true;
                } else emit dataChanged(index(row), index(row), {FirmwareRole, ArmedRole, FlightModeRole});
            } else if (row >= 0) {
                auto &peer = m_peers[row];
                // Only accept telemetry from the discovered flight controller endpoint/component.
                if (message.compid != peer.componentId || datagram.senderAddress() != peer.address
                    || datagram.senderPort() != peer.port) continue;
                if (message.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
                    mavlink_global_position_int_t position{};
                    mavlink_msg_global_position_int_decode(&message, &position);
                    peer.altitude = position.relative_alt / 1000.0;
                    peer.altitudeTime = m_clock.elapsed();
                    peer.velocity = std::hypot(double(position.vx), double(position.vy)) / 100.0;
                    peer.velocityTime = m_clock.elapsed();
                    emit dataChanged(index(row), index(row), {AltitudeRole, VelocityRole});
                } else if (message.msgid == MAVLINK_MSG_ID_VFR_HUD) {
                    mavlink_vfr_hud_t hud{};
                    mavlink_msg_vfr_hud_decode(&message, &hud);
                    if (std::isfinite(hud.groundspeed) && hud.groundspeed >= 0) {
                        peer.velocity = hud.groundspeed;
                        peer.velocityTime = m_clock.elapsed();
                        emit dataChanged(index(row), index(row), {VelocityRole});
                    }
                }
            }
        }
    }
    if (structureChanged) {
        emit groupsChanged();
        emit changed();
    }
    updateStatus();
}

void VehicleLink::send(const mavlink_message_t &message, const Peer &peer) {
    uint8_t bytes[MAVLINK_MAX_PACKET_LEN];
    const auto length = mavlink_msg_to_send_buffer(bytes, &message);
    m_socket.writeDatagram(reinterpret_cast<const char *>(bytes), length, peer.address, peer.port);
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
    for (const auto &peer : m_peers) send(message, peer);
    if (!m_peers.isEmpty()) emit dataChanged(index(0), index(m_peers.size() - 1), {AltitudeRole, VelocityRole});
    if (removed) {
        emit groupsChanged();
        emit changed();
    }
    updateStatus();
}

void VehicleLink::updateStatus() {
    const auto next = m_peers.isEmpty() ? QString("Searching for UDP on %1").arg(m_socket.localPort())
        : QString("Connected");
    if (m_status != next) { m_status = next; emit changed(); }
}
