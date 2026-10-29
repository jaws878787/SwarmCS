#include "VehicleLink.h"
#include <QNetworkDatagram>
#include <QStringList>

VehicleLink::VehicleLink(QObject *parent) : QObject(parent) {
    m_clock.start();
    m_timer.setInterval(1000);
    connect(&m_socket, &QUdpSocket::readyRead, this, &VehicleLink::receive);
    connect(&m_timer, &QTimer::timeout, this, &VehicleLink::tick);
}

void VehicleLink::toggle() {
    if (listening()) {
        m_timer.stop();
        m_socket.close();
        m_peers.clear();
        m_status = "Disconnected";
    } else if (!m_socket.bind(QHostAddress::AnyIPv4, 14550, QAbstractSocket::DontShareAddress)) {
        m_status = "Cannot listen on UDP 14550: " + m_socket.errorString();
    } else {
        m_status = "Searching on UDP 14550…";
        m_timer.start();
    }
    emit changed();
}

void VehicleLink::receive() {
    while (m_socket.hasPendingDatagrams()) {
        const auto datagram = m_socket.receiveDatagram();
        // UDP MAVLink frames are self-contained. Never combine data from different senders.
        mavlink_message_t message{}, buffer{};
        mavlink_status_t result{}, parser{};
        for (const auto byte : datagram.data()) {
            if (mavlink_frame_char_buffer(&buffer, &parser, static_cast<uint8_t>(byte),
                                          &message, &result) != MAVLINK_FRAMING_OK)
                continue;
            if (message.msgid != MAVLINK_MSG_ID_HEARTBEAT || message.sysid == 0) continue;
            mavlink_heartbeat_t heartbeat{};
            mavlink_msg_heartbeat_decode(&message, &heartbeat);
            const bool copter = heartbeat.type == MAV_TYPE_QUADROTOR || heartbeat.type == MAV_TYPE_COAXIAL
                || heartbeat.type == MAV_TYPE_HELICOPTER || heartbeat.type == MAV_TYPE_HEXAROTOR
                || heartbeat.type == MAV_TYPE_OCTOROTOR || heartbeat.type == MAV_TYPE_TRICOPTER
                || heartbeat.type == MAV_TYPE_DODECAROTOR || heartbeat.type == MAV_TYPE_DECAROTOR;
            if (!copter || (heartbeat.autopilot != MAV_AUTOPILOT_ARDUPILOTMEGA
                            && heartbeat.autopilot != MAV_AUTOPILOT_PX4)) continue;
            const auto key = datagram.senderAddress().toString() + ':'
                + QString::number(datagram.senderPort()) + ':' + QString::number(message.sysid);
            m_peers.insert(key, {datagram.senderAddress(), static_cast<quint16>(datagram.senderPort()), m_clock.elapsed(),
                QString("%1 · Vehicle %2").arg(heartbeat.autopilot == MAV_AUTOPILOT_PX4 ? "PX4" : "ArduPilot")
                    .arg(message.sysid)});
        }
    }
    updateStatus();
}

void VehicleLink::tick() {
    for (auto it = m_peers.begin(); it != m_peers.end();) {
        if (m_clock.elapsed() - it->lastSeen > 5000) it = m_peers.erase(it);
        else ++it;
    }
    mavlink_message_t message{};
    mavlink_status_t tx{};
    tx.current_tx_seq = m_sequence++;
    mavlink_msg_heartbeat_pack_status(255, MAV_COMP_ID_MISSIONPLANNER, &tx, &message,
        MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
    uint8_t bytes[MAVLINK_MAX_PACKET_LEN];
    const auto length = mavlink_msg_to_send_buffer(bytes, &message);
    for (const auto &peer : m_peers)
        m_socket.writeDatagram(reinterpret_cast<const char *>(bytes), length, peer.address, peer.port);
    updateStatus();
}

void VehicleLink::updateStatus() {
    QStringList names;
    for (const auto &peer : m_peers) names.append(peer.name);
    names.sort();
    const auto next = names.isEmpty() ? QString("Searching on UDP 14550…") : names.join("   |   ");
    if (m_status != next) { m_status = next; emit changed(); }
}
