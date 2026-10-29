#include <QtTest>
#include "VehicleLink.h"
#include <QAbstractItemModelTester>
#include <QTemporaryDir>
#include <cmath>

namespace {
void sendPacket(QUdpSocket &socket, VehicleLink &link, const mavlink_message_t &message) {
    uint8_t bytes[MAVLINK_MAX_PACKET_LEN];
    const auto length = mavlink_msg_to_send_buffer(bytes, &message);
    QCOMPARE(socket.writeDatagram(reinterpret_cast<const char *>(bytes), length,
                                 QHostAddress::LocalHost, link.localPort()), qint64(length));
}
void heartbeat(QUdpSocket &socket, VehicleLink &link, int systemId, quint8 autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA,
               quint32 mode = 0, bool armed = false) {
    mavlink_message_t message{};
    mavlink_msg_heartbeat_pack(systemId, MAV_COMP_ID_AUTOPILOT1, &message, MAV_TYPE_QUADROTOR, autopilot,
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | (armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0), mode, MAV_STATE_STANDBY);
    sendPacket(socket, link, message);
}
}

class VehicleLinkTest : public QObject {
    Q_OBJECT
    QTemporaryDir m_storage;
    QString settingsFile() const { return m_storage.filePath(QString(QTest::currentTestFunction()) + ".ini"); }
private slots:
    void discovery() {
        VehicleLink link(nullptr, settingsFile(), 0);
        QAbstractItemModelTester tester(&link, QAbstractItemModelTester::FailureReportingMode::QtTest);
        QUdpSocket vehicle;
        QVERIFY(vehicle.bind(QHostAddress::LocalHost, 0));
        link.toggle();
        QVERIFY2(link.listening(), qPrintable(link.status()));
        QVERIFY(link.status().contains("Searching"));
        auto sendHeartbeat = [&](quint8 autopilot, quint8 type, bool corrupt = false, bool v1 = false) {
            mavlink_message_t message{};
            mavlink_status_t status{};
            if (v1) status.flags = MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
            mavlink_msg_heartbeat_pack_status(1, MAV_COMP_ID_AUTOPILOT1, &status, &message,
                type, autopilot, 0, 0, MAV_STATE_STANDBY);
            uint8_t bytes[MAVLINK_MAX_PACKET_LEN];
            const auto length = mavlink_msg_to_send_buffer(bytes, &message);
            if (corrupt) bytes[length - 1] ^= 0xff;
            QCOMPARE(vehicle.writeDatagram(reinterpret_cast<const char *>(bytes), length,
                QHostAddress::LocalHost, link.localPort()), qint64(length));
        };
        sendHeartbeat(MAV_AUTOPILOT_PX4, MAV_TYPE_FIXED_WING);
        QTest::qWait(100);
        QVERIFY(link.status().contains("Searching"));
        sendHeartbeat(MAV_AUTOPILOT_PX4, MAV_TYPE_QUADROTOR, true);
        QTest::qWait(100);
        QVERIFY(link.status().contains("Searching"));
        sendHeartbeat(MAV_AUTOPILOT_PX4, MAV_TYPE_QUADROTOR);
        QTRY_COMPARE(link.count(), 1);
        QCOMPARE(link.data(link.index(0), VehicleLink::FirmwareRole).toString(), QString("PX4"));
        QVERIFY(std::isnan(link.data(link.index(0), VehicleLink::AltitudeRole).toDouble()));
        QTRY_VERIFY_WITH_TIMEOUT(vehicle.hasPendingDatagrams(), 2000);
        sendHeartbeat(MAV_AUTOPILOT_ARDUPILOTMEGA, MAV_TYPE_HEXAROTOR, false, true);
        QTRY_COMPARE(link.data(link.index(0), VehicleLink::FirmwareRole).toString(), QString("ArduCopter"));
        QTRY_VERIFY_WITH_TIMEOUT(link.status().contains("Searching"), 7000);
        link.toggle();
        QVERIFY(!link.listening());
        QCOMPARE(link.status(), QString("Disconnected"));
        link.toggle();
        QVERIFY(link.listening());
    }
    void portConflict() {
        QUdpSocket blocker;
        QVERIFY(blocker.bind(QHostAddress::AnyIPv4, 0, QAbstractSocket::DontShareAddress));
        VehicleLink link(nullptr, settingsFile(), blocker.localPort());
        link.toggle();
        QVERIFY(!link.listening());
        QVERIFY(link.status().contains("Cannot listen"));
    }
    void telemetryAndIdentity() {
        VehicleLink link(nullptr, settingsFile(), 0);
        QAbstractItemModelTester tester(&link, QAbstractItemModelTester::FailureReportingMode::QtTest);
        QUdpSocket vehicle;
        QVERIFY(vehicle.bind(QHostAddress::LocalHost, 0));
        link.toggle();
        heartbeat(vehicle, link, 3);
        heartbeat(vehicle, link, 1, MAV_AUTOPILOT_ARDUPILOTMEGA, 4, true);
        heartbeat(vehicle, link, 2, MAV_AUTOPILOT_PX4, (4u << 16) | (3u << 24));
        QTRY_COMPARE(link.count(), 3);
        QCOMPARE(link.data(link.index(0), VehicleLink::SystemIdRole).toInt(), 1);
        QCOMPARE(link.data(link.index(0), VehicleLink::ArmedRole).toBool(), true);
        QCOMPARE(link.data(link.index(0), VehicleLink::FlightModeRole).toString(), QString("Guided"));
        QCOMPARE(link.data(link.index(1), VehicleLink::FlightModeRole).toString(), QString("Hold"));
        QCOMPARE(link.groupCounts().value("Ungrouped").toInt(), 3);
        QVERIFY(!link.status().contains("Vehicle 1"));

        mavlink_message_t position{};
        mavlink_msg_global_position_int_pack(1, MAV_COMP_ID_AUTOPILOT1, &position,
            1000, 397684000, -861581000, 250000, 12500, 300, 400, 0, 0);
        sendPacket(vehicle, link, position);
        QTRY_COMPARE(link.data(link.index(0), VehicleLink::AltitudeRole).toDouble(), 12.5);
        QCOMPARE(link.data(link.index(0), VehicleLink::VelocityRole).toDouble(), 5.0);

        // Another component or endpoint must not overwrite flight-controller telemetry.
        mavlink_msg_global_position_int_pack(1, MAV_COMP_ID_CAMERA, &position,
            1000, 0, 0, 0, 999000, 0, 0, 0, 0);
        sendPacket(vehicle, link, position);
        QUdpSocket otherEndpoint;
        QVERIFY(otherEndpoint.bind(QHostAddress::LocalHost, 0));
        mavlink_msg_global_position_int_pack(1, MAV_COMP_ID_AUTOPILOT1, &position,
            1000, 0, 0, 0, 999000, 0, 0, 0, 0);
        sendPacket(otherEndpoint, link, position);
        QTest::qWait(100);
        QCOMPARE(link.data(link.index(0), VehicleLink::AltitudeRole).toDouble(), 12.5);

        mavlink_message_t hud{};
        mavlink_msg_vfr_hud_pack(1, MAV_COMP_ID_AUTOPILOT1, &hud, 0, 7.25f, 0, 0, 500, 0);
        sendPacket(vehicle, link, hud);
        QTRY_COMPARE(link.data(link.index(0), VehicleLink::VelocityRole).toDouble(), 7.25);
        mavlink_msg_vfr_hud_pack(1, MAV_COMP_ID_AUTOPILOT1, &hud, 0,
            std::numeric_limits<float>::quiet_NaN(), 0, 0, 500, 0);
        sendPacket(vehicle, link, hud);
        QTest::qWait(100);
        QCOMPARE(link.data(link.index(0), VehicleLink::VelocityRole).toDouble(), 7.25);
        // A new source port for the same system ID keeps a single vehicle row.
        heartbeat(otherEndpoint, link, 1, MAV_AUTOPILOT_ARDUPILOTMEGA, 5, false);
        QTRY_COMPARE(link.data(link.index(0), VehicleLink::FlightModeRole).toString(), QString("Loiter"));
        QCOMPARE(link.count(), 3);
        QCOMPARE(link.data(link.index(0), VehicleLink::ArmedRole).toBool(), false);
    }
    void persistenceAndGroups() {
        const auto file = settingsFile();
        {
            VehicleLink link(nullptr, file, 0);
            QUdpSocket vehicle;
            QVERIFY(vehicle.bind(QHostAddress::LocalHost, 0));
            link.toggle();
            heartbeat(vehicle, link, 1);
            heartbeat(vehicle, link, 2);
            QTRY_COMPARE(link.count(), 2);
            QVERIFY(!link.renameVehicle(1, "  "));
            QVERIFY(link.renameVehicle(1, "  Scout  "));
            QCOMPARE(link.createGroup("  Alpha  "), QString("Alpha"));
            QCOMPARE(link.createGroup("alpha"), QString("Alpha"));
            QCOMPARE(link.groups().size(), 2);
            link.moveToGroup(1, "Alpha");
            QCOMPARE(link.groupCounts().value("Alpha").toInt(), 1);
            QCOMPARE(link.groupCounts().value("Ungrouped").toInt(), 1);
            link.moveToGroup(2, "Missing");
            QCOMPARE(link.data(link.index(1), VehicleLink::VehicleGroupRole).toString(), QString("Ungrouped"));
        }
        {
            VehicleLink reopened(nullptr, file, 0);
            QUdpSocket vehicle;
            QVERIFY(vehicle.bind(QHostAddress::LocalHost, 0));
            reopened.toggle();
            heartbeat(vehicle, reopened, 1);
            heartbeat(vehicle, reopened, 2);
            QTRY_COMPARE(reopened.count(), 2);
            QCOMPARE(reopened.data(reopened.index(0), VehicleLink::VehicleNameRole).toString(), QString("Scout"));
            QCOMPARE(reopened.data(reopened.index(1), VehicleLink::VehicleNameRole).toString(), QString("Vehicle 2"));
            QCOMPARE(reopened.data(reopened.index(0), VehicleLink::VehicleGroupRole).toString(), QString("Alpha"));
            reopened.clearVehicleNames();
            QCOMPARE(reopened.data(reopened.index(0), VehicleLink::VehicleNameRole).toString(), QString("Vehicle 1"));
            QCOMPARE(reopened.data(reopened.index(0), VehicleLink::VehicleGroupRole).toString(), QString("Alpha"));
        }
        VehicleLink cleared(nullptr, file, 0);
        QUdpSocket vehicle;
        QVERIFY(vehicle.bind(QHostAddress::LocalHost, 0));
        cleared.toggle();
        heartbeat(vehicle, cleared, 1);
        QTRY_COMPARE(cleared.count(), 1);
        QCOMPARE(cleared.data(cleared.index(0), VehicleLink::VehicleNameRole).toString(), QString("Vehicle 1"));
    }
    void staleTelemetry() {
        VehicleLink link(nullptr, settingsFile(), 0);
        QUdpSocket vehicle;
        QVERIFY(vehicle.bind(QHostAddress::LocalHost, 0));
        link.toggle();
        heartbeat(vehicle, link, 1);
        QTRY_COMPARE(link.count(), 1);
        mavlink_message_t position{};
        mavlink_msg_global_position_int_pack(1, MAV_COMP_ID_AUTOPILOT1, &position,
            0, 0, 0, 0, 2000, 100, 0, 0, 0);
        sendPacket(vehicle, link, position);
        QTRY_COMPARE(link.data(link.index(0), VehicleLink::AltitudeRole).toDouble(), 2.0);
        QTimer heartbeats;
        connect(&heartbeats, &QTimer::timeout, &link, [&] { heartbeat(vehicle, link, 1); });
        heartbeats.start(500);
        QTRY_VERIFY_WITH_TIMEOUT(std::isnan(link.data(link.index(0), VehicleLink::AltitudeRole).toDouble()), 7000);
        QVERIFY(std::isnan(link.data(link.index(0), VehicleLink::VelocityRole).toDouble()));
        QCOMPARE(link.count(), 1);
    }
};
QTEST_GUILESS_MAIN(VehicleLinkTest)
#include "VehicleLinkTest.moc"
