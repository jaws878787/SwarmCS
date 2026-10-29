#include <QtTest>
#include "VehicleLink.h"

class VehicleLinkTest : public QObject {
    Q_OBJECT
private slots:
    void discovery() {
        VehicleLink link;
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
                QHostAddress::LocalHost, 14550), qint64(length));
        };
        sendHeartbeat(MAV_AUTOPILOT_PX4, MAV_TYPE_FIXED_WING);
        QTest::qWait(100);
        QVERIFY(link.status().contains("Searching"));
        sendHeartbeat(MAV_AUTOPILOT_PX4, MAV_TYPE_QUADROTOR, true);
        QTest::qWait(100);
        QVERIFY(link.status().contains("Searching"));
        sendHeartbeat(MAV_AUTOPILOT_PX4, MAV_TYPE_QUADROTOR);
        QTRY_VERIFY(link.status().contains("PX4"));
        QTRY_VERIFY_WITH_TIMEOUT(vehicle.hasPendingDatagrams(), 2000);
        sendHeartbeat(MAV_AUTOPILOT_ARDUPILOTMEGA, MAV_TYPE_HEXAROTOR, false, true);
        QTRY_VERIFY(link.status().contains("ArduPilot"));
        QTRY_VERIFY_WITH_TIMEOUT(link.status().contains("Searching"), 7000);
        link.toggle();
        QVERIFY(!link.listening());
        QCOMPARE(link.status(), QString("Disconnected"));
        link.toggle();
        QVERIFY(link.listening());
    }
    void portConflict() {
        QUdpSocket blocker;
        QVERIFY(blocker.bind(QHostAddress::AnyIPv4, 14550, QAbstractSocket::DontShareAddress));
        VehicleLink link;
        link.toggle();
        QVERIFY(!link.listening());
        QVERIFY(link.status().contains("Cannot listen"));
    }
};
QTEST_GUILESS_MAIN(VehicleLinkTest)
#include "VehicleLinkTest.moc"
