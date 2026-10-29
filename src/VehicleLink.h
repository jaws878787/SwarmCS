#pragma once
#include <QAbstractListModel>
#include <QUdpSocket>
#include <QTimer>
#include <QElapsedTimer>
#include <QSettings>
#include <memory>
#include <limits>
#include <QMap>
#include "FlightModes.h"
#include <common/mavlink.h>

class VehicleLink final : public QAbstractListModel {
    Q_OBJECT
    Q_PROPERTY(bool listening READ listening NOTIFY changed)
    Q_PROPERTY(QString status READ status NOTIFY changed)
    Q_PROPERTY(bool searching READ searching NOTIFY changed)
    Q_PROPERTY(int count READ count NOTIFY changed)
    Q_PROPERTY(QStringList groups READ groups NOTIFY groupsChanged)
    Q_PROPERTY(QVariantMap groupCounts READ groupCounts NOTIFY groupsChanged)
    Q_PROPERTY(QString velocityUnit READ velocityUnit NOTIFY velocityUnitChanged)
public:
    enum Role { SystemIdRole = Qt::UserRole + 1, VehicleNameRole, FirmwareRole,
                ArmedRole, FlightModeRole, AltitudeRole, VelocityRole, VehicleGroupRole,
                LatitudeRole, LongitudeRole, HeadingRole, PositionValidRole, IconIdRole,
                CommandPendingRole, CommandStatusRole };
    Q_ENUM(Role)
    explicit VehicleLink(QObject *parent = nullptr, const QString &settingsFile = {}, quint16 port = 14550);
    int rowCount(const QModelIndex &parent = {}) const override;
    QVariant data(const QModelIndex &index, int role) const override;
    QHash<int, QByteArray> roleNames() const override;
    bool listening() const { return m_socket.state() == QAbstractSocket::BoundState; }
    QString status() const { return m_status; }
    bool searching() const { return listening() && m_peers.isEmpty(); }
    int count() const { return m_peers.size(); }
    quint16 localPort() const { return m_socket.localPort(); }
    QStringList groups() const { return m_groups; }
    QVariantMap groupCounts() const;
    Q_INVOKABLE void toggle();
    Q_INVOKABLE bool renameVehicle(int systemId, const QString &name);
    Q_INVOKABLE void clearVehicleNames();
    Q_INVOKABLE QString createGroup(const QString &name);
    Q_INVOKABLE void moveToGroup(int systemId, const QString &group);
    Q_INVOKABLE QVariantMap vehicleDetails(int systemId) const;
    Q_INVOKABLE QVariantList iconOptions(int systemId) const;
    Q_INVOKABLE bool setVehicleIcon(int systemId, const QString &icon);
    Q_INVOKABLE QVariantList availableModes(int systemId) const;
    Q_INVOKABLE void refreshModes(int systemId);
    Q_INVOKABLE bool setArmed(int systemId, bool armed);
    Q_INVOKABLE bool setFlightMode(int systemId, quint32 mode, int standardMode = 0);
    Q_INVOKABLE void centerVehicle(int systemId);
    Q_INVOKABLE void toggleVelocityUnit();
    QString velocityUnit() const { return m_velocityUnit; }
signals:
    void changed();
    void groupsChanged();
    void velocityUnitChanged();
    void modeListChanged(int systemId);
    void centerRequested(int systemId, double latitude, double longitude);
    void notice(const QString &message);
private:
    struct Peer {
        int systemId = 0;
        quint8 componentId = 0;
        QHostAddress address;
        quint16 port = 0;
        qint64 lastSeen = 0;
        quint8 autopilot = 0;
        quint8 vehicleType = MAV_TYPE_QUADROTOR;
        bool armed = false;
        QString flightMode;
        quint32 customMode = 0;
        quint8 standardMode = 0;
        double latitude = std::numeric_limits<double>::quiet_NaN();
        double longitude = std::numeric_limits<double>::quiet_NaN();
        double heading = 0;
        qint64 positionTime = -1;
        QMap<int, FlightModeOption> reportedModes;
        int expectedModes = 0;
        quint8 modeSequence = 0;
        qint64 modesRequestedAt = -1;
        int modeRequests = 0;
        quint16 pendingCommand = 0;
        bool desiredArmed = false;
        quint32 desiredMode = 0;
        int desiredStandardMode = 0;
        qint64 commandDeadline = 0;
        QString commandStatus;
        double altitude = std::numeric_limits<double>::quiet_NaN();
        double velocity = std::numeric_limits<double>::quiet_NaN();
        qint64 altitudeTime = -1;
        qint64 velocityTime = -1;
    };
    int rowForSystem(int systemId) const;
    QString nameForSystem(int systemId) const;
    QMap<int, QString> resolvedNames() const;
    QString baseNameForSystem(int systemId) const;
    QString groupForSystem(int systemId) const;
    bool isPlane(const Peer &peer) const;
    bool positionValid(const Peer &peer) const;
    QString iconForPeer(const Peer &peer) const;
    QString modeName(const Peer &peer) const;
    bool reportedModesComplete(const Peer &peer) const;
    void requestModes(Peer &peer);
    bool sendCommand(Peer &peer, quint16 command, float p1 = 0, float p2 = 0, float p3 = 0);
    void finishCommand(Peer &peer, const QString &message);
    void confirmState(Peer &peer);
    void receive();
    void tick();
    void updateStatus();
    void requestTelemetry(const Peer &peer);
    bool send(const mavlink_message_t &message, const Peer &peer);
    QUdpSocket m_socket;
    QTimer m_timer;
    QElapsedTimer m_clock;
    QList<Peer> m_peers;
    std::unique_ptr<QSettings> m_settings;
    QStringList m_groups;
    QString m_status = "Disconnected";
    quint16 m_port;
    quint8 m_sequence = 0;
    QString m_velocityUnit = "kph";
    QMap<int, QString> m_resolvedNames;
};
