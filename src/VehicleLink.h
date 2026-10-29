#pragma once
#include <QAbstractListModel>
#include <QUdpSocket>
#include <QTimer>
#include <QElapsedTimer>
#include <QSettings>
#include <memory>
#include <limits>
#include <common/mavlink.h>

class VehicleLink final : public QAbstractListModel {
    Q_OBJECT
    Q_PROPERTY(bool listening READ listening NOTIFY changed)
    Q_PROPERTY(QString status READ status NOTIFY changed)
    Q_PROPERTY(bool searching READ searching NOTIFY changed)
    Q_PROPERTY(int count READ count NOTIFY changed)
    Q_PROPERTY(QStringList groups READ groups NOTIFY groupsChanged)
    Q_PROPERTY(QVariantMap groupCounts READ groupCounts NOTIFY groupsChanged)
public:
    enum Role { SystemIdRole = Qt::UserRole + 1, VehicleNameRole, FirmwareRole,
                ArmedRole, FlightModeRole, AltitudeRole, VelocityRole, VehicleGroupRole };
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
signals:
    void changed();
    void groupsChanged();
private:
    struct Peer {
        int systemId = 0;
        quint8 componentId = 0;
        QHostAddress address;
        quint16 port = 0;
        qint64 lastSeen = 0;
        quint8 autopilot = 0;
        bool armed = false;
        QString flightMode;
        double altitude = std::numeric_limits<double>::quiet_NaN();
        double velocity = std::numeric_limits<double>::quiet_NaN();
        qint64 altitudeTime = -1;
        qint64 velocityTime = -1;
    };
    int rowForSystem(int systemId) const;
    QString nameForSystem(int systemId) const;
    QString groupForSystem(int systemId) const;
    void receive();
    void tick();
    void updateStatus();
    void requestTelemetry(const Peer &peer);
    void send(const mavlink_message_t &message, const Peer &peer);
    QUdpSocket m_socket;
    QTimer m_timer;
    QElapsedTimer m_clock;
    QList<Peer> m_peers;
    std::unique_ptr<QSettings> m_settings;
    QStringList m_groups;
    QString m_status = "Disconnected";
    quint16 m_port;
    quint8 m_sequence = 0;
};
