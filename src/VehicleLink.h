#pragma once
#include <QObject>
#include <QUdpSocket>
#include <QTimer>
#include <QHash>
#include <QElapsedTimer>
#include <common/mavlink.h>

class VehicleLink final : public QObject {
    Q_OBJECT
    Q_PROPERTY(bool listening READ listening NOTIFY changed)
    Q_PROPERTY(QString status READ status NOTIFY changed)
public:
    explicit VehicleLink(QObject *parent = nullptr);
    bool listening() const { return m_socket.state() == QAbstractSocket::BoundState; }
    QString status() const { return m_status; }
    Q_INVOKABLE void toggle();
signals:
    void changed();
private:
    struct Peer { QHostAddress address; quint16 port; qint64 lastSeen; QString name; };
    void receive();
    void tick();
    void updateStatus();
    QUdpSocket m_socket;
    QTimer m_timer;
    QElapsedTimer m_clock;
    QHash<QString, Peer> m_peers;
    QString m_status = "Disconnected";
    quint8 m_sequence = 0;
};
