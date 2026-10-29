#pragma once
#include <QObject>
#include <QTimer>

// Access the Windows location service when the Qt kit has no native position plugin.
class ComputerLocation final : public QObject {
    Q_OBJECT
public:
    explicit ComputerLocation(QObject *parent = nullptr);
    ~ComputerLocation() override;
    Q_INVOKABLE void request();
signals:
    void located(double latitude, double longitude);
private:
    void poll();
    void release();
    QTimer m_timer;
    void *m_location = nullptr;
    bool m_ownsCom = false;
    int m_attempts = 0;
};
