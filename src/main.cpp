#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QQuickStyle>
#include "VehicleLink.h"

int main(int argc, char *argv[]) {
    QGuiApplication app(argc, argv);
    QQuickStyle::setStyle("Basic");
    app.setApplicationName("SwarmCS");
    app.setOrganizationName("SwarmCS");
    VehicleLink link;
    QQmlApplicationEngine engine;
    engine.rootContext()->setContextProperty("vehicleLink", &link);
    QObject::connect(&engine, &QQmlApplicationEngine::objectCreationFailed,
                     &app, [] { QCoreApplication::exit(1); }, Qt::QueuedConnection);
    engine.loadFromModule("SwarmCS", "Main");
    return app.exec();
}
