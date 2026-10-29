#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QQuickStyle>
#include <QStandardPaths>
#include "VehicleLink.h"
#include "ComputerLocation.h"

int main(int argc, char *argv[]) {
    QGuiApplication app(argc, argv);
    QQuickStyle::setStyle("Basic");
    app.setApplicationName("SwarmCS");
    app.setOrganizationName("SwarmCS");
    VehicleLink link;
    ComputerLocation computerLocation;
    QQmlApplicationEngine engine;
    engine.rootContext()->setContextProperty("vehicleLink", &link);
    engine.rootContext()->setContextProperty("computerLocation", &computerLocation);
    engine.rootContext()->setContextProperty("mapCacheRoot", QStandardPaths::writableLocation(QStandardPaths::CacheLocation));
    QObject::connect(&engine, &QQmlApplicationEngine::objectCreationFailed,
                     &app, [] { QCoreApplication::exit(1); }, Qt::QueuedConnection);
    engine.loadFromModule("SwarmCS", "Main");
    return app.exec();
}
