#pragma once
#include <QMap>
#include <QString>
#include <QList>
#include <common/mavlink.h>

struct FlightModeOption {
    quint32 id;
    QString name;
    quint8 standardMode = 0;
    bool enabled = true;
};

// Numeric definitions: ArduPilot ArduCopter/mode.h, ArduPlane/mode.h,
// and PX4 src/modules/commander/px4_custom_mode.h. Reported AVAILABLE_MODES take precedence.
inline QList<FlightModeOption> firmwareModes(quint8 autopilot, bool plane) {
    QList<FlightModeOption> result;
    if (autopilot == MAV_AUTOPILOT_ARDUPILOTMEGA) {
        const QMap<quint32, QString> copter = {
            {0,"Stabilize"},{1,"Acro"},{2,"Alt Hold"},{3,"Auto"},{4,"Guided"},{5,"Loiter"},
            {6,"RTL"},{7,"Circle"},{9,"Land"},{11,"Drift"},{13,"Sport"},{14,"Flip"},
            {15,"Autotune"},{16,"PosHold"},{17,"Brake"},{18,"Throw"},{19,"Avoid ADS-B"},
            {20,"Guided No GPS"},{21,"Smart RTL"},{22,"FlowHold"},{23,"Follow"},{24,"ZigZag"},
            {25,"System ID"},{26,"Autorotate"},{27,"Auto RTL"},{28,"Turtle"}
        };
        const QMap<quint32, QString> fixedWing = {
            {0,"Manual"},{1,"Circle"},{2,"Stabilize"},{3,"Training"},{4,"Acro"},{5,"FBWA"},
            {6,"FBWB"},{7,"Cruise"},{8,"Autotune"},{10,"Auto"},{11,"RTL"},{12,"Loiter"},
            {13,"Takeoff"},{14,"Avoid ADS-B"},{15,"Guided"},{17,"QStabilize"},{18,"QHover"},
            {19,"QLoiter"},{20,"QLand"},{21,"QRTL"},{22,"QAutotune"},{23,"QAcro"},
            {24,"Thermal"},{25,"Loiter Alt QLand"},{26,"Autoland"}
        };
        const auto &table = plane ? fixedWing : copter;
        for (auto it = table.cbegin(); it != table.cend(); ++it) result.append({it.key(),it.value()});
    } else {
        const QMap<quint32, QString> main = {{1,"Manual"},{2,"Altitude"},{3,"Position"},
            {5,"Acro"},{6,"Offboard"},{7,"Stabilized"},{8,"Rattitude"},
            {10,"Termination"},{11,"Altitude Cruise"},{12,"Manual Parking"}};
        for (auto it = main.cbegin(); it != main.cend(); ++it)
            result.append({it.key() << 16,it.value(),0,it.key() != 10});
        result.append({(3u << 16) | (1u << 24),"Orbit"});
        result.append({(3u << 16) | (2u << 24),"Position Slow"});
        const QMap<quint32, QString> automatic = {{2,"Takeoff"},{3,"Hold"},{4,"Mission"},
            {5,"RTL"},{6,"Land"},{8,"Follow"},{9,"Precision Land"},{10,"VTOL Takeoff"},
            {19,"Guided Course"},{20,"Descend"}};
        for (auto it = automatic.cbegin(); it != automatic.cend(); ++it)
            result.append({(4u << 16) | (it.key() << 24),it.value()});
        for (quint32 i = 11; i <= 18; ++i)
            result.append({(4u << 16) | (i << 24),QString("External %1").arg(i - 10)});
    }
    return result;
}

inline QString standardModeName(quint8 mode) {
    static const QMap<int,QString> names = {{MAV_STANDARD_MODE_POSITION_HOLD,"Position Hold"},
        {MAV_STANDARD_MODE_ORBIT,"Orbit"},{MAV_STANDARD_MODE_CRUISE,"Cruise"},
        {MAV_STANDARD_MODE_ALTITUDE_HOLD,"Altitude Hold"},{MAV_STANDARD_MODE_SAFE_RECOVERY,"Safe Recovery"},
        {MAV_STANDARD_MODE_MISSION,"Mission"},{MAV_STANDARD_MODE_LAND,"Land"},{MAV_STANDARD_MODE_TAKEOFF,"Takeoff"}};
    return names.value(mode,QString("Standard mode %1").arg(mode));
}
