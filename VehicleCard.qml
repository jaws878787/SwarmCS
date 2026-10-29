import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Rectangle {
    id: card
    required property int systemId
    required property string vehicleName
    required property string firmware
    required property bool armed
    required property string flightMode
    required property double altitude
    required property double velocity
    required property string vehicleGroup
    signal contextRequested(int sysId, string name)
    implicitHeight: details.implicitHeight + 28
    radius: 8
    color: hover.hovered ? "#223542" : "#1a2a36"
    border.color: hover.hovered ? "#456273" : "#2b3f4d"
    Accessible.role: Accessible.ListItem
    Accessible.name: vehicleName + ", " + firmware + ", " + (armed ? "Armed" : "Disarmed") + ", " + flightMode
    HoverHandler { id: hover }
    TapHandler {
        acceptedButtons: Qt.RightButton
        onTapped: card.contextRequested(card.systemId, card.vehicleName)
    }
    ColumnLayout {
        id: details
        anchors.fill: parent
        anchors.margins: 14
        spacing: 6
        RowLayout {
            Layout.fillWidth: true
            Label { text: card.vehicleName; font.pixelSize: 17; font.bold: true; elide: Text.ElideRight; Layout.fillWidth: true }
            Label { text: "#" + card.systemId; color: "#8098a9"; font.pixelSize: 11 }
        }
        Label { text: card.firmware; color: "#879eae"; font.pixelSize: 11 }
        RowLayout {
            Layout.topMargin: 5
            Layout.fillWidth: true
            Label { text: card.armed ? "Armed" : "Disarmed"; color: card.armed ? "#ffc078" : "#71cdbd"; font.pixelSize: 17; font.bold: true }
            Item { Layout.fillWidth: true }
            Label { text: card.flightMode; color: "#d9e4eb"; font.pixelSize: 14; elide: Text.ElideRight; Layout.maximumWidth: 140 }
        }
        Rectangle { Layout.fillWidth: true; height: 1; color: "#304451"; Layout.topMargin: 6; Layout.bottomMargin: 4 }
        GridLayout {
            Layout.fillWidth: true
            columns: 2
            columnSpacing: 16
            rowSpacing: 3
            Label { text: "Altitude"; color: "#8ba2b2"; font.pixelSize: 11; Layout.fillWidth: true; Layout.preferredWidth: 1 }
            Label { text: "Velocity"; color: "#8ba2b2"; font.pixelSize: 11; Layout.fillWidth: true; Layout.preferredWidth: 1 }
            Label { text: Number.isFinite(card.altitude) ? card.altitude.toFixed(1) + " m" : "—"; font.pixelSize: 17; Layout.fillWidth: true; Layout.preferredWidth: 1 }
            Label { text: Number.isFinite(card.velocity) ? card.velocity.toFixed(1) + " m/s" : "—"; font.pixelSize: 17; Layout.fillWidth: true; Layout.preferredWidth: 1 }
        }
    }
}
