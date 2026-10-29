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
    required property string iconId
    required property bool commandPending
    signal contextRequested(int sysId)
    signal settingsRequested(int sysId)
    signal modesRequested(int sysId)
    implicitHeight: details.implicitHeight + 14
    Accessible.role: Accessible.ListItem
    Accessible.name: vehicleName + ", " + firmware + ", " + (armed ? "Armed" : "Disarmed") + ", " + flightMode
    radius: 4
    color: hover.hovered ? "#223542" : "#1a2a36"
    border.color: hover.hovered ? "#456273" : "#2b3f4d"
    HoverHandler { id: hover }
    MouseArea {
        anchors.fill: parent
        acceptedButtons: Qt.LeftButton | Qt.RightButton
        onClicked: function(mouse) {
            if (mouse.button === Qt.RightButton) card.contextRequested(card.systemId)
            else vehicleLink.centerVehicle(card.systemId)
        }
    }
   ColumnLayout {
        id: details
        z: 1
        anchors.fill: parent
        anchors.margins: 7
        spacing: 2
        RowLayout {
            Layout.fillWidth: true
            spacing: 2
            VehicleIcon { kind: card.iconId; Layout.preferredWidth: 13; Layout.preferredHeight: 13 }
            Label { text: card.vehicleName; textFormat: Text.PlainText; font.pixelSize: 9; font.bold: true; elide: Text.ElideRight; Layout.fillWidth: true }
            ToolButton {
                text: "\u2699"
                font.pixelSize: 12
                implicitWidth: 17
                implicitHeight: 15
                padding: 0
                Accessible.name: "Settings for " + card.vehicleName
                ToolTip.visible: hovered
                ToolTip.text: "Vehicle settings"
                onClicked: card.settingsRequested(card.systemId)
            }
        }
        Label { text: card.firmware + " · #" + card.systemId; color: "#879eae"; font.pixelSize: 6 }
        RowLayout {
            Layout.topMargin: 1
            spacing: 2
            Label { text: card.armed ? "Armed" : "Disarmed"; color: card.armed ? "#ffc078" : "#71cdbd"; font.pixelSize: 9; font.bold: true }
            Label {
                text: card.flightMode + " ▾"
                color: "#d9e4eb"
                font.pixelSize: 7
                elide: Text.ElideRight
                Layout.fillWidth: true
                Accessible.role: Accessible.Button
                Accessible.name: "Change flight mode for " + card.vehicleName
                MouseArea { anchors.fill: parent; cursorShape: Qt.PointingHandCursor; onClicked: card.modesRequested(card.systemId) }
            }
        }
        Rectangle { Layout.fillWidth: true; height: 0.5; color: "#304451"; Layout.topMargin: 2; Layout.bottomMargin: 1 }
        RowLayout {
            spacing: 6
            ColumnLayout {
                spacing: 2
                Label { text: "Altitude"; color: "#8ba2b2"; font.pixelSize: 6 }
                Label { text: Number.isFinite(card.altitude) ? card.altitude.toFixed(1) + " m" : "—"; font.pixelSize: 9 }
            }
            Item {
                implicitWidth: speedColumn.implicitWidth
                implicitHeight: speedColumn.implicitHeight
                Accessible.role: Accessible.Button
                Accessible.name: "Toggle velocity units, currently " + vehicleLink.velocityUnit
                ColumnLayout {
                    id: speedColumn
                    spacing: 2
                    Label { text: "Velocity"; color: "#8ba2b2"; font.pixelSize: 6 }
                    Label {
                        text: Number.isFinite(card.velocity) ? (card.velocity * (vehicleLink.velocityUnit === "kph" ? 3.6 : 2.2369362921)).toFixed(1) + " " + vehicleLink.velocityUnit : "—"
                        font.pixelSize: 9
                    }
                }
                MouseArea { anchors.fill: parent; cursorShape: Qt.PointingHandCursor; onClicked: vehicleLink.toggleVelocityUnit() }
            }
        }
    }
}
