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
    required property string errorMessage
    signal contextRequested(int sysId)
    signal settingsRequested(int sysId)
    signal modesRequested(int sysId)
    implicitHeight: details.implicitHeight + 22
    Accessible.role: Accessible.ListItem
    Accessible.name: vehicleName + ", " + firmware + ", " + (armed ? "Armed" : "Disarmed") + ", " + flightMode
    radius: 6
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
        anchors.margins: 11
        spacing: 4
        RowLayout {
            Layout.fillWidth: true
            spacing: 4
            Item {
                Layout.preferredWidth: 26
                Layout.preferredHeight: 23
                VehicleIcon { anchors.centerIn: parent; kind: card.iconId; width: 20; height: 20 }
            }
            Label { text: card.vehicleName; textFormat: Text.PlainText; font.pixelSize: 13; font.bold: true; elide: Text.ElideRight; horizontalAlignment: Text.AlignHCenter; Layout.fillWidth: true }
            ToolButton {
                text: "\u2699"
                font.pixelSize: 18
                implicitWidth: 26
                implicitHeight: 23
                padding: 0
                Accessible.name: "Settings for " + card.vehicleName
                ToolTip.visible: hovered
                ToolTip.text: "Vehicle settings"
                onClicked: card.settingsRequested(card.systemId)
            }
        }
        Label { text: card.firmware; color: "#879eae"; font.pixelSize: 8; horizontalAlignment: Text.AlignHCenter; Layout.fillWidth: true; Layout.topMargin: -5 }
        RowLayout {
            Layout.fillWidth: true
            Layout.topMargin: 2
            spacing: 4
            Label { text: card.armed ? "Armed" : "Disarmed"; color: card.armed ? "#ffc078" : "#71cdbd"; font.pixelSize: 13; font.bold: true; horizontalAlignment: Text.AlignHCenter; Layout.fillWidth: true; Layout.preferredWidth: 1 }
            Label {
                text: card.flightMode + " ▾"
                color: "#d9e4eb"
                font.pixelSize: 11
                elide: Text.ElideRight
                horizontalAlignment: Text.AlignHCenter
                Layout.fillWidth: true
                Layout.preferredWidth: 1
                Accessible.role: Accessible.Button
                Accessible.name: "Change flight mode for " + card.vehicleName
                MouseArea { anchors.fill: parent; cursorShape: Qt.PointingHandCursor; onClicked: card.modesRequested(card.systemId) }
            }
        }
        Rectangle { Layout.fillWidth: true; height: 1; color: "#304451"; Layout.topMargin: 3; Layout.bottomMargin: 2 }
        RowLayout {
            Layout.fillWidth: true
            spacing: 8
            ColumnLayout {
                Layout.fillWidth: true
                Layout.preferredWidth: 1
                spacing: 3
                Label { text: "Altitude"; color: "#8ba2b2"; font.pixelSize: 8; horizontalAlignment: Text.AlignHCenter; Layout.fillWidth: true }
                Label { text: Number.isFinite(card.altitude) ? card.altitude.toFixed(1) + " m" : "—"; font.pixelSize: 13; horizontalAlignment: Text.AlignHCenter; Layout.fillWidth: true }
            }
            Item {
                Layout.fillWidth: true
                Layout.preferredWidth: 1
                implicitHeight: speedColumn.implicitHeight
                Accessible.role: Accessible.Button
                Accessible.name: "Toggle velocity units, currently " + vehicleLink.velocityUnit
                ColumnLayout {
                    id: speedColumn
                    width: parent.width
                    spacing: 3
                    Label { text: "Velocity"; color: "#8ba2b2"; font.pixelSize: 8; horizontalAlignment: Text.AlignHCenter; Layout.fillWidth: true }
                    Label {
                        text: Number.isFinite(card.velocity) ? (card.velocity * (vehicleLink.velocityUnit === "kph" ? 3.6 : 2.2369362921)).toFixed(1) + " " + vehicleLink.velocityUnit : "—"
                        font.pixelSize: 13
                        horizontalAlignment: Text.AlignHCenter
                        Layout.fillWidth: true
                    }
                }
                MouseArea { anchors.fill: parent; cursorShape: Qt.PointingHandCursor; onClicked: vehicleLink.toggleVelocityUnit() }
            }
        }
        Label {
            visible: card.errorMessage.length > 0
            text: card.errorMessage
            textFormat: Text.PlainText
            color: "#ffb6a2"
            font.pixelSize: 9
            wrapMode: Text.WordWrap
            maximumLineCount: 3
            elide: Text.ElideRight
            Layout.fillWidth: true
            Layout.topMargin: 3
            HoverHandler { id: errorHover }
            ToolTip {
                id: errorTip
                visible: errorHover.hovered
                delay: 500
                width: 320
                text: card.errorMessage
                background: Rectangle { color: "#203743"; border.color: "#557584"; radius: 5 }
                contentItem: Label { text: errorTip.text; textFormat: Text.PlainText; color: "#e5edf3"; wrapMode: Text.WordWrap; font.pixelSize: 11 }
            }
        }
    }
}
