import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Rectangle {
    id: panel
    required property var units
    color: "#111d26"
    ColumnLayout {
        anchors.fill: parent; anchors.margins: 12; spacing: 10
        Label { text: "Mission items"; font.pixelSize: 19; font.bold: true }
        Label {
            text: vehicleLink.planStatus; textFormat: Text.PlainText; wrapMode: Text.WordWrap
            color: vehicleLink.planValid ? "#71cdbd" : "#ffca82"; Layout.fillWidth: true; font.pixelSize: 12
        }
        Label {
            text: "Cyan: clicked center route · Orange: vehicle routes\nRelative altitude · nominal timing, no live arrival barrier"
            color: "#91a9b9"; wrapMode: Text.WordWrap; Layout.fillWidth: true; font.pixelSize: 10
        }
        ScrollView {
            id: itemScroll
            Layout.fillWidth: true; Layout.fillHeight: true; clip: true; contentWidth: availableWidth
            Column {
                width: itemScroll.availableWidth; spacing: 8
                Repeater {
                    model: vehicleLink.planItems
                    delegate: Rectangle {
                        id: rowCard
                        required property var modelData
                        required property int index
                        width: parent.width; height: content.implicitHeight+20; radius: 5
                        color: "#1a2a36"; border.color: "#304451"
                        ColumnLayout {
                            id: content
                            anchors.left: parent.left; anchors.right: parent.right; anchors.top: parent.top; anchors.margins: 10
                            spacing: 6
                            RowLayout {
                                Layout.fillWidth: true
                                Label { text: rowCard.modelData.title; textFormat: Text.PlainText; wrapMode: Text.WordWrap; font.bold: true; Layout.fillWidth: true }
                                ToolButton { text: "×"; visible: rowCard.modelData.editable && rowCard.index>0; enabled: !vehicleLink.missionBusy; onClicked: vehicleLink.removePlanItem(rowCard.index) }
                            }
                            RowLayout {
                                Layout.fillWidth: true
                                Label { text: rowCard.modelData.editable ? "Altitude · Relative" : "Altitude · frame " + rowCard.modelData.frame; font.pixelSize: 10; Layout.fillWidth: true }
                                TextField {
                                    id: altitude
                                    Layout.preferredWidth: 68; implicitHeight: 30
                                    text: (rowCard.modelData.altitude*(panel.units.altitudeMetric ? 1 : 1/0.3048)).toFixed(1)
                                    enabled: rowCard.modelData.editable && !vehicleLink.missionBusy
                                    validator: DoubleValidator { bottom: panel.units.altitudeMetric ? 0.3 : 0.9843; top: panel.units.altitudeMetric ? 500 : 1640.41; locale: "en_US" }
                                    onEditingFinished: if (acceptableInput) vehicleLink.editPlanItem(rowCard.index,Number(text)*(panel.units.altitudeMetric ? 1/0.3048 : 1),rowCard.modelData.speed)
                                }
                                ToolButton { text: panel.units.altitudeMetric ? "m" : "ft"; onClicked: panel.units.altitudeMetric=!panel.units.altitudeMetric }
                            }
                            RowLayout {
                                visible: rowCard.index>0 || !rowCard.modelData.editable
                                Label { text: "Incoming speed"; font.pixelSize: 10; Layout.fillWidth: true }
                                TextField {
                                    Layout.preferredWidth: 68; implicitHeight: 30
                                    text: (rowCard.modelData.speed*(panel.units.speedMetric ? 3.6 : 2.2369362921)).toFixed(1)
                                    enabled: rowCard.modelData.editable && !vehicleLink.missionBusy
                                    validator: DoubleValidator { bottom: panel.units.speedMetric ? 0.36 : 0.224; top: panel.units.speedMetric ? 108 : 67.1; locale: "en_US" }
                                    onEditingFinished: if (acceptableInput) vehicleLink.editPlanItem(rowCard.index,rowCard.modelData.altitude/0.3048,Number(text)/(panel.units.speedMetric ? 3.6 : 2.2369362921))
                                }
                                ToolButton { text: panel.units.speedMetric ? "kph" : "mph"; onClicked: panel.units.speedMetric=!panel.units.speedMetric }
                            }
                        }
                    }
                }
            }
        }
    }
}
