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
        ComboBox {
            Layout.fillWidth: true
            model: vehicleLink.planVehicles; textRole: "name"; valueRole: "systemId"
            currentIndex: vehicleLink.planVehicles.findIndex(v => v.systemId === vehicleLink.planVehicleId)
            Accessible.name: "Mission vehicle"
            onActivated: vehicleLink.setPlanVehicleId(currentValue)
        }
        Button { text: "Add RTL"; enabled: !vehicleLink.missionBusy; Layout.fillWidth: true; onClicked: vehicleLink.addPlanRtl() }
        Label {
            text: vehicleLink.planStatus; textFormat: Text.PlainText; wrapMode: Text.WordWrap
            color: vehicleLink.planValid ? "#71cdbd" : "#ffca82"; Layout.fillWidth: true; font.pixelSize: 12
        }
        Label {
            text: "Cyan: clicked center route · Orange: vehicle routes\nCrossings use separate altitude bands.\nNominal timing, no live arrival barrier"
            color: "#91a9b9"; wrapMode: Text.WordWrap; Layout.fillWidth: true; font.pixelSize: 10
        }
        ListView {
            id: itemScroll
            Layout.fillWidth: true; Layout.fillHeight: true; clip: true; spacing: 8
            model: vehicleLink.planItems
            currentIndex: vehicleLink.selectedPlanItem
            boundsBehavior: Flickable.StopAtBounds
            header: Item { width: 1; height: itemScroll.height/2 }
            footer: Item { width: 1; height: itemScroll.height/2 }
            ScrollBar.vertical: ScrollBar { policy: ScrollBar.AsNeeded }
            function centerSelected() {
                if (currentIndex >= 0 && currentIndex < count) positionViewAtIndex(currentIndex,ListView.Center)
                else if (count > 0) positionViewAtIndex(0,ListView.Beginning)
            }
            onCountChanged: Qt.callLater(centerSelected)
            onHeightChanged: Qt.callLater(centerSelected)
            Connections {
                target: vehicleLink
                function onPlanSelectionChanged() { Qt.callLater(itemScroll.centerSelected) }
            }
            delegate: Rectangle {
                        id: rowCard
                        required property var modelData
                        required property int index
                        width: itemScroll.width; height: content.implicitHeight+20; radius: 5
                        onHeightChanged: { if (vehicleLink.selectedPlanItem === index) Qt.callLater(itemScroll.centerSelected) }
                        color: vehicleLink.selectedPlanItem === index ? "#25413f" : "#1a2a36"; border.color: vehicleLink.selectedPlanItem === index ? "#31e4ee" : "#304451"
                        TapHandler { onTapped: vehicleLink.selectPlanItem(rowCard.index) }
                        ColumnLayout {
                            id: content
                            anchors.left: parent.left; anchors.right: parent.right; anchors.top: parent.top; anchors.margins: 10
                            spacing: 6
                            RowLayout {
                                Layout.fillWidth: true
                                Label {
                                    text: rowCard.modelData.title; textFormat: Text.PlainText; wrapMode: Text.WordWrap; font.bold: true; Layout.fillWidth: true
                                    TapHandler { onTapped: { rowCard.forceActiveFocus(); vehicleLink.selectPlanItem(rowCard.index) } }
                                }
                                ToolButton { text: "×"; visible: rowCard.modelData.deletable; enabled: !vehicleLink.missionBusy; onClicked: vehicleLink.removePlanItem(rowCard.index) }
                            }
                            Label {
                                visible: rowCard.modelData.editable
                                text: Number(rowCard.modelData.latitude || 0).toFixed(6) + ", " + Number(rowCard.modelData.longitude || 0).toFixed(6)
                                color: "#91a9b9"; font.pixelSize: 10
                            }
                            RowLayout {
                                visible: !rowCard.modelData.isRtl && (rowCard.modelData.editable || rowCard.modelData.hasAltitude)
                                Layout.fillWidth: true
                                Label { text: rowCard.modelData.editable ? "Base altitude · Relative" : "Altitude · frame " + rowCard.modelData.frame; font.pixelSize: 10; Layout.fillWidth: true }
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
                                visible: !rowCard.modelData.isRtl && (rowCard.modelData.editable ? rowCard.index>0 : rowCard.modelData.speed>0)
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
                            Label {
                                visible: rowCard.modelData.editable
                                text: "Assigned altitude: " + Number(rowCard.modelData.assignedAltitude*(panel.units.altitudeMetric ? 1 : 1/0.3048)).toFixed(1) + (panel.units.altitudeMetric ? " m" : " ft") + " relative"
                                color: "#ffbc78"; font.pixelSize: 10
                                Layout.fillWidth: true; wrapMode: Text.WordWrap
                            }
                            ToolButton {
                                id: commandToggle
                                visible: (rowCard.modelData.commands || []).length > 0
                                text: (checked ? "▾ " : "▸ ") + "Vehicle commands"
                                checkable: true; font.pixelSize: 10
                            }
                            ColumnLayout {
                                visible: commandToggle.checked
                                Layout.fillWidth: true
                                Repeater {
                                    model: rowCard.modelData.commands || []
                                    delegate: Label {
                                        required property var modelData
                                        Layout.fillWidth: true; wrapMode: Text.WordWrap; color: "#91a9b9"; font.pixelSize: 10
                                        text: modelData.sequence + " · " + (modelData.command===178
                                            ? (["Air", "Ground", "Climb", "Descent"][modelData.speedType] || "") + " speed " + (modelData.speed*(panel.units.speedMetric ? 3.6 : 2.2369362921)).toFixed(1) + (panel.units.speedMetric ? " kph" : " mph")
                                            : modelData.command===93 ? "Hold " + modelData.hold.toFixed(1) + " s"
                                            : modelData.command===20 ? "RTL"
                                            : (modelData.command===22 ? "Takeoff " : "Waypoint ") + (modelData.altitude*(panel.units.altitudeMetric ? 1 : 1/0.3048)).toFixed(1) + (panel.units.altitudeMetric ? " m" : " ft"))
                                    }
                                }
                            }
                        }
                    }
        }
    }
}
