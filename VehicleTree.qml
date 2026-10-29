import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Rectangle {
    id: tree
    color: "#111d26"
    property int selectedSystemId: 0
    property var selectedVehicle: ({})
    property var iconChoices: []
    property var modeEntries: []
    property bool iconsExpanded: false
    property string selectedGroup: ""
    property var groupModeEntries: []
    function refreshSelected() {
        if (selectedSystemId > 0) selectedVehicle = vehicleLink.vehicleDetails(selectedSystemId)
    }
    function openSettings(systemId) {
        selectedSystemId = systemId
        refreshSelected()
        settingsName.text = selectedVehicle.nameInput
        settingsError.text = ""
        iconChoices = vehicleLink.iconOptions(systemId)
        iconsExpanded = false
        settingsDialog.open()
    }
    function openModes(systemId) {
        selectedSystemId = systemId
        refreshSelected()
        modeEntries = vehicleLink.availableModes(systemId)
        vehicleLink.refreshModes(systemId)
        modeDialog.open()
    }
    Connections {
        target: vehicleLink
        function onDataChanged() { tree.refreshSelected() }
        function onRowsRemoved() { tree.refreshSelected() }
        function onModelReset() { tree.refreshSelected() }
        function onModeListChanged(systemId) {
            if (groupModeDialog.visible) tree.groupModeEntries = vehicleLink.groupModes(tree.selectedGroup)
            if (systemId === tree.selectedSystemId) {
                tree.refreshSelected()
                if (modeDialog.visible) tree.modeEntries = vehicleLink.availableModes(systemId)
            }
        }
        function onOperationReport(title, message) {
            resultDialog.title = title
            resultText.text = message
            resultDialog.open()
        }
    }
    function openVehicleMenu(systemId) {
        selectedSystemId = systemId
        refreshSelected()
        vehicleMenu.popup()
    }
    Rectangle {
        id: titleCard
        anchors.top: parent.top
        anchors.left: parent.left
        anchors.right: parent.right
        height: 48
        color: "#162530"
        RowLayout {
            anchors.fill: parent
            anchors.margins: 12
            Label { text: "Vehicles"; font.pixelSize: 19; font.bold: true; Layout.fillWidth: true }
            Label { text: vehicleLink.count; color: "#91a9b9"; font.pixelSize: 13 }
        }
        TapHandler { acceptedButtons: Qt.RightButton; onTapped: titleMenu.popup() }
    }
    ScrollView {
        id: vehicleScroll
        anchors.top: titleCard.bottom
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.bottom: parent.bottom
        clip: true
        contentWidth: availableWidth
        ScrollBar.horizontal.policy: ScrollBar.AlwaysOff
        Column {
            width: vehicleScroll.availableWidth
            padding: 12
            spacing: 9
            Repeater {
                model: vehicleLink.groups
                delegate: Column {
                    id: section
                    required property string modelData
                    property string groupTitle: modelData
                    property bool expanded: true
                    width: parent.width - 24
                    spacing: 6
                    Button {
                        width: parent.width
                        implicitHeight: 28
                        text: (section.expanded ? "▾  " : "▸  ") + section.groupTitle
                              + "  (" + (vehicleLink.groupCounts[section.groupTitle] || 0) + ")"
                        Accessible.name: section.groupTitle + " vehicle group"
                        onClicked: section.expanded = !section.expanded
                        TapHandler {
                            acceptedButtons: Qt.RightButton
                            onTapped: { tree.selectedGroup = section.groupTitle; groupActionsMenu.popup() }
                        }
                        background: Rectangle { radius: 5; color: parent.hovered ? "#253945" : "#172832" }
                        contentItem: Text { text: parent.text; color: "#a7bfcd"; font.pixelSize: 12; font.bold: true; verticalAlignment: Text.AlignVCenter; elide: Text.ElideRight }
                    }
                    Repeater {
                        model: vehicleLink
                        delegate: VehicleCard {
                            width: section.width
                            visible: section.expanded && vehicleGroup === section.groupTitle
                            onContextRequested: function(sysId) { tree.openVehicleMenu(sysId) }
                            onSettingsRequested: function(sysId) { tree.openSettings(sysId) }
                            onModesRequested: function(sysId) { tree.openModes(sysId) }
                        }
                    }
                }
            }
            Label {
                visible: vehicleLink.count === 0
                width: parent.width - 24
                text: vehicleLink.listening ? "Waiting for vehicle heartbeats…" : "Connect to discover vehicles."
                color: "#8098a9"
                font.pixelSize: 12
                wrapMode: Text.WordWrap
                topPadding: 8
                bottomPadding: 8
            }
        }
    }
    Menu {
        id: groupActionsMenu
        popupType: Popup.Item
        MenuItem { text: "Arm all vehicles"; onTriggered: vehicleLink.groupAction(tree.selectedGroup, "arm") }
        MenuItem { text: "Disarm all vehicles"; onTriggered: vehicleLink.groupAction(tree.selectedGroup, "disarm") }
        MenuSeparator {}
        MenuItem { text: "Start missions"; onTriggered: vehicleLink.groupAction(tree.selectedGroup, "mission") }
        MenuItem { text: "RTL"; onTriggered: vehicleLink.groupAction(tree.selectedGroup, "rtl") }
        MenuItem {
            text: "Change flight mode…"
            onTriggered: {
                tree.groupModeEntries = vehicleLink.groupModes(tree.selectedGroup)
                for (const vehicle of vehicleLink.groupVehicles(tree.selectedGroup)) vehicleLink.refreshModes(vehicle.systemId)
                groupModeDialog.open()
            }
        }
        MenuSeparator {}
        MenuItem { text: "Parameters…"; onTriggered: parameterEditor.openGroup(tree.selectedGroup) }
    }
    ParameterEditor { id: parameterEditor }
    Dialog {
        id: resultDialog
        parent: Overlay.overlay
        anchors.centerIn: parent
        width: Math.min(660, parent.width - 32)
        height: Math.min(420, parent.height - 48)
        modal: true
        standardButtons: Dialog.Ok
        ScrollView {
            id: resultScroll
            anchors.fill: parent
            clip: true
            Label { id: resultText; width: resultScroll.availableWidth; textFormat: Text.PlainText; wrapMode: Text.WordWrap }
        }
    }
    Dialog {
        id: groupModeDialog
        parent: Overlay.overlay
        anchors.centerIn: parent
        width: Math.min(360, parent.width - 32)
        height: Math.min(540, parent.height - 48)
        modal: true
        title: "Flight mode · " + tree.selectedGroup
        standardButtons: Dialog.Close
        ColumnLayout {
            anchors.fill: parent
            Label { text: "Modes available on every connected group member"; wrapMode: Text.WordWrap; Layout.fillWidth: true; color: "#91a9b9" }
            ListView {
                Layout.fillWidth: true
                Layout.fillHeight: true
                clip: true
                model: tree.groupModeEntries
                ScrollBar.vertical: ScrollBar {}
                delegate: Button {
                    required property var modelData
                    width: ListView.view.width
                    text: modelData.name
                    onClicked: { groupModeDialog.close(); vehicleLink.groupAction(tree.selectedGroup, "mode", modelData.key) }
                }
                Label { anchors.centerIn: parent; visible: tree.groupModeEntries.length === 0; text: "No shared modes available" }
            }
        }
    }
    Menu {
        id: titleMenu
        popupType: Popup.Item
        MenuItem { text: "Clear vehicle names"; onTriggered: vehicleLink.clearVehicleNames() }
    }
    Menu {
        id: vehicleMenu
        popupType: Popup.Item
        cascade: true
        MenuItem {
            text: tree.selectedVehicle.armed ? "Disarm" : "Arm"
            enabled: tree.selectedVehicle.connected === true && !tree.selectedVehicle.commandPending
            onTriggered: vehicleLink.setArmed(tree.selectedSystemId, !tree.selectedVehicle.armed)
        }
        Menu {
            id: groupMenu
            popupType: Popup.Item
            title: "Add to group"
            Instantiator {
                model: vehicleLink.groups
                delegate: MenuItem {
                    required property string modelData
                    text: modelData
                    onTriggered: vehicleLink.moveToGroup(tree.selectedSystemId, modelData)
                }
                onObjectAdded: function(index, object) { groupMenu.insertItem(index, object) }
                onObjectRemoved: function(index, object) { groupMenu.removeItem(object) }
            }
            MenuSeparator {}
            MenuItem {
                text: "Create new group…"
                onTriggered: {
                    groupField.text = ""
                    groupError.text = ""
                    groupDialog.open()
                    groupField.forceActiveFocus()
                }
            }
        }
    }
    Dialog {
        id: settingsDialog
        parent: Overlay.overlay
        anchors.centerIn: parent
        width: Math.min(380, parent.width - 32)
        modal: true
        title: "Vehicle settings · SYSID " + tree.selectedSystemId
        standardButtons: Dialog.Close
        ColumnLayout {
            width: parent.width
            spacing: 12
            Label { text: "Vehicle name" }
            TextField {
                id: settingsName
                Layout.fillWidth: true
                maximumLength: 64
                placeholderText: "Vehicle name"
                onTextEdited: {
                    if (text.trim().length > 0) {
                        settingsError.text = vehicleLink.renameVehicle(tree.selectedSystemId, text)
                            ? "" : "The name could not be saved."
                    }
                }
            }
            Label {
                text: "Map label: " + (tree.selectedVehicle.vehicleName || "")
                textFormat: Text.PlainText
                color: "#91a9b9"
                font.pixelSize: 11
                wrapMode: Text.WordWrap
                Layout.fillWidth: true
            }
            Label { text: "Vehicle icon" }
            Button {
                Layout.fillWidth: true
                implicitHeight: 46
                onClicked: tree.iconsExpanded = !tree.iconsExpanded
                contentItem: RowLayout {
                    spacing: 10
                    VehicleIcon { kind: tree.selectedVehicle.iconId || "quad"; Layout.preferredWidth: 30; Layout.preferredHeight: 30 }
                    Label {
                        text: {
                            const selected = tree.iconChoices.find(item => item.id === tree.selectedVehicle.iconId)
                            return selected ? selected.name : "Choose icon"
                        }
                        Layout.fillWidth: true
                    }
                    Label { text: tree.iconsExpanded ? "▴" : "▾" }
                }
                Accessible.name: "Choose vehicle icon"
            }
            GridLayout {
                visible: tree.iconsExpanded
                columns: 2
                columnSpacing: 8
                rowSpacing: 8
                Layout.fillWidth: true
                Repeater {
                    model: tree.iconChoices
                    delegate: Button {
                        required property var modelData
                        Layout.fillWidth: true
                        Layout.preferredWidth: 1
                        implicitHeight: 78
                        checked: tree.selectedVehicle.iconId === modelData.id
                        Accessible.name: modelData.name + " icon"
                        onClicked: {
                            if (vehicleLink.setVehicleIcon(tree.selectedSystemId, modelData.id)) {
                                settingsError.text = ""
                                tree.iconsExpanded = false
                                tree.refreshSelected()
                            } else settingsError.text = "The icon could not be saved."
                        }
                        background: Rectangle {
                            radius: 6
                            color: parent.checked ? "#254b4b" : "#223541"
                            border.color: parent.checked ? "#66cbbb" : "#36505f"
                        }
                        contentItem: ColumnLayout {
                            spacing: 4
                            VehicleIcon { kind: modelData.id; Layout.preferredWidth: 34; Layout.preferredHeight: 34; Layout.alignment: Qt.AlignHCenter }
                            Label { text: modelData.name; Layout.alignment: Qt.AlignHCenter; font.pixelSize: 12 }
                        }
                    }
                }
            }
            Label { id: settingsError; color: "#ffb6a2"; visible: text.length > 0; wrapMode: Text.WordWrap; Layout.fillWidth: true }
            Label { text: "Changes are saved as you type."; color: "#91a9b9"; font.pixelSize: 11 }
        }
    }
    Dialog {
        id: modeDialog
        parent: Overlay.overlay
        anchors.centerIn: parent
        width: Math.min(360, parent.width - 32)
        height: Math.min(540, parent.height - 48)
        modal: true
        title: "Flight mode · " + (tree.selectedVehicle.vehicleName || "")
        standardButtons: Dialog.Close
        ColumnLayout {
            anchors.fill: parent
            spacing: 10
            Label {
                text: tree.selectedVehicle.reportedModes ? "Modes reported by this vehicle"
                    : "Firmware defaults · unavailable modes may be rejected by the vehicle."
                wrapMode: Text.WordWrap
                color: "#91a9b9"
                font.pixelSize: 12
                Layout.fillWidth: true
            }
            Label {
                visible: tree.selectedVehicle.commandPending === true
                text: tree.selectedVehicle.commandStatus || ""
                wrapMode: Text.WordWrap
                Layout.fillWidth: true
            }
            ListView {
                Layout.fillWidth: true
                Layout.fillHeight: true
                clip: true
                model: tree.modeEntries
                spacing: 3
                ScrollBar.vertical: ScrollBar {}
                delegate: Button {
                    required property var modelData
                    width: ListView.view.width
                    implicitHeight: 36
                    text: modelData.name
                    checked: modelData.standardMode ? tree.selectedVehicle.standardMode === modelData.standardMode
                        : tree.selectedVehicle.customMode === modelData.id
                    enabled: tree.selectedVehicle.connected === true && !tree.selectedVehicle.commandPending && modelData.enabled
                    onClicked: {
                        if (vehicleLink.setFlightMode(tree.selectedSystemId, modelData.id, modelData.standardMode))
                            modeDialog.close()
                    }
                }
            }
        }
    }
    Dialog {
        id: groupDialog
        parent: Overlay.overlay
        anchors.centerIn: parent
        width: Math.min(380, parent.width - 32)
        modal: true
        title: "Create vehicle group"
        ColumnLayout {
            width: parent.width
            Label { text: "Group name" }
            TextField { id: groupField; Layout.fillWidth: true; maximumLength: 48; placeholderText: "Group name" }
            Label { id: groupError; color: "#ffb6a2"; visible: text.length > 0; wrapMode: Text.WordWrap; Layout.fillWidth: true }
        }
        footer: DialogButtonBox {
            standardButtons: DialogButtonBox.Cancel
            Button {
                text: "Create"
                enabled: groupField.text.trim().length > 0
                DialogButtonBox.buttonRole: DialogButtonBox.ActionRole
                onClicked: {
                    const group = vehicleLink.createGroup(groupField.text)
                    if (group.length > 0) {
                        vehicleLink.moveToGroup(tree.selectedSystemId, group)
                        groupDialog.close()
                    } else groupError.text = "The group could not be saved. Please try again."
                }
            }
            onRejected: groupDialog.close()
        }
    }
}
