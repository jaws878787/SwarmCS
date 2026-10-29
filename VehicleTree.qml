import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Rectangle {
    id: tree
    color: "#111d26"
    property int selectedSystemId: 0
    property string selectedName: ""
    function openVehicleMenu(systemId, name) {
        selectedSystemId = systemId
        selectedName = name
        vehicleMenu.popup()
    }
    Rectangle {
        id: titleCard
        anchors.top: parent.top
        anchors.left: parent.left
        anchors.right: parent.right
        height: 62
        color: "#162530"
        RowLayout {
            anchors.fill: parent
            anchors.margins: 18
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
            spacing: 12
            Repeater {
                model: vehicleLink.groups
                delegate: Column {
                    id: section
                    required property string modelData
                    property string groupTitle: modelData
                    property bool expanded: true
                    width: parent.width - 24
                    spacing: 8
                    Button {
                        width: parent.width
                        implicitHeight: 34
                        text: (section.expanded ? "▾  " : "▸  ") + section.groupTitle
                              + "  (" + (vehicleLink.groupCounts[section.groupTitle] || 0) + ")"
                        Accessible.name: section.groupTitle + " vehicle group"
                        onClicked: section.expanded = !section.expanded
                        background: Rectangle { radius: 5; color: parent.hovered ? "#253945" : "#172832" }
                        contentItem: Text { text: parent.text; color: "#a7bfcd"; font.pixelSize: 12; font.bold: true; verticalAlignment: Text.AlignVCenter; elide: Text.ElideRight }
                    }
                    Repeater {
                        model: vehicleLink
                        delegate: VehicleCard {
                            width: section.width
                            visible: section.expanded && vehicleGroup === section.groupTitle
                            onContextRequested: function(sysId, name) { tree.openVehicleMenu(sysId, name) }
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
        id: titleMenu
        popupType: Popup.Item
        MenuItem { text: "Clear vehicle names"; onTriggered: vehicleLink.clearVehicleNames() }
    }
    Menu {
        id: vehicleMenu
        popupType: Popup.Item
        cascade: true
        MenuItem {
            text: "Rename vehicle…"
            onTriggered: {
                renameField.text = tree.selectedName
                renameError.text = ""
                renameDialog.open()
                renameField.forceActiveFocus()
                renameField.selectAll()
            }
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
        id: renameDialog
        parent: Overlay.overlay
        anchors.centerIn: parent
        width: Math.min(380, parent.width - 32)
        modal: true
        title: "Rename vehicle · SYSID " + tree.selectedSystemId
        ColumnLayout {
            width: parent.width
            Label { text: "Vehicle name" }
            TextField { id: renameField; Layout.fillWidth: true; maximumLength: 64; placeholderText: "Vehicle name" }
            Label { id: renameError; color: "#ffb6a2"; visible: text.length > 0; wrapMode: Text.WordWrap; Layout.fillWidth: true }
        }
        footer: DialogButtonBox {
            standardButtons: DialogButtonBox.Cancel
            Button {
                text: "Save"
                enabled: renameField.text.trim().length > 0
                DialogButtonBox.buttonRole: DialogButtonBox.ActionRole
                onClicked: {
                    if (vehicleLink.renameVehicle(tree.selectedSystemId, renameField.text)) renameDialog.close()
                    else renameError.text = "The name could not be saved. Please try again."
                }
            }
            onRejected: renameDialog.close()
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
