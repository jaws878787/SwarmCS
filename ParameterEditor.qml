import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Dialog {
    id: editor
    parent: Overlay.overlay
    anchors.centerIn: parent
    width: Math.min(780, parent.width - 32)
    height: Math.min(700, parent.height - 48)
    modal: true
    title: "Parameters · " + groupName
    standardButtons: Dialog.Close
    property string groupName: ""
    property int systemId: 0
    property var vehicles: []
    property var entries: []
    property var drafts: ({})
    property string statusText: ""
    property bool ready: false
    property bool saving: vehicleLink.parameterSaveBusy
    property int editCount: Object.keys(drafts[systemId] || {}).length
    function refresh(rebuild) {
        const wasReady = ready
        ready = vehicleLink.parametersReady(systemId)
        statusText = vehicleLink.parameterStatus(systemId)
        if (rebuild || !ready || !wasReady) entries = vehicleLink.parameters(systemId, search.text)
    }
    function openGroup(group) {
        groupName = group
        vehicles = vehicleLink.groupVehicles(group)
        drafts = ({})
        vehicleSelector.currentIndex = vehicles.length > 0 ? 0 : -1
        systemId = vehicles.length > 0 ? vehicles[0].systemId : 0
        saveToGroup.checked = false
        vehicleLink.loadParameters(systemId)
        refresh(true)
        open()
    }
    function chooseVehicle(id) {
        systemId = id
        ready = false
        vehicleLink.loadParameters(id)
        refresh(true)
    }
    function stage(name, value) {
        const all = Object.assign({}, drafts)
        const changes = Object.assign({}, all[systemId] || {})
        changes[name] = value
        all[systemId] = changes
        drafts = all
    }
    function draftValue(name, fallback) {
        const changes = drafts[systemId] || {}
        return Object.prototype.hasOwnProperty.call(changes, name) ? changes[name] : fallback
    }
    Connections {
        target: vehicleLink
        function onParametersChanged() { if (editor.visible) editor.refresh(false) }
        function onChanged() { if (editor.visible) editor.refresh(false) }
        function onGroupsChanged() {
            if (!editor.visible || editor.saving) return
            editor.vehicles = vehicleLink.groupVehicles(editor.groupName)
            const index = editor.vehicles.findIndex(v => v.systemId === editor.systemId)
            vehicleSelector.currentIndex = index
            if (index < 0 && editor.vehicles.length > 0) {
                vehicleSelector.currentIndex = 0
                editor.chooseVehicle(editor.vehicles[0].systemId)
            }
            editor.refresh(false)
        }
        function onParametersSaved(sourceSystemId, confirmed) {
            const all = Object.assign({}, editor.drafts)
            const changes = Object.assign({}, all[sourceSystemId] || {})
            for (const name of confirmed) delete changes[name]
            all[sourceSystemId] = changes
            editor.drafts = all
            if (editor.visible) editor.refresh(true)
        }
    }
    ColumnLayout {
        anchors.fill: parent
        spacing: 10
        RowLayout {
            Layout.fillWidth: true
            Label { text: "Vehicle" }
            ComboBox {
                id: vehicleSelector
                Layout.fillWidth: true
                model: editor.vehicles
                textRole: "name"
                valueRole: "systemId"
                enabled: !editor.saving
                Accessible.name: "Vehicle to view parameters"
                onActivated: editor.chooseVehicle(currentValue)
            }
            Button {
                text: "Reload"
                enabled: !editor.saving && editor.systemId > 0
                onClicked: { vehicleLink.loadParameters(editor.systemId, true); editor.refresh(true) }
            }
        }
        TextField {
            id: search
            Layout.fillWidth: true
            placeholderText: "Search parameters…"
            Accessible.name: "Search parameters"
            onTextChanged: editor.refresh(true)
        }
        Label {
            text: editor.saving ? "Saving… waiting for vehicle confirmations" : editor.statusText
            color: "#a8bac8"
            Layout.fillWidth: true
            wrapMode: Text.WordWrap
        }
        RowLayout {
            Layout.fillWidth: true
            Label { text: "Parameter"; font.bold: true; Layout.fillWidth: true }
            Label { text: "Value"; font.bold: true; Layout.preferredWidth: 190 }
        }
        ListView {
            id: parameterList
            Layout.fillWidth: true
            Layout.fillHeight: true
            model: editor.entries
            clip: true
            spacing: 2
            ScrollBar.vertical: ScrollBar {}
            delegate: Rectangle {
                required property var modelData
                width: ListView.view.width
                height: 38
                color: index % 2 ? "#172a35" : "#1c303d"
                required property int index
                RowLayout {
                    anchors.fill: parent
                    anchors.leftMargin: 8
                    anchors.rightMargin: 14
                    Label {
                        text: modelData.name
                        textFormat: Text.PlainText
                        font.pixelSize: 12
                        elide: Text.ElideRight
                        Layout.fillWidth: true
                    }
                    TextField {
                        Layout.preferredWidth: 190
                        Layout.fillHeight: true
                        text: editor.draftValue(modelData.name, modelData.value)
                        enabled: editor.ready && !editor.saving && modelData.editable
                        selectByMouse: true
                        color: Object.prototype.hasOwnProperty.call(editor.drafts[editor.systemId] || {}, modelData.name) ? "#ffca82" : "#e5edf3"
                        Accessible.name: modelData.name + " value"
                        onTextEdited: editor.stage(modelData.name, text)
                    }
                }
            }
            Label { anchors.centerIn: parent; visible: parameterList.count === 0; text: editor.ready ? "No matching parameters" : "Waiting for parameters…"; color: "#91a9b9" }
        }
        Label {
            text: editor.editCount + " pending change(s) for this vehicle. Edits are sent only when you save."
            color: "#91a9b9"
            font.pixelSize: 11
            wrapMode: Text.WordWrap
            Layout.fillWidth: true
        }
        RowLayout {
            Layout.fillWidth: true
            Item { Layout.fillWidth: true }
            CheckBox {
                id: saveToGroup
                text: "Save to Group"
                enabled: !editor.saving
                ToolTip.visible: hovered
                ToolTip.text: "Apply this vehicle's pending values to connected group members that have each parameter."
            }
            Button {
                text: "Save"
                enabled: editor.ready && !editor.saving && editor.editCount > 0
                onClicked: vehicleLink.saveParameters(editor.systemId, editor.groupName, saveToGroup.checked, editor.drafts[editor.systemId] || {})
            }
        }
    }
}
