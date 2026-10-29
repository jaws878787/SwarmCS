import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtLocation
import QtPositioning

ApplicationWindow {
    id: window
    visible: true
    width: 1440
    height: 900
    minimumWidth: 640
    minimumHeight: 400
    title: "SwarmCS"
    color: "#101820"
    property bool satellite: true
    property var mapCenter: QtPositioning.coordinate(39.7684, -86.1581)
    property real mapZoom: 12
    property bool autoCenterAllowed: true
    property int searchDots: 0
    palette.window: "#17242f"
    palette.windowText: "#e5edf3"
    palette.base: "#101820"
    palette.text: "#e5edf3"
    palette.button: "#283b49"
    palette.buttonText: "#e5edf3"
    palette.highlight: "#39b6a7"
    Timer {
        interval: 400
        repeat: true
        running: vehicleLink.searching
        onRunningChanged: window.searchDots = 0
        onTriggered: window.searchDots = (window.searchDots + 1) % 4
    }
    PositionSource {
        id: computerPosition
        Component.onCompleted: {
            if (valid) update(15000)
            else computerLocation.request()
        }
        onPositionChanged: {
            if (window.autoCenterAllowed && position.coordinate.isValid) {
                window.mapCenter = position.coordinate
                window.autoCenterAllowed = false
            }
        }
    }
    Connections {
        target: computerLocation
        function onLocated(latitude, longitude) {
            if (window.autoCenterAllowed) {
                window.mapCenter = QtPositioning.coordinate(latitude, longitude)
                window.autoCenterAllowed = false
            }
        }
    }
    header: ToolBar {
        height: 64
        background: Rectangle { color: "#101820" }
        RowLayout {
            anchors.fill: parent
            anchors.leftMargin: 22
            anchors.rightMargin: 16
            spacing: 20
            Label { text: "SWARMCS"; color: "#f2f6f8"; font.pixelSize: 19; font.bold: true; font.letterSpacing: 2 }
            Rectangle { width: 1; height: 24; color: "#34414d" }
            Label {
                text: vehicleLink.status + (vehicleLink.searching ? ".".repeat(window.searchDots) : "")
                color: "#b6c6d3"
                font.pixelSize: 13
                elide: Text.ElideRight
                Layout.fillWidth: true
            }
            ToolButton {
                text: "\u2699"
                font.pixelSize: 26
                implicitWidth: 40
                implicitHeight: 38
                Accessible.name: "Connection settings"
                ToolTip.visible: hovered
                ToolTip.text: "Connection settings"
                onClicked: connectionSettings.open()
            }
            Button {
                text: vehicleLink.listening ? "Disconnect" : "Connect"
                onClicked: vehicleLink.toggle()
                implicitWidth: 120
                implicitHeight: 38
                background: Rectangle { radius: 6; color: parent.down ? "#258a82" : "#39b6a7" }
                contentItem: Text { text: parent.text; color: "#071b1b"; font.bold: true; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
            }
        }
    }
    Plugin {
        id: streetTiles
        name: "osm"
        PluginParameter { name: "osm.mapping.cache.directory"; value: mapCacheRoot + "/street" }
        PluginParameter { name: "osm.useragent"; value: "SwarmCS/0.1 (https://github.com/jaws878787/SwarmCS)" }
        PluginParameter { name: "osm.mapping.providersrepository.disabled"; value: true }
        PluginParameter { name: "osm.mapping.custom.host"; value: "https://tile.openstreetmap.org/" }
        PluginParameter { name: "osm.mapping.custom.mapcopyright"; value: "© <a href='https://www.openstreetmap.org/copyright'>OpenStreetMap</a> contributors" }
        PluginParameter { name: "osm.mapping.prefetching_style"; value: "NoPrefetching" }
    }
    Plugin {
        id: satelliteTiles
        name: "osm"
        PluginParameter { name: "osm.mapping.cache.directory"; value: mapCacheRoot + "/satellite" }
        PluginParameter { name: "osm.useragent"; value: "SwarmCS/0.1 (https://github.com/jaws878787/SwarmCS)" }
        PluginParameter { name: "osm.mapping.providersrepository.disabled"; value: true }
        PluginParameter { name: "osm.mapping.custom.host"; value: "https://services.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/%z/%y/%x.png" }
        PluginParameter { name: "osm.mapping.custom.mapcopyright"; value: "Imagery © <a href='https://www.arcgis.com/home/item.html?id=10df2279f9684e4a9f6a7f08febac2a9'>Esri</a>, Maxar, Earthstar Geographics and the GIS User Community" }
        PluginParameter { name: "osm.mapping.prefetching_style"; value: "NoPrefetching" }
    }
    Component { id: satelliteView; FlightMap { viewState: window; plugin: satelliteTiles } }
    Component { id: streetView; FlightMap { viewState: window; plugin: streetTiles } }
    Loader {
        id: mapLoader
        anchors.fill: parent
        sourceComponent: window.satellite ? satelliteView : streetView
    }
    Button {
        anchors.right: parent.right
        anchors.bottom: parent.bottom
        anchors.margins: 20
        text: window.satellite ? "Street map" : "Satellite"
        Accessible.name: window.satellite ? "Switch to street map" : "Switch to satellite imagery"
        implicitWidth: 126
        implicitHeight: 42
        onClicked: window.satellite = !window.satellite
    }
    Dialog {
        id: connectionSettings
        parent: Overlay.overlay
        anchors.centerIn: parent
        width: Math.min(420, window.width - 32)
        modal: true
        title: "Connection options"
        standardButtons: Dialog.Close
        ColumnLayout {
            width: parent.width
            spacing: 14
            Label {
                text: "Preview options — connections still use UDP 14550."
                wrapMode: Text.WordWrap
                color: "#a8bac8"
                Layout.fillWidth: true
            }
            Label { text: "Connection type" }
            ComboBox { model: ["UDP", "TCP", "Serial"]; Layout.fillWidth: true }
            Label { text: "Listen address" }
            TextField { text: "0.0.0.0"; placeholderText: "Listen address"; Layout.fillWidth: true }
            Label { text: "Port" }
            SpinBox { from: 1; to: 65535; value: 14550; editable: true; Layout.fillWidth: true }
            CheckBox { text: "Connect automatically on startup" }
        }
    }
}
