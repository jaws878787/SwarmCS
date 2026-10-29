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
                text: vehicleLink.status
                color: "#b6c6d3"
                font.pixelSize: 13
                elide: Text.ElideRight
                Layout.fillWidth: true
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
    Plugin { id: osm; name: "osm" }
    Map {
        id: map
        anchors.fill: parent
        plugin: osm
        center: QtPositioning.coordinate(39.7684, -86.1581)
        zoomLevel: 12
        copyrightsVisible: true
        DragHandler {
            target: null
            onTranslationChanged: function(delta) { map.pan(-delta.x, -delta.y) }
        }
        WheelHandler {
            onWheel: function(event) {
                const coordinate = map.toCoordinate(Qt.point(event.x, event.y), false)
                map.zoomLevel = Math.max(map.minimumZoomLevel, Math.min(map.maximumZoomLevel, map.zoomLevel + event.angleDelta.y / 120))
                map.alignCoordinateToPoint(coordinate, Qt.point(event.x, event.y))
            }
        }
        PinchHandler {
            target: null
            property var startCoordinate
            onActiveChanged: { if (active) startCoordinate = map.toCoordinate(centroid.position, false) }
            onScaleChanged: function(delta) {
                map.zoomLevel += Math.log2(delta)
                map.alignCoordinateToPoint(startCoordinate, centroid.position)
            }
        }
    }
}
