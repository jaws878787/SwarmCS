import QtQuick
import QtLocation
import QtPositioning
import QtQuick.Controls

Map {
    id: map
    required property var viewState
    activeMapType: supportedMapTypes.length > 0 ? supportedMapTypes[supportedMapTypes.length - 1] : null
    center: viewState.mapCenter
    zoomLevel: viewState.mapZoom
    copyrightsVisible: true
    onCenterChanged: { if (viewState) viewState.mapCenter = center }
    MapItemView {
        model: vehicleLink
        delegate: MapQuickItem {
            id: marker
            required property int systemId
            required property string vehicleName
            required property string iconId
            required property double latitude
            required property double longitude
            required property double heading
            required property bool positionValid
            visible: positionValid
            coordinate: positionValid ? QtPositioning.coordinate(latitude, longitude) : QtPositioning.coordinate()
            anchorPoint.x: markerContent.width / 2
            anchorPoint.y: 18
            sourceItem: Item {
                id: markerContent
                width: Math.max(40, markerName.implicitWidth + 14)
                height: 58
                VehicleIcon {
                    anchors.top: parent.top
                    anchors.horizontalCenter: parent.horizontalCenter
                    width: 36
                    height: 36
                    kind: marker.iconId
                    tint: map.viewState.selectedSystemId === marker.systemId ? "#ffca82" : "#66e0ca"
                    rotation: marker.heading
                }
                Rectangle {
                    anchors.top: parent.top
                    anchors.topMargin: 38
                    width: parent.width
                    height: 20
                    radius: 4
                    color: "#e6101820"
                    Label { id: markerName; anchors.centerIn: parent; text: marker.vehicleName; textFormat: Text.PlainText; color: "#f3f7fa"; font.pixelSize: 11 }
                }
            }
        }
    }
    DragHandler {
        target: null
        onActiveChanged: { if (active) map.viewState.autoCenterAllowed = false }
        onTranslationChanged: function(delta) { map.pan(-delta.x, -delta.y) }
    }
    WheelHandler {
        acceptedDevices: PointerDevice.Mouse | PointerDevice.TouchPad
        property real accumulatedSteps: 0
        onWheel: function(event) {
            map.viewState.autoCenterAllowed = false
            const coordinate = map.toCoordinate(Qt.point(event.x, event.y), false)
            accumulatedSteps += event.angleDelta.y !== 0 ? event.angleDelta.y / 120 : event.pixelDelta.y / 100
            const steps = Math.trunc(accumulatedSteps)
            if (steps === 0) return
            accumulatedSteps -= steps
            map.viewState.selectZoom(map.viewState.zoomStep - steps)
            map.alignCoordinateToPoint(coordinate, Qt.point(event.x, event.y))
            event.accepted = true
        }
    }
    PinchHandler {
        target: null
        acceptedDevices: PointerDevice.TouchPad | PointerDevice.TouchScreen
        property real gestureZoom: 0
        property var startCoordinate
        onActiveChanged: {
            if (active) {
                map.viewState.autoCenterAllowed = false
                startCoordinate = map.toCoordinate(centroid.position, false)
                gestureZoom = map.viewState.mapZoom
            }
        }
        onScaleChanged: function(delta) {
            gestureZoom = Math.max(1, Math.min(20, gestureZoom + Math.log2(delta)))
            map.viewState.selectZoom(21 - gestureZoom)
            map.alignCoordinateToPoint(startCoordinate, centroid.position)
        }
    }
}
