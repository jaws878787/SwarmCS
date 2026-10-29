import QtQuick
import QtLocation

Map {
    id: map
    required property var viewState
    activeMapType: supportedMapTypes.length > 0 ? supportedMapTypes[supportedMapTypes.length - 1] : null
    center: viewState.mapCenter
    zoomLevel: viewState.mapZoom
    copyrightsVisible: true
    onCenterChanged: { if (viewState) viewState.mapCenter = center }
    onZoomLevelChanged: { if (viewState) viewState.mapZoom = zoomLevel }
    DragHandler {
        target: null
        onActiveChanged: { if (active) map.viewState.autoCenterAllowed = false }
        onTranslationChanged: function(delta) { map.pan(-delta.x, -delta.y) }
    }
    WheelHandler {
        onWheel: function(event) {
            map.viewState.autoCenterAllowed = false
            const coordinate = map.toCoordinate(Qt.point(event.x, event.y), false)
            map.zoomLevel = Math.max(map.minimumZoomLevel, Math.min(map.maximumZoomLevel, map.zoomLevel + event.angleDelta.y / 120))
            map.alignCoordinateToPoint(coordinate, Qt.point(event.x, event.y))
        }
    }
    PinchHandler {
        target: null
        property var startCoordinate
        onActiveChanged: {
            if (active) {
                map.viewState.autoCenterAllowed = false
                startCoordinate = map.toCoordinate(centroid.position, false)
            }
        }
        onScaleChanged: function(delta) {
            map.zoomLevel += Math.log2(delta)
            map.alignCoordinateToPoint(startCoordinate, centroid.position)
        }
    }
}
