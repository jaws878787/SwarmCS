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
    property string pendingGroupFit: ""
    function frameGroup(group) {
        if (!mapReady) { pendingGroupFit = group; return }
        const members = []
        for (let i = 0; i < mapItems.length; ++i) {
            const item = mapItems[i]
            if ((group === "__selected__" ? vehicleLink.selectedIds.indexOf(item.systemId) >= 0 : item.vehicleGroup === group) && item.positionValid) members.push(item)
        }
        if (members.length === 0) return
        fitViewportToMapItems(members)
        const fitted = Math.max(1, Math.min(17, Math.floor(zoomLevel) - 1))
        viewState.mapCenter = center
        viewState.selectZoom(21 - fitted)
        zoomLevel = Qt.binding(function() { return map.viewState.mapZoom })
        pendingGroupFit = ""
    }
    onMapReadyChanged: { if (mapReady && pendingGroupFit.length > 0) frameGroup(pendingGroupFit) }
    MapItemView {
        model: vehicleLink.planActive ? vehicleLink.planPaths : []
        delegate: MapPolyline {
            required property var modelData
            line.width: 2; line.color: "#ffb360"
            path: modelData.path.map(p => QtPositioning.coordinate(p.latitude,p.longitude))
        }
    }
    MapPolyline {
        visible: vehicleLink.planActive
        line.width: 5; line.color: "#31e4ee"
        path: vehicleLink.planCenterPath.map(p => QtPositioning.coordinate(p.latitude,p.longitude))
    }
    MapItemView {
        model: vehicleLink.planActive ? vehicleLink.planCenterPath : []
        delegate: MapQuickItem {
            required property var modelData
            required property int index
            coordinate: QtPositioning.coordinate(modelData.latitude,modelData.longitude)
            anchorPoint.x: 11; anchorPoint.y: 11
            sourceItem: Rectangle {
                width: 22; height: 22; radius: 11; color: "#102a31"; border.color: "#31e4ee"; border.width: 2
                Label { anchors.centerIn: parent; text: index===0 ? "T" : index; font.pixelSize: 11; color: "#31e4ee" }
            }
        }
    }
    MapItemView {
        model: vehicleLink.navigationCenters
        delegate: MapQuickItem {
            required property var modelData
            coordinate: QtPositioning.coordinate(modelData.latitude,modelData.longitude)
            anchorPoint.x: 12; anchorPoint.y: 12
            sourceItem: Item {
                width: 24; height: 24
                Rectangle { anchors.fill: parent; radius: 12; color: "#d9101820"; border.color: "#ffca82"; border.width: 2 }
                Label { anchors.centerIn: parent; text: "+"; color: "#ffca82"; font.pixelSize: 20 }
                Label { x: 28; y: 3; text: "Go Here"; color: "#ffca82"; font.pixelSize: 12 }
            }
        }
    }
    MapItemView {
        model: vehicleLink.navigationTargets
        delegate: MapPolyline {
            required property var modelData
            line.width: 2
            line.color: "#ffca82"
            path: [QtPositioning.coordinate(modelData.fromLatitude,modelData.fromLongitude),
                   QtPositioning.coordinate(modelData.latitude,modelData.longitude)]
        }
    }
    MapItemView {
        model: vehicleLink.navigationTargets
        delegate: MapQuickItem {
            required property var modelData
            coordinate: QtPositioning.coordinate(modelData.latitude,modelData.longitude)
            anchorPoint.x: 8
            anchorPoint.y: 8
            sourceItem: Item {
                width: 16; height: 16
                Rectangle { anchors.fill: parent; rotation: 45; color: "#b3101820"; border.color: "#ffca82"; border.width: 2 }
                Label { x: 20; y: -2; text: modelData.name + " destination"; textFormat: Text.PlainText; color: "#ffca82"; font.pixelSize: 11 }
            }
        }
    }
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
            required property string vehicleGroup
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
                    tint: vehicleLink.selectedIds.indexOf(marker.systemId) >= 0 ? "#ffca82" : "#66e0ca"
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
    TapHandler {
        acceptedButtons: Qt.LeftButton
        onTapped: function(eventPoint) {
            const coordinate = map.toCoordinate(eventPoint.position, false)
            if (!coordinate.isValid) return
            if (vehicleLink.planActive) { vehicleLink.addPlanWaypoint(coordinate.latitude,coordinate.longitude); return }
            goHereMenu.latitude = coordinate.latitude
            goHereMenu.longitude = coordinate.longitude
            goHereMenu.systemId = vehicleLink.selectedSystemId
            goHereMenu.groupName = vehicleLink.selectedGroup
            goHereMenu.targetName = goHereMenu.groupName === "__selected__" ? "Selected" : goHereMenu.groupName.length > 0 ? goHereMenu.groupName
                : (vehicleLink.vehicleDetails(goHereMenu.systemId).vehicleName || "Select a vehicle or group")
            goHereMenu.popup()
        }
    }
    Menu {
        id: goHereMenu
        popupType: Popup.Item
        property double latitude
        property double longitude
        property int systemId: 0
        property string groupName: ""
        property string targetName: ""
        MenuItem {
            text: "Go Here · " + goHereMenu.targetName
            enabled: goHereMenu.systemId > 0 || goHereMenu.groupName.length > 0
            onTriggered: {
                if (goHereMenu.groupName.length > 0) vehicleLink.goToGroup(goHereMenu.groupName,goHereMenu.latitude,goHereMenu.longitude)
                else vehicleLink.goTo(goHereMenu.systemId,goHereMenu.latitude,goHereMenu.longitude)
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
