import QtQuick

Canvas {
    id: icon
    property string kind: "quad"
    property color tint: "#66e0ca"
    onKindChanged: requestPaint()
    onTintChanged: requestPaint()
    onWidthChanged: requestPaint()
    onHeightChanged: requestPaint()
    onPaint: {
        const ctx = getContext("2d")
        ctx.reset()
        ctx.clearRect(0, 0, width, height)
        ctx.scale(width / 48, height / 48)
        ctx.strokeStyle = tint
        ctx.fillStyle = tint
        ctx.lineWidth = 2.5
        ctx.lineJoin = "round"
        function polygon(points) {
            ctx.beginPath()
            ctx.moveTo(points[0][0], points[0][1])
            for (let i = 1; i < points.length; ++i) ctx.lineTo(points[i][0], points[i][1])
            ctx.closePath()
            ctx.fill()
        }
        if (kind === "fixed") {
            polygon([[24,2],[27,20],[45,29],[45,33],[27,28],[27,39],[33,43],[33,46],[24,43],[15,46],[15,43],[21,39],[21,28],[3,33],[3,29],[21,20]])
        } else if (kind === "delta") {
            polygon([[24,3],[46,42],[29,36],[24,42],[19,36],[2,42]])
        } else if (kind === "wing") {
            polygon([[24,12],[46,32],[43,39],[27,30],[24,34],[21,30],[5,39],[2,32]])
        } else if (kind === "bullet") {
            polygon([[24,2],[30,12],[30,35],[34,44],[27,41],[24,46],[21,41],[14,44],[18,35],[18,12]])
            ctx.beginPath(); ctx.moveTo(7,20); ctx.lineTo(41,20); ctx.stroke()
            ctx.beginPath(); ctx.arc(9,20,5,0,Math.PI*2); ctx.stroke()
            ctx.beginPath(); ctx.arc(39,20,5,0,Math.PI*2); ctx.stroke()
        } else {
            const motors = kind === "hex" ? 6 : kind === "octo" ? 8 : 4
            const offset = motors === 4 ? -Math.PI/4 : -Math.PI/2
            for (let i = 0; i < motors; ++i) {
                const angle = offset + i * Math.PI * 2 / motors
                const x = 24 + Math.cos(angle) * 16, y = 24 + Math.sin(angle) * 16
                ctx.beginPath(); ctx.moveTo(24,24); ctx.lineTo(x,y); ctx.stroke()
                ctx.beginPath(); ctx.arc(x,y,motors === 8 ? 4.5 : 5.5,0,Math.PI*2); ctx.stroke()
            }
            polygon([[24,14],[30,24],[27,32],[21,32],[18,24]])
        }
    }
}
