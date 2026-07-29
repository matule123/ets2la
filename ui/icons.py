"""Small original monochrome line icons for UltraPilot navigation."""

from PyQt6.QtCore import Qt, QRectF, QPointF
from PyQt6.QtGui import (QIcon, QPixmap, QPainter, QPen, QPolygonF, QColor,
                         QPainterPath)


def line_icon(name: str, color="#4B5563", size=22) -> QIcon:
    pm = QPixmap(size, size)
    pm.fill(Qt.GlobalColor.transparent)
    p = QPainter(pm)
    p.setRenderHint(QPainter.RenderHint.Antialiasing)
    # All paths below use a 22×22 design grid. Scale the painter rather than
    # only enlarging the canvas; otherwise 24/30 px icons sit visibly high-left.
    p.scale(float(size) / 22.0, float(size) / 22.0)
    pen = QPen()
    pen.setColor(QColor(color))
    pen.setWidthF(1.7)
    pen.setCapStyle(Qt.PenCapStyle.RoundCap)
    pen.setJoinStyle(Qt.PenJoinStyle.RoundJoin)
    p.setPen(pen); p.setBrush(Qt.BrushStyle.NoBrush)
    if name == "dashboard":
        roof = QPainterPath()
        roof.moveTo(3.5, 10.0); roof.lineTo(11.0, 3.8)
        roof.lineTo(18.5, 10.0)
        p.drawPath(roof)
        p.drawRoundedRect(QRectF(5.5, 9.0, 11.0, 9.0), 1.8, 1.8)
        p.drawLine(QPointF(9.0, 18.0), QPointF(9.0, 13.5))
        p.drawLine(QPointF(13.0, 13.5), QPointF(13.0, 18.0))
    elif name == "navigation":
        p.drawPolygon(QPolygonF([QPointF(11,3), QPointF(18,18), QPointF(11,15), QPointF(4,18)]))
        p.drawLine(QPointF(11,15), QPointF(11,8))
    elif name == "visualization":
        p.drawRoundedRect(QRectF(3,4,16,12),2,2); p.drawLine(7,19,15,19); p.drawLine(11,16,11,19)
    elif name == "plugins":
        for y, knob_x in ((6.0, 8.0), (11.0, 14.0), (16.0, 10.5)):
            p.drawLine(QPointF(4.0, y), QPointF(18.0, y))
            p.setBrush(QColor(color))
            p.drawEllipse(QRectF(knob_x - 1.7, y - 1.7, 3.4, 3.4))
            p.setBrush(Qt.BrushStyle.NoBrush)
    elif name == "performance":
        p.drawLine(QPointF(4.0, 17.5), QPointF(4.0, 12.5))
        p.drawLine(QPointF(8.5, 17.5), QPointF(8.5, 8.5))
        p.drawLine(QPointF(13.0, 17.5), QPointF(13.0, 5.0))
        p.drawLine(QPointF(17.5, 17.5), QPointF(17.5, 10.5))
        p.drawLine(QPointF(3.0, 18.5), QPointF(19.0, 18.5))
    elif name == "autopilot":
        # Lane-assist mark: the master action is clearer as a road/heading
        # symbol than as a steering wheel (which looked like a settings dial).
        p.drawLine(QPointF(5.0, 19.0), QPointF(8.0, 3.5))
        p.drawLine(QPointF(17.0, 19.0), QPointF(14.0, 3.5))
        p.drawLine(QPointF(11.0, 18.5), QPointF(11.0, 14.5))
        p.drawLine(QPointF(11.0, 11.0), QPointF(11.0, 7.0))
        p.drawPolygon(QPolygonF([QPointF(8.7, 8.5), QPointF(11.0, 5.5),
                                 QPointF(13.3, 8.5)]))
    elif name == "steering":
        p.drawArc(QRectF(4.0, 4.0, 14.0, 14.0), 20 * 16, 140 * 16)
        p.drawArc(QRectF(4.0, 4.0, 14.0, 14.0), 200 * 16, 140 * 16)
        p.drawLine(QPointF(7.0, 12.0), QPointF(15.0, 12.0))
        p.drawLine(QPointF(11.0, 12.0), QPointF(11.0, 17.2))
        p.drawEllipse(QRectF(9.2, 10.2, 3.6, 3.6))
    elif name == "settings":
        # Three balanced tuning sliders avoid the generic gear icon and match
        # the controls users actually find on the page.
        for y, knob_x in ((5.5, 8.0), (11.0, 14.0), (16.5, 10.5)):
            p.drawLine(QPointF(3.5, y), QPointF(18.5, y))
            p.setBrush(QColor(color))
            p.drawEllipse(QRectF(knob_x - 1.8, y - 1.8, 3.6, 3.6))
            p.setBrush(Qt.BrushStyle.NoBrush)
    elif name == "about":
        p.drawEllipse(QRectF(4,4,14,14)); p.drawLine(11,10,11,16); p.drawPoint(11,7)
    else:
        p.drawRoundedRect(QRectF(4, 4, 14, 14), 3, 3)
    p.end()
    return QIcon(pm)
