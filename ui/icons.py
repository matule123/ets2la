"""Small original monochrome line icons for UltraPilot navigation."""

from PyQt6.QtCore import Qt, QRectF, QPointF
from PyQt6.QtGui import (QIcon, QPixmap, QPainter, QPen, QPolygonF, QColor,
                         QPainterPath)


def line_icon(name: str, color="#4B5563", size=22) -> QIcon:
    pm = QPixmap(size, size)
    pm.fill(Qt.GlobalColor.transparent)
    p = QPainter(pm)
    p.setRenderHint(QPainter.RenderHint.Antialiasing)
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
    elif name == "settings":
        p.drawEllipse(QRectF(5,5,12,12)); p.drawEllipse(QRectF(9,9,4,4))
        for a,b,c,d in ((11,2,11,5),(11,17,11,20),(2,11,5,11),(17,11,20,11),(4,4,6,6),(16,16,18,18),(18,4,16,6),(4,18,6,16)):
            p.drawLine(a,b,c,d)
    elif name == "about":
        p.drawEllipse(QRectF(4,4,14,14)); p.drawLine(11,10,11,16); p.drawPoint(11,7)
    else:
        p.drawRoundedRect(QRectF(4, 4, 14, 14), 3, 3)
    p.end()
    return QIcon(pm)
