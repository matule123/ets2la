"""Small original monochrome line icons for UltraPilot navigation."""

from PyQt6.QtCore import Qt, QRectF, QPointF
from PyQt6.QtGui import QIcon, QPixmap, QPainter, QPen, QPolygonF, QColor


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
        p.drawRoundedRect(QRectF(4, 4, 6, 6), 1.5, 1.5); p.drawRoundedRect(QRectF(12, 4, 6, 6), 1.5, 1.5)
        p.drawRoundedRect(QRectF(4, 12, 6, 6), 1.5, 1.5); p.drawRoundedRect(QRectF(12, 12, 6, 6), 1.5, 1.5)
    elif name == "navigation":
        p.drawPolygon(QPolygonF([QPointF(11,3), QPointF(18,18), QPointF(11,15), QPointF(4,18)]))
        p.drawLine(QPointF(11,15), QPointF(11,8))
    elif name == "visualization":
        p.drawRoundedRect(QRectF(3,4,16,12),2,2); p.drawLine(7,19,15,19); p.drawLine(11,16,11,19)
    elif name == "plugins":
        p.drawRoundedRect(QRectF(6,6,10,10),2,2)
        for a,b,c,d in ((9,3,9,6),(13,3,13,6),(9,16,9,19),(13,16,13,19),(3,9,6,9),(16,9,19,9),(3,13,6,13),(16,13,19,13)):
            p.drawLine(a,b,c,d)
    elif name == "settings":
        p.drawEllipse(QRectF(5,5,12,12)); p.drawEllipse(QRectF(9,9,4,4))
        for a,b,c,d in ((11,2,11,5),(11,17,11,20),(2,11,5,11),(17,11,20,11),(4,4,6,6),(16,16,18,18),(18,4,16,6),(4,18,6,16)):
            p.drawLine(a,b,c,d)
    else:
        p.drawEllipse(QRectF(4,4,14,14)); p.drawLine(11,10,11,16); p.drawPoint(11,7)
    p.end()
    return QIcon(pm)
