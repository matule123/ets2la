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
        # House — the reference dashboard icon.
        roof = QPainterPath()
        roof.moveTo(3.5, 10.5); roof.lineTo(11.0, 4.0)
        roof.lineTo(18.5, 10.5)
        p.drawPath(roof)
        p.drawLine(QPointF(5.5, 9.0), QPointF(5.5, 18.0))
        p.drawLine(QPointF(16.5, 9.0), QPointF(16.5, 18.0))
        p.drawLine(QPointF(5.5, 18.0), QPointF(16.5, 18.0))
        p.drawLine(QPointF(9.0, 18.0), QPointF(9.0, 13.5))
        p.drawLine(QPointF(9.0, 13.5), QPointF(13.0, 13.5))
        p.drawLine(QPointF(13.0, 13.5), QPointF(13.0, 18.0))
    elif name == "navigation":
        p.drawPolygon(QPolygonF([QPointF(11,3), QPointF(18,18), QPointF(11,15), QPointF(4,18)]))
        p.drawLine(QPointF(11,15), QPointF(11,8))
    elif name == "visualization":
        # TvMinimal — thin monitor outline used by ETS2LA Visualization.
        p.drawRoundedRect(QRectF(3.0, 4.0, 16.0, 12.0), 2.0, 2.0)
        p.drawLine(QPointF(8.0, 19.0), QPointF(14.0, 19.0))
    elif name == "plugins":
        # ChartNoAxesGantt — plugin manager in the reference sidebar.
        p.drawLine(QPointF(4.0, 5.5), QPointF(15.0, 5.5))
        p.drawLine(QPointF(7.0, 10.8), QPointF(18.0, 10.8))
        p.drawLine(QPointF(4.0, 16.2), QPointF(13.5, 16.2))
        p.drawLine(QPointF(4.0, 4.0), QPointF(4.0, 7.0))
        p.drawLine(QPointF(18.0, 9.3), QPointF(18.0, 12.3))
        p.drawLine(QPointF(13.5, 14.7), QPointF(13.5, 17.7))
    elif name == "performance":
        # ChartArea — matching ETS2LA's performance entry.
        p.drawLine(QPointF(3.5, 4.0), QPointF(3.5, 18.0))
        p.drawLine(QPointF(3.5, 18.0), QPointF(19.0, 18.0))
        graph = QPainterPath()
        graph.moveTo(4.0, 14.5); graph.lineTo(8.0, 10.5)
        graph.lineTo(11.0, 13.0); graph.lineTo(15.0, 7.0)
        graph.lineTo(18.5, 9.5)
        p.drawPath(graph)
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
        # Bolt — the settings symbol used by the ETS2LA sidebar.
        bolt = QPainterPath()
        bolt.moveTo(12.5, 2.8); bolt.lineTo(5.5, 12.0)
        bolt.lineTo(10.3, 12.0); bolt.lineTo(9.4, 19.2)
        bolt.lineTo(16.7, 9.7); bolt.lineTo(11.8, 9.7)
        bolt.closeSubpath()
        p.drawPath(bolt)
    elif name == "about":
        # BookText — consistent with ETS2LA's help/about group.
        p.drawRoundedRect(QRectF(4.0, 3.5, 14.0, 15.0), 1.8, 1.8)
        p.drawLine(QPointF(7.0, 7.5), QPointF(15.0, 7.5))
        p.drawLine(QPointF(7.0, 11.0), QPointF(14.0, 11.0))
        p.drawLine(QPointF(7.0, 14.5), QPointF(11.5, 14.5))
    else:
        p.drawRoundedRect(QRectF(4, 4, 14, 14), 3, 3)
    p.end()
    return QIcon(pm)
