from PyQt6.QtWidgets import QApplication, QWidget
from PyQt6.QtGui import QPainter, QColor, QFont, QPen
from PyQt6.QtCore import Qt, QTimer, QRectF
import sys

# State -> accent colour for the HUD.
_STATE_COLORS = {
    "EMERGENCY": "#FF3B30",
    "AVOID_OBSTACLE": "#FF9500",
    "OVERTAKING": "#FFCC00",
    "PAY_TOLL": "#FFD60A",
    "FOLLOW_LANE": "#00FFCC",
    "CRUISE": "#34C759",
    "IDLE": "#8E8E93",
}


def _gear_text(gear: int) -> str:
    if gear is None:
        return "N"
    if gear > 0:
        return str(int(gear))
    if gear < 0:
        return f"R{abs(int(gear))}" if gear < -1 else "R"
    return "N"


class UltraPilotHUD(QWidget):
    """Transparent, always-on-top, custom-painted HUD overlay for ETS2."""

    W, H = 300, 150

    def __init__(self, shared_state):
        super().__init__()
        self.shared_state = shared_state
        self._blink_phase = True  # for flashing blinker arrows
        self.init_ui()

    def init_ui(self):
        self.setWindowFlags(
            Qt.WindowType.FramelessWindowHint |
            Qt.WindowType.WindowStaysOnTopHint |
            Qt.WindowType.Tool
        )
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground)
        self.resize(self.W, self.H)
        self.move_to_corner()

        self.timer = QTimer()
        self.timer.timeout.connect(self._tick)
        self.timer.start(100)  # 10 FPS

    def move_to_corner(self):
        screen = QApplication.primaryScreen().geometry()
        self.move(screen.width() - self.W - 20, 40)

    def _tick(self):
        self._blink_phase = not self._blink_phase
        self.update()

    # --- Data helpers ---------------------------------------------------------
    def _read(self):
        s = self.shared_state
        state = s.get("system_state", "IDLE")
        state = state.name if hasattr(state, "name") else str(state)

        speed = s.get("speed", 0) or 0
        try:
            speed = float(speed)
        except (TypeError, ValueError):
            speed = 0.0
        speed_kmh = speed * 3.6 if abs(speed) < 200 else speed

        truck = (s.get("telemetry", {}) or {}).get("truck", {}) or {}
        return {
            "state": state,
            "speed_kmh": abs(speed_kmh),
            "gear": truck.get("gear", 0),
            "rpm": truck.get("engineRpm", 0.0) or 0.0,
            "fuel": truck.get("fuel", 0.0) or 0.0,
            "limit_ms": truck.get("speedLimit", 0.0) or 0.0,
            "blinkerL": bool(truck.get("blinkerLeft", False)),
            "blinkerR": bool(truck.get("blinkerRight", False)),
            "active": bool(s.get("autopilot_active", False)),
            "nav_active": bool(s.get("nav_active", False)),
            "nav_dist": s.get("distance_to_dest"),
            "acc_speed": s.get("tags.acc.acc_speed"),
        }

    # --- Painting -------------------------------------------------------------
    def paintEvent(self, event):
        d = self._read()
        qp = QPainter(self)
        qp.setRenderHint(QPainter.RenderHint.Antialiasing)

        accent = QColor(_STATE_COLORS.get(d["state"], "#10B981"))
        TEXT = QColor("#111827")
        MUTED = QColor("#6B7280")

        # Clean white translucent card with a soft border.
        qp.setBrush(QColor(255, 255, 255, 235))
        qp.setPen(QPen(QColor(0, 0, 0, 25), 1))
        qp.drawRoundedRect(QRectF(1, 1, self.W - 2, self.H - 2), 14, 14)
        # Thin accent strip on the left edge.
        qp.setPen(Qt.PenStyle.NoPen)
        qp.setBrush(accent)
        qp.drawRoundedRect(QRectF(1, 1, 6, self.H - 2), 3, 3)

        # State (dot + name) top-left.
        qp.setBrush(accent); qp.setPen(Qt.PenStyle.NoPen)
        qp.drawEllipse(QRectF(20, 16, 9, 9))
        qp.setPen(TEXT)
        qp.setFont(QFont("Segoe UI Semibold", 11, QFont.Weight.DemiBold))
        qp.drawText(QRectF(36, 11, self.W - 130, 20), Qt.AlignmentFlag.AlignVCenter, d["state"])

        # Autopilot pill top-right.
        on = d["active"]
        pill = QRectF(self.W - 104, 12, 86, 20)
        qp.setBrush(QColor("#10B981") if on else QColor("#9CA3AF"))
        qp.setPen(Qt.PenStyle.NoPen)
        qp.drawRoundedRect(pill, 10, 10)
        qp.setPen(QColor("#FFFFFF"))
        qp.setFont(QFont("Segoe UI", 8, QFont.Weight.Bold))
        qp.drawText(pill, Qt.AlignmentFlag.AlignCenter, "AUTOPILOT" if on else "MANUAL")

        # Big speed.
        qp.setPen(TEXT)
        qp.setFont(QFont("Segoe UI", 44, QFont.Weight.Bold))
        qp.drawText(QRectF(18, 34, 160, 64), Qt.AlignmentFlag.AlignLeft, f"{d['speed_kmh']:.0f}")
        qp.setPen(MUTED)
        qp.setFont(QFont("Segoe UI", 10))
        qp.drawText(QRectF(20, 96, 160, 18), Qt.AlignmentFlag.AlignLeft, "km/h")

        # Gear badge (right).
        qp.setBrush(QColor("#F3F4F6")); qp.setPen(QPen(QColor("#E5E7EB"), 1))
        qp.drawRoundedRect(QRectF(self.W - 96, 44, 78, 52), 10, 10)
        qp.setPen(accent)
        qp.setFont(QFont("Segoe UI", 26, QFont.Weight.Bold))
        qp.drawText(QRectF(self.W - 96, 46, 78, 48), Qt.AlignmentFlag.AlignCenter,
                    _gear_text(d["gear"]))

        # Bottom info line: limit • nav (clean, muted).
        parts = []
        if d["limit_ms"] and d["limit_ms"] > 1:
            parts.append(f"Limit {d['limit_ms'] * 3.6:.0f}")
        if d["acc_speed"] is not None:
            try:
                parts.append(f"Set {float(d['acc_speed']):.0f}")
            except (TypeError, ValueError):
                pass
        if d["nav_active"] and d["nav_dist"] is not None:
            parts.append(f"Nav {float(d['nav_dist']) / 1000:.1f} km")
        parts.append(f"Fuel {d['fuel']:.0f} L")
        qp.setPen(MUTED)
        qp.setFont(QFont("Segoe UI", 9))
        qp.drawText(QRectF(20, self.H - 26, self.W - 36, 18),
                    Qt.AlignmentFlag.AlignLeft, "   •   ".join(parts))

    # --- Dragging -------------------------------------------------------------
    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.old_pos = event.globalPosition().toPoint()

    def mouseMoveEvent(self, event):
        if hasattr(self, "old_pos"):
            delta = event.globalPosition().toPoint() - self.old_pos
            self.move(self.x() + delta.x(), self.y() + delta.y())
            self.old_pos = event.globalPosition().toPoint()


def run_hud(shared_state):
    app = QApplication(sys.argv)
    hud = UltraPilotHUD(shared_state)
    hud.show()
    sys.exit(app.exec())
