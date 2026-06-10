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

        accent = QColor(_STATE_COLORS.get(d["state"], "#00FFCC"))

        # Panel background.
        qp.setBrush(QColor(10, 10, 12, 200))
        qp.setPen(QPen(accent, 2))
        qp.drawRoundedRect(QRectF(1, 1, self.W - 2, self.H - 2), 10, 10)

        # State title.
        qp.setPen(accent)
        qp.setFont(QFont("Segoe UI", 11, QFont.Weight.Bold))
        qp.drawText(QRectF(14, 8, self.W - 28, 20), Qt.AlignmentFlag.AlignLeft,
                    f"● {d['state']}")

        # Autopilot status (top right).
        ap_color = QColor("#34C759") if d["active"] else QColor("#FF453A")
        qp.setPen(ap_color)
        qp.setFont(QFont("Segoe UI", 9, QFont.Weight.Bold))
        qp.drawText(QRectF(14, 8, self.W - 28, 20), Qt.AlignmentFlag.AlignRight,
                    "AUTOPILOT ON" if d["active"] else "AUTOPILOT OFF")

        # Big speed readout.
        qp.setPen(QColor("#FFFFFF"))
        qp.setFont(QFont("Segoe UI", 38, QFont.Weight.Bold))
        qp.drawText(QRectF(10, 28, 150, 56), Qt.AlignmentFlag.AlignLeft,
                    f"{d['speed_kmh']:.0f}")
        qp.setPen(QColor("#8E8E93"))
        qp.setFont(QFont("Segoe UI", 9))
        qp.drawText(QRectF(12, 80, 150, 16), Qt.AlignmentFlag.AlignLeft, "km/h")

        # Gear box (right of speed).
        qp.setPen(QPen(QColor("#444"), 1))
        qp.setBrush(QColor(30, 30, 34, 220))
        qp.drawRoundedRect(QRectF(150, 36, 50, 44), 6, 6)
        qp.setPen(QColor("#FFD60A"))
        qp.setFont(QFont("Segoe UI", 22, QFont.Weight.Bold))
        qp.drawText(QRectF(150, 38, 50, 40), Qt.AlignmentFlag.AlignCenter,
                    _gear_text(d["gear"]))

        # Blinkers (flashing arrows, top-area right side).
        if d["blinkerL"] and self._blink_phase:
            qp.setPen(QColor("#34C759"))
            qp.setFont(QFont("Segoe UI", 16, QFont.Weight.Bold))
            qp.drawText(QRectF(210, 36, 30, 44), Qt.AlignmentFlag.AlignCenter, "◀")
        if d["blinkerR"] and self._blink_phase:
            qp.setPen(QColor("#34C759"))
            qp.setFont(QFont("Segoe UI", 16, QFont.Weight.Bold))
            qp.drawText(QRectF(255, 36, 30, 44), Qt.AlignmentFlag.AlignCenter, "▶")

        # RPM bar.
        rpm_frac = max(0.0, min(1.0, d["rpm"] / 2500.0))
        bar = QRectF(14, 96, self.W - 28, 8)
        qp.setPen(Qt.PenStyle.NoPen)
        qp.setBrush(QColor(40, 40, 44))
        qp.drawRoundedRect(bar, 4, 4)
        rpm_color = QColor("#FF453A") if rpm_frac > 0.85 else accent
        qp.setBrush(rpm_color)
        qp.drawRoundedRect(QRectF(14, 96, (self.W - 28) * rpm_frac, 8), 4, 4)

        # Info line (fuel | limit | target | nav).
        parts = [f"⛽ {d['fuel']:.0f}L"]
        if d["limit_ms"] and d["limit_ms"] > 1:
            parts.append(f"LIM {d['limit_ms'] * 3.6:.0f}")
        if d["acc_speed"] is not None:
            try:
                parts.append(f"SET {float(d['acc_speed']):.0f}")
            except (TypeError, ValueError):
                pass
        if d["nav_active"] and d["nav_dist"] is not None:
            parts.append(f"🧭 {float(d['nav_dist']) / 1000:.1f}km")
        qp.setPen(QColor("#C8C8C8"))
        qp.setFont(QFont("Consolas", 9))
        qp.drawText(QRectF(14, 112, self.W - 28, 20), Qt.AlignmentFlag.AlignLeft,
                    "   ".join(parts))

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
