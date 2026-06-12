"""
Theme system for UltraPilot: light / dark / system, ETS2LA-style.

Call ``stylesheet(mode)`` to get the QSS for the whole app.  ``mode`` is one of
"light", "dark", "system" (system follows the OS dark-mode setting on Windows).
Colours match the clean ETS2LA look (white or near-black surfaces, green accent).
"""

ACCENT = "#10B981"

_LIGHT = {
    "bg": "#F4F6F8", "surface": "#FFFFFF", "text": "#1A1D21", "muted": "#6B7280",
    "border": "#E5E7EB", "sidebar": "#FFFFFF", "field": "#FFFFFF",
    "title": "#065F46",
}
_DARK = {
    "bg": "#16181D", "surface": "#1E2228", "text": "#E6E8EB", "muted": "#9AA0A6",
    "border": "#2C313A", "sidebar": "#1A1D22", "field": "#23272E",
    "title": "#34D399",
}


def is_system_dark() -> bool:
    try:
        import winreg
        k = winreg.OpenKey(
            winreg.HKEY_CURRENT_USER,
            r"Software\Microsoft\Windows\CurrentVersion\Themes\Personalize")
        val, _ = winreg.QueryValueEx(k, "AppsUseLightTheme")
        return val == 0
    except Exception:
        return False


def palette(mode: str) -> dict:
    if mode == "system":
        return _DARK if is_system_dark() else _LIGHT
    return _DARK if mode == "dark" else _LIGHT


def stylesheet(mode: str = "light") -> str:
    c = palette(mode)
    return f"""
QMainWindow {{ background-color: {c['bg']}; }}
QWidget {{ background-color: {c['bg']}; color: {c['text']}; font-family: 'Segoe UI', sans-serif; }}
QPushButton {{ background-color: {c['surface']}; border: 1px solid {c['border']};
    border-radius: 8px; padding: 10px; color: {c['text']}; }}
QPushButton:hover {{ border-color: {ACCENT}; color: {ACCENT}; }}
QPushButton:pressed {{ background-color: {ACCENT}; color: #FFFFFF; }}
QLabel {{ color: {c['text']}; }}
QFrame#Sidebar {{ background-color: {c['sidebar']}; border-right: 1px solid {c['border']}; }}
QComboBox, QLineEdit {{ background-color: {c['field']}; border: 1px solid {c['border']};
    border-radius: 8px; padding: 7px; color: {c['text']}; }}
QComboBox QAbstractItemView {{ background-color: {c['field']}; color: {c['text']};
    selection-background-color: {ACCENT}; }}
QCheckBox {{ spacing: 8px; }}
QSlider::groove:horizontal {{ height: 6px; background: {c['border']}; border-radius: 3px; }}
QSlider::handle:horizontal {{ background: {ACCENT}; width: 16px; margin: -6px 0; border-radius: 8px; }}
QStatusBar {{ background-color: {c['surface']}; border-top: 1px solid {c['border']}; }}
QProgressBar {{ background-color: {c['field']}; border: 1px solid {c['border']};
    border-radius: 6px; height: 18px; text-align: center; color: {c['text']}; }}
QProgressBar::chunk {{ background-color: {ACCENT}; border-radius: 5px; }}
"""
