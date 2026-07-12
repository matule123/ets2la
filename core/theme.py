"""
Theme system for UltraPilot: light / dark / system, ETS2LA-style.

Call ``stylesheet(mode)`` to get the QSS for the whole app.  ``mode`` is one of
"light", "dark", "system" (system follows the OS dark-mode setting on Windows).
Colours match the clean ETS2LA look (white or near-black surfaces, green accent).
"""

ACCENT = "#10B981"

_LIGHT = {
    "bg": "#F4F6F8", "surface": "#FFFFFF", "card": "#FFFFFF", "card2": "#F7F9FB",
    "text": "#1A1D21", "muted": "#6B7280", "border": "#E5E7EB",
    "sidebar": "#FFFFFF", "sidebar2": "#F7F9FB", "field": "#FFFFFF", "title": "#065F46",
    "accent2": "#34D399", "success": "#16A34A", "warn": "#D97706", "danger": "#DC2626",
    "glass": "rgba(255,255,255,0.72)", "glass2": "rgba(255,255,255,0.55)",
    "hero_a": "#0F172A", "hero_b": "#065F46", "glow": "rgba(46,160,67,0.18)",
}
_DARK = {
    "bg": "#0D1117", "surface": "#161B22", "card": "#161B22", "card2": "#21262D",
    "text": "#E6EDF3", "muted": "#8B949E", "border": "#30363D",
    "sidebar": "#010409", "sidebar2": "#161B22", "field": "#0D1117", "title": "#2EA043",
    "accent2": "#2EA043", "success": "#2EA043", "warn": "#D29922", "danger": "#F85149",
    "glass": "rgba(22,27,34,0.72)", "glass2": "rgba(22,27,34,0.55)",
    "hero_a": "#0D1117", "hero_b": "#0E2A1A", "glow": "rgba(46,160,67,0.28)",
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
    accent2 = c['accent2']
    return f"""
QMainWindow {{ background-color: {c['bg']}; }}
QWidget {{ background-color: {c['bg']}; color: {c['text']};
    font-family: 'Segoe UI', 'Inter', sans-serif; font-size: 14px; }}

/* Sidebar — ETS2LA deep panel with a faint accent glow at the top. */
QFrame#Sidebar {{ background-color: {c['sidebar']}; border: none;
    border-right: 1px solid {c['border']}; }}
QFrame#Sidebar QPushButton {{ background-color: transparent; border: none;
    border-radius: 10px; padding: 11px 14px; margin: 2px 8px; text-align: left;
    color: {c['muted']}; font-weight: 600; }}
QFrame#Sidebar QPushButton:hover {{ background-color: {c['field']}; color: {c['text']}; }}
QFrame#Sidebar QPushButton:checked {{ background-color: {ACCENT}; color: #FFFFFF; }}

/* General buttons — soft, rounded, accent on hover. */
QPushButton {{ background-color: {c['surface']}; border: 1px solid {c['border']};
    border-radius: 10px; padding: 9px 16px; color: {c['text']}; font-weight: 600; }}
QPushButton:hover {{ border-color: {ACCENT}; color: {ACCENT}; }}
QPushButton:pressed {{ background-color: {ACCENT}; color: #FFFFFF; border-color: {ACCENT}; }}

QLabel {{ color: {c['text']}; background: transparent; }}
QFrame {{ border-radius: 14px; }}

/* Cards / surfaces — ETS2LA elevation: solid card with a crisp border. */
QFrame#Card, QFrame#Panel {{ background-color: {c['card']};
    border: 1px solid {c['border']}; border-radius: 14px; }}
QFrame#ApCard {{ background-color: {c['card']};
    border: 1px solid {c['border']}; border-radius: 16px; }}
/* Hero card — subtle diagonal gradient for the dashboard's eye-catcher. */
QFrame#Hero {{ background: qlineargradient(x1:0,y1:0,x2:1,y2:1,
    stop:0 {c['hero_a']}, stop:1 {c['hero_b']});
    border: 1px solid {c['border']}; border-radius: 16px; }}
/* Glass island — frosted translucent panel (ETA / floating overlays). */
QFrame#Glass {{ background-color: {c['glass']};
    border: 1px solid {c['border']}; border-radius: 14px; }}

QComboBox, QLineEdit {{ background-color: {c['field']}; border: 1px solid {c['border']};
    border-radius: 10px; padding: 8px 10px; color: {c['text']}; }}
QComboBox:hover, QLineEdit:focus {{ border-color: {ACCENT}; }}
QComboBox::drop-down {{ border: none; width: 22px; }}
QComboBox QAbstractItemView {{ background-color: {c['surface']}; color: {c['text']};
    border: 1px solid {c['border']}; border-radius: 8px; outline: none;
    selection-background-color: {ACCENT}; selection-color: #FFFFFF; }}

QCheckBox {{ spacing: 9px; color: {c['text']}; }}
QCheckBox::indicator {{ width: 18px; height: 18px; border: 1px solid {c['border']};
    border-radius: 5px; background: {c['field']}; }}
QCheckBox::indicator:checked {{ background: {ACCENT}; border-color: {ACCENT}; }}

QSlider::groove:horizontal {{ height: 6px; background: {c['border']}; border-radius: 3px; }}
QSlider::sub-page:horizontal {{ background: {ACCENT}; border-radius: 3px; }}
QSlider::handle:horizontal {{ background: #FFFFFF; border: 2px solid {ACCENT};
    width: 16px; height: 16px; margin: -7px 0; border-radius: 9px; }}

QStatusBar {{ background-color: {c['sidebar']}; border-top: 1px solid {c['border']}; }}

/* Progress bar — gradient chunk for a richer look. */
QProgressBar {{ background-color: {c['field']}; border: none;
    border-radius: 8px; height: 18px; text-align: center; color: {c['text']}; font-weight: 600; }}
QProgressBar::chunk {{ background-color: {accent2};
    background: qlineargradient(x1:0,y1:0,x2:1,y2:0,
        stop:0 {ACCENT}, stop:1 {accent2});
    border-radius: 8px; }}

QScrollBar:vertical {{ background: transparent; width: 10px; margin: 2px; }}
QScrollBar::handle:vertical {{ background: {c['border']}; border-radius: 5px; min-height: 30px; }}
QScrollBar::add-line, QScrollBar::sub-line {{ height: 0; }}

/* QScrollArea must not paint a mismatched background (white-on-dark bug). */
QScrollArea {{ background: transparent; border: none; }}
QScrollArea > QWidget > QWidget {{ background: transparent; }}
QTextEdit {{ background-color: {c['field']}; color: {c['text']};
    border: 1px solid {c['border']}; border-radius: 10px; }}
"""
