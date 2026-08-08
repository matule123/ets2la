"""
Theme system for UltraPilot: light / dark / system.

Call ``stylesheet(mode)`` to get the QSS for the whole app.  ``mode`` is one of
"light", "dark", "system" (system follows the OS dark-mode setting on Windows).
Interactive controls use a calm Codex-like blue while green remains reserved
for connected, safe and successful runtime states.
"""

ACCENT = "#2563EB"

_LIGHT = {
    "bg": "#F4F6F8", "surface": "#FFFFFF", "card": "#FFFFFF", "card2": "#F7F9FB",
    "text": "#1A1D21", "muted": "#6B7280", "border": "#E5E7EB",
    "sidebar": "#FFFFFF", "sidebar2": "#F7F9FB", "field": "#FFFFFF", "title": "#1D4ED8",
    "accent2": "#60A5FA", "success": "#16A34A", "warn": "#D97706", "danger": "#DC2626",
    "glass": "rgba(255,255,255,0.72)", "glass2": "rgba(255,255,255,0.55)",
    "hero_a": "#0F172A", "hero_b": "#1E3A8A", "glow": "rgba(37,99,235,0.18)",
}
_DARK = {
    "bg": "#0D1117", "surface": "#161B22", "card": "#161B22", "card2": "#21262D",
    "text": "#E6EDF3", "muted": "#8B949E", "border": "#30363D",
    "sidebar": "#010409", "sidebar2": "#161B22", "field": "#0D1117", "title": "#60A5FA",
    "accent2": "#60A5FA", "success": "#2EA043", "warn": "#D29922", "danger": "#F85149",
    "glass": "rgba(22,27,34,0.72)", "glass2": "rgba(22,27,34,0.55)",
    "hero_a": "#0D1117", "hero_b": "#172554", "glow": "rgba(37,99,235,0.28)",
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
    frame_border = "#AEB5BE" if c is _LIGHT else "#4A515C"
    nav_text = "#343941" if c is _LIGHT else "#C9D1D9"
    nav_hover = "#F5F8FF" if c is _LIGHT else "#111827"
    nav_hover_border = "#E2E8F0" if c is _LIGHT else "#253047"
    nav_active = "#EFF6FF" if c is _LIGHT else "#172554"
    nav_active_border = "#DBEAFE" if c is _LIGHT else "#1E3A8A"
    nav_active_text = "#1D4ED8" if c is _LIGHT else "#BFDBFE"
    sidebar_card = "#FAFAFB" if c is _LIGHT else "#0D1117"
    sidebar_card_border = "#E4E6E9" if c is _LIGHT else "#30363D"
    return f"""
QMainWindow {{ background-color: transparent; }}
QWidget {{ background-color: {c['bg']}; color: {c['text']};
    font-family: 'Segoe UI', 'Inter', sans-serif; font-size: 14px; }}
QFrame#WindowSurface {{ background-color: {c['bg']};
    border: 1px solid {frame_border}; border-radius: 15px; }}

/* Sidebar — compact reference-style navigation cards. */
QFrame#Sidebar {{ background-color: {c['sidebar']}; border: none;
    border-right: 1px solid {c['border']}; border-top-left-radius: 15px;
    border-bottom-left-radius: 15px; }}
QFrame#Sidebar QLabel#BrandSubtitle {{ color: {c['muted']}; font-size: 10px;
    font-weight: 600; border: none; }}
QFrame#Sidebar QLabel#NavSection {{ color: {c['muted']}; font-size: 11px;
    font-weight: 500; padding: 10px 8px 4px 8px; border: none; }}
QFrame#Sidebar QPushButton#NavButton {{ background-color: transparent;
    border: 1px solid transparent; border-radius: 9px; padding: 7px 10px;
    margin: 1px 0; text-align: left; color: {nav_text}; font-size: 13px;
    font-weight: 500; }}
QFrame#Sidebar QPushButton#NavButton:hover {{ background-color: {nav_hover};
    border-color: {nav_hover_border}; color: {c['text']}; }}
QFrame#Sidebar QPushButton#NavButton:checked {{ background-color: {nav_active};
    border-color: {nav_active_border}; color: {nav_active_text}; font-weight: 650; }}
QFrame#Sidebar QFrame#SidebarUpdateCard, QFrame#Sidebar QFrame#SidebarStatusCard {{
    background-color: {sidebar_card}; border: 1px solid {sidebar_card_border};
    border-radius: 10px; }}
QFrame#Sidebar QLabel#SidebarConnection {{ color: {c['muted']}; font-size: 11px;
    font-weight: 650; border: none; }}
QFrame#Sidebar QLabel#SidebarConnection[connectionState="connected"] {{
    color: #0E9F6E; }}
QFrame#Sidebar QLabel#SidebarConnection[connectionState="autopilot"] {{
    color: #057A55; font-weight: 700; }}
QFrame#Sidebar QPushButton#SidebarPerformance {{ background-color: {c['surface']};
    border: 1px solid {c['border']}; border-radius: 8px; padding: 5px 9px;
    margin: 0; color: {c['muted']}; font-size: 11px; font-weight: 650;
    text-align: left; }}
QFrame#Sidebar QPushButton#SidebarPerformance:hover {{ background-color:{nav_hover};
    border-color:{nav_active_border}; color:{nav_active_text}; }}
QFrame#Sidebar QPushButton#SidebarPerformance[active="true"] {{
    background-color:#20242A; border-color:#20242A; color:#FFFFFF; }}
QFrame#Sidebar QPushButton#SidebarAutopilot {{ background-color:#0E9F6E;
    background:qlineargradient(x1:0,y1:0,x2:1,y2:0,stop:0 #059669,stop:1 #10B981);
    border:1px solid #047857; border-radius:12px; padding:9px 12px;
    margin:0; color:#FFFFFF; font-size:12px; font-weight:700;
    text-align:center; }}
QFrame#Sidebar QPushButton#SidebarAutopilot:hover {{ background:#047857;
    border-color:#046C4E; color:#FFFFFF; }}
QFrame#Sidebar QPushButton#SidebarAutopilot[active="true"] {{
    background:qlineargradient(x1:0,y1:0,x2:1,y2:0,stop:0 #DC2626,stop:1 #F05252);
    border-color:#B91C1C; color:#FFFFFF; }}

/* General buttons — soft, rounded, accent on hover. */
QPushButton {{ background-color: {c['surface']}; border: 1px solid {c['border']};
    border-radius: 10px; padding: 9px 16px; color: {c['text']}; font-weight: 600; }}
QPushButton:hover {{ border-color: {ACCENT}; color: {ACCENT}; }}
QPushButton:pressed {{ background-color: {ACCENT}; color: #FFFFFF; border-color: {ACCENT}; }}
QPushButton:focus {{ border: 1px solid {ACCENT}; }}

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
QComboBox:hover, QComboBox:focus, QLineEdit:focus {{ border-color: {ACCENT}; }}
QComboBox::drop-down {{ border: none; width: 22px; }}
QComboBox QAbstractItemView {{ background-color: {c['surface']}; color: {c['text']};
    border: 1px solid {c['border']}; border-radius: 8px; outline: none;
    selection-background-color: {ACCENT}; selection-color: #FFFFFF; }}

QCheckBox {{ spacing: 9px; color: {c['text']}; }}
QCheckBox::indicator {{ width: 18px; height: 18px; border: 1px solid {c['border']};
    border-radius: 5px; background: {c['field']}; }}
QCheckBox::indicator:checked {{ background: {ACCENT}; border-color: {ACCENT}; }}

QSlider {{ min-height: 26px; }}
QSlider::groove:horizontal {{ height: 6px; background: {c['border']}; border-radius: 3px; }}
QSlider::sub-page:horizontal {{ background: {ACCENT}; border-radius: 3px; }}
QSlider::handle:horizontal {{ background: #FFFFFF; border: 2px solid {ACCENT};
    width: 16px; height: 16px; margin: -6px 0; border-radius: 9px; }}

/* Progress bar — gradient chunk for a richer look. */
QProgressBar {{ background-color: {c['field']}; border: none;
    border-radius: 8px; height: 18px; text-align: center; color: {c['text']}; font-weight: 600; }}
QProgressBar::chunk {{ background-color: {accent2};
    background: qlineargradient(x1:0,y1:0,x2:1,y2:0,
        stop:0 {ACCENT}, stop:1 {accent2});
    border-radius: 8px; }}

QToolTip {{ background-color: {c['surface']}; color: {c['text']};
    border: 1px solid {c['border']}; border-radius: 8px; padding: 7px 9px; }}

QScrollBar:vertical {{ background: transparent; width: 10px; margin: 2px; }}
QScrollBar::handle:vertical {{ background: {c['border']}; border-radius: 5px; min-height: 30px; }}
QScrollBar::add-line, QScrollBar::sub-line {{ height: 0; }}

/* QScrollArea must not paint a mismatched background (white-on-dark bug). */
QScrollArea {{ background: transparent; border: none; }}
QScrollArea > QWidget > QWidget {{ background: transparent; }}
QTextEdit {{ background-color: {c['field']}; color: {c['text']};
    border: 1px solid {c['border']}; border-radius: 10px; }}
"""
