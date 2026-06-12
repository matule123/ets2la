"""
Lightweight translation table for the UltraPilot UI.

Each language maps UI keys to strings.  English is the reference (100%).  Other
languages may be partial; ``coverage(lang)`` reports how complete they are so the
Settings page can show "Slovenčina — 100% translated".
"""

# Reference language defines the full set of keys.
_EN = {
    "dashboard": "Dashboard", "navigation": "Navigation", "plugins": "Plugins",
    "settings": "Settings", "about": "About",
    "current_speed": "CURRENT SPEED", "system": "SYSTEM",
    "enable_ap": "ENABLE AUTOPILOT", "disable_ap": "DISABLE AUTOPILOT",
    "telemetry_connected": "Telemetry connected",
    "waiting_telemetry": "Waiting for game telemetry…",
    "appearance": "Appearance", "theme": "Theme", "language": "Language",
    "theme_light": "Light", "theme_dark": "Dark", "theme_system": "System",
    "acc": "Adaptive Cruise Control", "target_speed": "Target Speed",
    "safe_distance": "Safe Distance", "obey_limit": "Obey posted speed limit",
    "steering": "Steering", "invert_steering": "Invert steering",
    "sensitivity": "Sensitivity", "translated": "translated",
    "open_ets2la": "Open ETS2LA web app",
}

# Partial translations are fine — missing keys fall back to English.
_SK = {
    "dashboard": "Prehľad", "navigation": "Navigácia", "plugins": "Pluginy",
    "settings": "Nastavenia", "about": "O aplikácii",
    "current_speed": "AKTUÁLNA RÝCHLOSŤ", "system": "SYSTÉM",
    "enable_ap": "ZAPNÚŤ AUTOPILOTA", "disable_ap": "VYPNÚŤ AUTOPILOTA",
    "telemetry_connected": "Telemetria pripojená",
    "waiting_telemetry": "Čakám na telemetriu z hry…",
    "appearance": "Vzhľad", "theme": "Téma", "language": "Jazyk",
    "theme_light": "Svetlá", "theme_dark": "Tmavá", "theme_system": "Systémová",
    "acc": "Adaptívny tempomat", "target_speed": "Cieľová rýchlosť",
    "safe_distance": "Bezpečná vzdialenosť", "obey_limit": "Dodržiavať rýchlostný limit",
    "steering": "Riadenie", "invert_steering": "Otočiť zatáčanie",
    "sensitivity": "Citlivosť", "translated": "preložené",
    "open_ets2la": "Otvoriť web ETS2LA",
}

_CS = {
    "dashboard": "Přehled", "navigation": "Navigace", "plugins": "Pluginy",
    "settings": "Nastavení", "about": "O aplikaci",
    "current_speed": "AKTUÁLNÍ RYCHLOST", "system": "SYSTÉM",
    "enable_ap": "ZAPNOUT AUTOPILOTA", "disable_ap": "VYPNOUT AUTOPILOTA",
    "appearance": "Vzhled", "theme": "Motiv", "language": "Jazyk",
    "theme_light": "Světlý", "theme_dark": "Tmavý", "theme_system": "Systémový",
    "steering": "Řízení", "sensitivity": "Citlivost", "translated": "přeloženo",
}

_DE = {
    "dashboard": "Übersicht", "navigation": "Navigation", "plugins": "Plugins",
    "settings": "Einstellungen", "about": "Über",
    "enable_ap": "AUTOPILOT EIN", "disable_ap": "AUTOPILOT AUS",
    "appearance": "Darstellung", "theme": "Design", "language": "Sprache",
    "theme_light": "Hell", "theme_dark": "Dunkel", "theme_system": "System",
    "steering": "Lenkung", "translated": "übersetzt",
}

_PL = {
    "dashboard": "Pulpit", "navigation": "Nawigacja", "plugins": "Wtyczki",
    "settings": "Ustawienia", "about": "O programie",
    "enable_ap": "WŁĄCZ AUTOPILOTA", "disable_ap": "WYŁĄCZ AUTOPILOTA",
    "appearance": "Wygląd", "theme": "Motyw", "language": "Język",
    "theme_light": "Jasny", "theme_dark": "Ciemny", "theme_system": "Systemowy",
    "translated": "przetłumaczono",
}

LANGUAGES = {
    "English": _EN, "Slovenčina": _SK, "Čeština": _CS,
    "Deutsch": _DE, "Polski": _PL,
}


def coverage(lang: str) -> int:
    """Percent of reference keys present in ``lang``."""
    tbl = LANGUAGES.get(lang, {})
    if not _EN:
        return 100
    return round(100 * sum(1 for k in _EN if k in tbl) / len(_EN))


def t(lang: str, key: str) -> str:
    """Translate ``key`` for ``lang``, falling back to English then the key."""
    return LANGUAGES.get(lang, {}).get(key) or _EN.get(key, key)
