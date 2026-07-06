# UltraPilot

**Autopilot for Euro Truck Simulator 2 (and ATS)** — lane keeping, adaptive
cruise control, collision avoidance, map-based navigation, a transparent HUD
overlay and voice announcements. Inspired by [ETS2LA](https://github.com/ETS2LA).

![version](https://img.shields.io/badge/version-0.4.0-10B981)
![python](https://img.shields.io/badge/python-3.10%2B-blue)
![platform](https://img.shields.io/badge/platform-Windows%2010%2F11-lightgrey)
![license](https://img.shields.io/badge/license-Educational%2FEntertainment-9AA4B2)

---

## ✨ Features

| | |
|---|---|
| 🛣️ **Lane keeping** | OpenCV perception keeps the truck centred in its lane. |
| 🎯 **Adaptive cruise control** | Holds a target speed and brakes for slower traffic ahead. |
| 🚦 **Traffic & obstacles** | Reacts to stops, signals and obstacles on the road. |
| 🗺️ **Map navigation** | Drive the ETS2/ATS world by coordinates — supports **ProMods 2.82** (ETS2 1.59), vanilla ETS2/ATS, TruckersMP and more. |
| 🔄 **Auto turn signals** | Turn signals + blind-spot checks. |
| 🖥️ **HUD overlay** | Transparent, always-on-top 3D HUD drawn over the game (chase-cam scene, traffic, traffic-light, ETA). |
| 🛰️ **AR route overlay** | Draws the planned route directly on the road (experimental). |
| 🔊 **Voice announcements** | Spoken speed, fuel and event updates. |
| 📊 **Performance monitor** | Live RAM usage per plugin, in a floating mini-panel. |
| 🌍 **Multi-language** | Slovak & English bundled; Czech / German / Polish / French / Spanish downloadable from the repository. |
| ⚡ **Auto-update** | Checks GitHub for new releases and applies them in-app. |
| 🧭 **First-run onboarding** | A setup wizard picks the language, installs the game SDK and downloads the matching map pack. |

---

## 🏗️ Architecture

UltraPilot runs as several cooperating OS processes, supervised by a
bootloader, so a crash in any single component can't take down the autopilot:

```
main.py → bootloader.py
            ├── Engine process   (core/engine.py)   — telemetry, perception, planner, control flush
            │     └── Plugin processes (plugins/*)   — autopilot, acc, collision, map, tts, …
            ├── UI process       (ui/app.py)        — control panel (PyQt6)
            ├── HUD process      (core/hud.py)       — transparent always-on-top overlay
            └── AR process       (core/ar_overlay)   — click-through route overlay over the game
```

* **Shared state** (`core/ipc/shared_state.py`) — one `multiprocessing.Manager().dict()` shared by
  every process. All inter-process communication goes through it.
* **Control-intent pattern** — plugins never touch the input device. They write *intents*
  (`ctl_steering`, `ctl_throttle`, …) into shared state; only the Engine owns the real
  `Controller` (`core/controller.py`) and flushes intents to the device, gated by the
  `autopilot_active` master safety switch.
* **Control backends** (priority): SCS SDK DLL → virtual joystick (vgamepad) → digital keys.

---

## 📥 Install

### Windows installer (recommended)

1. Download **`UltraPilot_Installer.exe`** from the [latest release](https://github.com/matule123/ets2la/releases).
2. Run it. The installer will:
   - download the latest sources from GitHub,
   - make sure a usable **Python ≥ 3.10** is installed (auto-installs from python.org if missing),
   - install the Python dependencies,
   - copy the **SCS SDK plugin DLLs** into the game,
   - set up the **ViGEmBus** driver (virtual-joystick fallback),
   - create Desktop / Start-menu shortcuts.
3. Launch **UltraPilot**. On first run, the **onboarding wizard** guides you through:
   - choosing a **language**,
   - installing the **SDK** into your detected game(s),
   - downloading a **map pack** that matches your game version / mods.

### From source

```bash
git clone https://github.com/matule123/ets2la.git
cd ets2la
pip install -r requirements.txt
python main.py
```

---

## 🚀 Usage

1. Launch the game, then start UltraPilot.
2. In the control panel, configure plugins and press **ENABLE AUTOPILOT**
   (the master switch in the dashboard).
3. **Navigation:** the onboarding wizard downloads the map pack that matches
   your game. You can also *Record* a route on the **Map** page and replay it later.

---

## 🧩 Plugins

| Plugin | Purpose |
|---|---|
| `autopilot` | Fuses ACC + navigation + lane offset into final steering/throttle |
| `acc` | Adaptive cruise control (speed + posted-limit obeying) |
| `collision` | Emergency braking + obstacle bypass requests |
| `map` | Coordinate / map-based navigation (record / replay) |
| `drivepolicy` | Driving strategy (planned speed, lane offset) |
| `lanecontrol` | Overtaking, merge, hazard handling |
| `turnsignals` | Automatic turn signals + blind-spot checks |
| `toll` | Toll booth payment |
| `tts` | Voice announcements |
| `hud` | On-screen HUD data (off by default; the on-screen HUD is `core/hud.py`) |
| `ecodrive` | Throttle smoothing for fuel economy (opt-in) |
| `discord` | Discord Rich Presence (opt-in) |

---

## 🔧 Build the installer

The installer is a single self-contained `.exe` — it downloads the application
from GitHub at install time, so the build output is just one file:

```bash
pip install pyinstaller
python build_installer.py
# → dist/UltraPilot_Installer.exe
```

The bundle includes only `assets/` (SDK DLLs, icon, logo) and `languages/`
(Slovak + English); everything else is fetched from the repository on install.

---

## 🎛️ Steering / control backend

UltraPilot drives the truck through, in priority order:

1. **SCS SDK plugin** (`scs_sdk_controller.dll` → `Local\SCSControls` shared memory) — the
   preferred path. Writes steering/throttle/brake straight into the game, so a real wheel
   (e.g. Logitech G29) stays connected and the in-game wheel turns with the autopilot.
2. **Virtual Xbox controller** (vgamepad) — fallback; needs the **ViGEmBus** driver.
3. **Keyboard** — last-resort digital fallback.

Disabling the autopilot instantly releases all controls.

---

## 🌍 Languages

Translation files live in [`languages/`](./languages) as JSON (`sk.json`,
`en.json`, …). Slovak and English ship with the app; others can be downloaded
from the in-app **Settings → Appearance** page or during onboarding. Missing
keys fall back to English; each language shows its translation coverage %.

---

## 🔗 Links

* **Repository:** [matule123/ets2la](https://github.com/matule123/ets2la)
* **Issues:** [report a bug](https://github.com/matule123/ets2la/issues)
* **Inspiration:** [ETS2LA](https://github.com/ETS2LA/Euro-Truck-Simulator-2-Lane-Assist)

## License

Educational / entertainment use with Euro Truck Simulator 2. Provided „as is“,
without warranty. SCS SDK plugins and ViGEmBus are subject to their own licenses.
