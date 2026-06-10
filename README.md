# UltraPilot — Autopilot for Euro Truck Simulator 2

UltraPilot is a self-driving assistant for **Euro Truck Simulator 2** (and ATS), inspired by
[ETS2LA](https://github.com/ETS2LA/Euro-Truck-Simulator-2-Lane-Assist). It provides lane keeping,
adaptive cruise control, collision avoidance, coordinate-based route navigation, a voice assistant
and a transparent on-screen HUD.

## Architecture

UltraPilot runs as several cooperating OS processes, supervised by a bootloader, so a crash in any
single component can't take down the autopilot:

```
main.py → bootloader.py
            ├── Engine process   (core/engine.py)   — telemetry, perception, planner, control flush
            │     └── Plugin processes (plugins/*)  — autopilot, acc, collision, map, tts, discord …
            ├── UI process       (ui/app.py)        — control panel (PyQt6)
            └── HUD process      (core/hud.py)       — transparent always-on-top overlay
```

* **Shared state** (`core/ipc/shared_state.py`) — one `multiprocessing.Manager().dict()` shared by
  every process. All inter-process communication goes through it.
* **Control-intent pattern** — plugins never touch the input device. They write *intents*
  (`ctl_steering`, `ctl_throttle`, …) into shared state; only the Engine owns the real
  `Controller` (`core/controller.py`) and flushes intents to the device, gated by the
  `autopilot_active` master safety switch.
* **Control backends** (priority): SCS SDK DLL → virtual joystick (vgamepad) → digital keys.

## Requirements

* Windows 10/11, Python 3.10+
* Euro Truck Simulator 2 (or American Truck Simulator)
* The **SCS SDK telemetry plugin** DLL installed into the game's
  `bin/win_x64/plugins/` folder (the `.msi` installer does this automatically).

## Install

**From source:**

```bash
pip install -r requirements.txt
python main.py
```

**Windows installer (.msi):**

```bash
pip install cx_Freeze
python setup_msi.py bdist_msi
# → dist/UltraPilot-<version>-win64.msi
```

The MSI installs UltraPilot to `Program Files\UltraPilot`, creates Start-menu and desktop
shortcuts, and copies the SCS telemetry DLL into the game.

## Usage

1. Launch the game, then start UltraPilot.
2. In the control panel, toggle plugins and press **ENABLE AUTOPILOT** (master switch),
   or just press **N** on the keyboard (works from inside the game) to toggle it.
3. **Navigation:** on the *Map* page, press *Record* and drive a route once; press *Stop* to save it.
   Later, *Load* the route to have UltraPilot follow it using world coordinates from telemetry.

## Steering / control backend

UltraPilot drives the truck through, in priority order:

1. **SCS SDK plugin** (`scs_sdk_controller.dll` → `Local\SCSControls` shared memory) — the
   preferred path. It writes steering/throttle/brake straight into the game, so a real wheel
   (e.g. a **Logitech G29**) stays connected and the in-game wheel turns with the autopilot.
   No virtual controller is created. Force feedback is left to the game.
2. **Virtual Xbox controller** (vgamepad) — fallback; needs the **ViGEmBus** driver, which the
   installer sets up automatically (drop `ViGEmBus_Setup.exe` into `assets/` to bundle it).
3. **Keyboard** — last-resort digital fallback.

The `N` key toggles the autopilot at any time. Disabling it instantly releases all controls.

## Plugins

| Plugin     | Purpose                                                            |
|------------|-------------------------------------------------------------------|
| autopilot  | Fuses ACC + navigation + lane offset into final steering/throttle |
| acc        | Adaptive cruise control (speed + posted-limit obeying)            |
| collision  | Emergency braking + obstacle bypass requests                      |
| map        | Coordinate-based route navigation (record / replay)               |
| tts        | Voice announcements                                               |
| discord    | Discord Rich Presence (opt-in)                                    |
| ecodrive   | Throttle smoothing for fuel economy (opt-in)                      |

## License

Educational / entertainment use with Euro Truck Simulator 2.
