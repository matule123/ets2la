"""
Unified Plugin SDK for ETS2-UltraPilot.

This object is constructed *inside* every plugin process from the shared
``multiprocessing.Manager().dict()`` proxy.  It gives every plugin a single,
consistent API regardless of which style it was written in:

    self.sdk.get(key) / self.sdk.set(key, value)        # raw shared state
    self.sdk.shared_state.get(...) / .set(...)          # same, namespaced
    self.sdk.telemetry.get("truck", {})                 # read-only telemetry view
    self.sdk.controller.set_steering(0.3)               # writes *control intent*
    self.sdk.settings.get("acc", {})                    # persisted settings (read)
    self.sdk.tags.acc_speed = 80                        # publish UI tags

Key design choice (improvement over the old code):
plugins never touch the physical input device directly.  They only write
*control intents* into shared state.  The Engine process is the single owner
of the real Controller and flushes those intents to vJoy / SDK once per frame.
This removes the race where multiple processes fought over the virtual joystick.
"""

from typing import Any, Dict


# --- Control intent keys (the Engine reads these and applies them) ------------
CTL_STEERING = "ctl_steering"
CTL_THROTTLE = "ctl_throttle"
CTL_BRAKE = "ctl_brake"
CTL_BLINKER = "ctl_blinker"
CTL_PAY_TOLL = "ctl_pay_toll"


class _SharedStateView:
    """Thin get/set wrapper around the shared managed dict."""

    def __init__(self, state: Dict[str, Any]):
        self._state = state

    def get(self, key: str, default: Any = None) -> Any:
        try:
            return self._state.get(key, default)
        except Exception:
            return default

    def set(self, key: str, value: Any):
        self._state[key] = value

    def update_batch(self, data: Dict[str, Any]):
        self._state.update(data)

    def get_all(self) -> Dict[str, Any]:
        return dict(self._state)


class _TelemetryView:
    """
    Read-only access to the latest telemetry snapshot the Engine published
    under shared_state["telemetry"].  Behaves like a dict for compatibility
    with ``self.sdk.telemetry.get("truck", {})``.
    """

    def __init__(self, state: Dict[str, Any]):
        self._state = state

    @property
    def data(self) -> Dict[str, Any]:
        return self._state.get("telemetry", {}) or {}

    def get(self, key: str, default: Any = None) -> Any:
        return self.data.get(key, default)

    def __getitem__(self, key: str) -> Any:
        return self.data[key]

    def __contains__(self, key: str) -> bool:
        return key in self.data


class _ControllerProxy:
    """
    Plugins call this exactly like the real Controller, but instead of touching
    the device it stores the desired value as an intent in shared state.
    The Engine flushes the latest intents to the real device each frame.
    """

    def __init__(self, state: Dict[str, Any]):
        self._state = state

    def set_steering(self, value: float):
        self._state[CTL_STEERING] = float(max(-1.0, min(1.0, value)))

    def set_throttle(self, value: float):
        self._state[CTL_THROTTLE] = float(max(0.0, min(1.0, value)))

    def set_brake(self, value: float):
        self._state[CTL_BRAKE] = float(max(0.0, min(1.0, value)))

    def set_blinker(self, side: str):
        self._state[CTL_BLINKER] = side

    def stop_completely(self):
        self._state[CTL_THROTTLE] = 0.0
        self._state[CTL_BRAKE] = 1.0

    def pay_toll(self):
        self._state[CTL_PAY_TOLL] = True


class _Tags:
    """
    A namespace plugins use to publish UI values:  ``self.tags.acc_speed = 80``.
    Everything is written to shared_state under ``tags.<attr>`` so the UI/HUD
    (in another process) can read it.  Reading an unknown tag returns None.
    """

    def __init__(self, state: Dict[str, Any], plugin_name: str = "plugin"):
        # Use object.__setattr__ to avoid recursion into __setattr__.
        object.__setattr__(self, "_state", state)
        object.__setattr__(self, "_prefix", f"tags.{plugin_name}.")

    def __setattr__(self, name: str, value: Any):
        self._state[self._prefix + name] = value

    def __getattr__(self, name: str) -> Any:
        # __getattr__ only fires for names not found normally.
        return self._state.get(self._prefix + name)


class _SettingsView:
    """Read-only snapshot of persisted settings, published by the Engine."""

    def __init__(self, state: Dict[str, Any]):
        self._state = state

    def get(self, key: str, default: Any = None) -> Any:
        return (self._state.get("settings", {}) or {}).get(key, default)


class PluginSDK:
    """The single object handed to every plugin instance."""

    def __init__(self, shared_dict: Dict[str, Any], plugin_name: str = "plugin"):
        self._state = shared_dict
        self.name = plugin_name

        self.shared_state = _SharedStateView(shared_dict)
        self.telemetry = _TelemetryView(shared_dict)
        self.controller = _ControllerProxy(shared_dict)
        self.tags = _Tags(shared_dict, plugin_name)
        self.settings = _SettingsView(shared_dict)

    # Convenience pass-throughs so ``self.sdk.get/set`` also work directly.
    def get(self, key: str, default: Any = None) -> Any:
        return self.shared_state.get(key, default)

    def set(self, key: str, value: Any):
        self.shared_state.set(key, value)
