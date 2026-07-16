import logging
import time
from sdk.base_plugin import BasePlugin


# --- Tuning -----------------------------------------------------------------
APPROACH_SPEED_MS = 7.0   # if we're creeping slower than this, we may be at a booth
DWELL_S = 1.5             # how long we must be nearly stopped before we "pay"
STOP_SPEED_MS = 0.6       # "nearly stopped" threshold
COOLDOWN_S = 12.0         # don't try to pay again for this long after a payment


class Plugin(BasePlugin):
    """Automatic toll / gate payment.

    ETS2's toll booths and some ferries / borders require the truck to stop and
    press the action key (Enter / 'E') to pay. This plugin detects that
    situation — the autopilot has brought us to a near-stop on the route — and
    presses the pay key for us, once, then waits.

    Detection is intentionally conservative: we only pay when the autopilot is
    on, the truck has been essentially stopped for ``DWELL_S`` seconds while
    still on a route (so we don't pay at random red lights or traffic jams),
    and a cooldown prevents double-paying. This replaces the old flaky
    vision-based toll detection (yellow pixels) that stopped the truck every
    few seconds for no reason.
    """

    NAME = "toll"

    def on_start(self):
        logging.info("Toll plugin started.")
        self.enabled = True
        self._dwell = 0.0       # how long we've been stopped near a booth
        self._last_pay = 0.0    # timestamp of the last payment (cooldown)

    def on_stop(self):
        # Never leave the autopilot stuck in a PAY_TOLL state we caused.
        self.sdk.shared_state.set("enable_toll", False)

    def on_tick(self, delta_time: float):
        dt = max(delta_time, 1e-3)

        # Only act while the autopilot is driving — never pay manually.
        if not self.sdk.shared_state.get("autopilot_active", False):
            self._dwell = 0.0
            return

        speed = abs(float(self.sdk.shared_state.get("truck_speed_ms", 0.0) or 0.0))
        nav = bool(self.sdk.shared_state.get("nav_active", False))
        toll_confirmed = bool(self.sdk.shared_state.get("toll_detected", False))
        now = time.time()
        on_cooldown = (now - self._last_pay) < COOLDOWN_S

        if nav and toll_confirmed and not on_cooldown and speed <= APPROACH_SPEED_MS:
            # We're creeping on a route — likely approaching a booth. If we then
            # come to a full stop, that's the pay moment.
            if speed <= STOP_SPEED_MS:
                self._dwell += dt
                if self._dwell >= DWELL_S:
                    self._pay()
                    self._dwell = 0.0
                    self._last_pay = now
            else:
                # Still rolling toward the booth — reset the dwell timer so a
                # full stop later still counts from zero.
                self._dwell = 0.0
        else:
            self._dwell = 0.0

        self.tags.toll_dwell = round(self._dwell, 1)

    def _pay(self):
        """Press the action key once to pay the toll / open the gate."""
        try:
            self.sdk.controller.pay_toll()
            logging.info("Toll: paid (action key).")
            self.sdk.shared_state.set("tts_message", "Platba mýta.")
        except Exception as e:
            logging.error("Toll: pay failed: %s", e)
