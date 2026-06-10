import multiprocessing as mp
from typing import Any, Dict, Optional


class SharedState:
    """
    Shared dictionary-style state for inter-process communication.

    Improvement over the original: a SharedState can wrap an *existing* managed
    dict.  The bootloader creates ONE ``manager.dict()`` and hands the same
    proxy to the Engine, UI, HUD and every plugin, so they all see the same
    data.  Previously each component created its own manager and they never
    actually shared anything.
    """

    def __init__(self, shared_dict: Optional[Dict[str, Any]] = None):
        if shared_dict is not None:
            self._manager = None
            self._state = shared_dict
        else:
            self._manager = mp.Manager()
            self._state = self._manager.dict()

    @property
    def raw(self) -> Dict[str, Any]:
        """The underlying managed dict (picklable, shareable across processes)."""
        return self._state

    def set(self, key: str, value: Any):
        self._state[key] = value

    def get(self, key: str, default: Any = None) -> Any:
        try:
            return self._state.get(key, default)
        except Exception:
            return default

    def update_batch(self, data: Dict[str, Any]):
        self._state.update(data)

    def get_all(self) -> Dict[str, Any]:
        return dict(self._state)
