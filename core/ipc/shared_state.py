import multiprocessing as mp
from typing import Any, Dict
import ctypes

class SharedState:
    """
    High-performance shared memory state for inter-process communication.
    Allows the engine to write telemetry and plugins to read it without locks.
    """
    def __init__(self):
        # We use a Manager for flexible dictionary-like shared state
        self._manager = mp.Manager()
        self._state = self._manager.dict()

    def set(self, key: str, value: Any):
        self._state[key] = value

    def get(self, key: str, default: Any = None) -> Any:
        return self._state.get(key, default)

    def update_batch(self, data: Dict[str, Any]):
        self._state.update(data)

    def get_all(self) -> Dict[str, Any]:
        return dict(self._state)
