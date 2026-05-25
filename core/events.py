import logging
from typing import Callable, Dict, List, Any

class EventBus:
    """
    A simple asynchronous event bus for communication between
    core modules and plugins.
    """
    def __init__(self):
        self._subscribers: Dict[str, List[Callable]] = {}

    def subscribe(self, event_type: str, callback: Callable):
        """Subscribe to a specific event type."""
        if event_type not in self._subscribers:
            self._subscribers[event_type] = []
        self._subscribers[event_type].append(callback)
        logging.debug(f"Subscribed to event: {event_type}")

    def publish(self, event_type: str, data: Any = None):
        """Publish an event to all subscribers."""
        if event_type in self._subscribers:
            for callback in self._subscribers[event_type]:
                try:
                    callback(data)
                except Exception as e:
                    logging.error(f"Error in event callback for {event_type}: {e}")

# Global event bus instance
bus = EventBus()
