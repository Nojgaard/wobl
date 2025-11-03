import signal
import threading
import time
from typing import Callable, Optional

import zenoh
from google.protobuf.message import Message


class Node:
    def __init__(self):
        self.session = zenoh.open(zenoh.Config())
        self._pubs: list[zenoh.Publisher] = []
        self._subs: list[zenoh.Subscriber] = []
        self._timer_threads: list[threading.Thread] = []
        self._is_open = True
        self._lock = threading.RLock()  # Protect shared state
        # Register signal handlers for graceful shutdown
        signal.signal(signal.SIGTERM, self._signal_handler)
        signal.signal(signal.SIGINT, self._signal_handler)

    def _signal_handler(self, sig, frame):
        self.close()

    def spin(self):
        try:
            while self._is_open:
                time.sleep(1)
        except KeyboardInterrupt:
            pass

    def add_pub(self, key: str):
        self._pubs.append(self.session.declare_publisher(key))
        return len(self._pubs) - 1

    def add_sub(
        self,
        key: str,
        message: Optional[Message] = None,
        callback: Optional[Callable[[zenoh.Sample], None]] = None,
    ):
        if message is not None:

            def msg_callback(sample: zenoh.Sample):
                message.ParseFromString(sample.payload.to_bytes())

            callback = msg_callback

        if callback is None:
            raise ValueError("Either message or callback must be provided")

        def safe_callback(sample: zenoh.Sample):
            with self._lock:
                callback(sample)

        self._subs.append(self.session.declare_subscriber(key, safe_callback))
        return self._subs[-1]

    def send(self, pub_id: int, message: Message):
        if pub_id < 0 or pub_id >= len(self._pubs):
            raise ValueError("Invalid publisher ID")
        self._pubs[pub_id].put(message.SerializeToString())

    def send_string(self, pub_id: int, message: str):
        if pub_id < 0 or pub_id >= len(self._pubs):
            raise ValueError("Invalid publisher ID")
        self._pubs[pub_id].put(message.encode("utf-8"))

    def add_timer(self, callback: Callable[[], None], frequency_hz: float):
        period = 1.0 / frequency_hz

        def timer_loop():
            next_call = time.time()
            while self._is_open:
                with self._lock:
                    callback()

                next_call += period
                sleep_time = next_call - time.time()

                if sleep_time > 0:
                    time.sleep(sleep_time)

        timer_thread = threading.Thread(target=timer_loop, daemon=True)
        self._timer_threads.append(timer_thread)
        timer_thread.start()

    def close(self):
        if not self._is_open:
            return

        self._is_open = False
        for thread in self._timer_threads:
            thread.join(timeout=0.1)

        # Wait for timer threads to finish (with timeout)
        for timer_thread in self._timer_threads:
            timer_thread.join(timeout=0.1)

        # Close publishers and subscribers
        for pubsub in self._pubs + self._subs:
            pubsub.undeclare()
        self.session.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()
        return False

    def is_open(self) -> bool:
        return self._is_open
