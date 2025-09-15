#!/usr/bin/env python3
"""
Zenoh Topic Frequency Monitor
Similar to ROS2 'ros2 topic hz' command - monitors message rates on zenoh topics
"""

import argparse
import threading
import time
from typing import Dict

import zenoh

from woblpy.common.node import Node


class TopicStats:
    """Tracks statistics for a single topic"""

    def __init__(self):
        self.total_messages = 0
        self.messages = 0
        self.last_hz = 0.0
        self.timestamp = time.time()
        self.lock = threading.Lock()

    def add_message(self):
        """Record a new message arrival"""
        with self.lock:
            self.total_messages += 1
            self.messages += 1

    def calculate_hz(self) -> float:
        """Calculate current message frequency"""
        with self.lock:
            now = time.time()
            elapsed = now - self.timestamp
            self.last_hz = self.messages / elapsed if elapsed > 0 else 0.0
            self.timestamp = now
            self.messages = 0  # Reset count after calculating Hz
            return self.last_hz


class ZenohHzMonitor(Node):
    """Monitors message rates on all zenoh topics"""

    def __init__(self, topic_filter: str = "**", update_interval: float = 2.0):
        super().__init__()
        self.topic_filter = topic_filter
        self.update_interval = update_interval
        self.stats: Dict[str, TopicStats] = {}
        self.stats_lock = threading.Lock()

        self.add_sub(self.topic_filter, callback=self._message_callback)

        # Set up timer for display updates
        self.add_timer(self.display_stats, 1.0 / update_interval)

        print(f"Monitoring zenoh topics matching: {self.topic_filter}")
        print("Press Ctrl+C to stop\n")

    def _message_callback(self, sample: zenoh.Sample):
        """Callback for zenoh messages"""
        topic = str(sample.key_expr)

        with self.stats_lock:
            if topic not in self.stats:
                self.stats[topic] = TopicStats()
            self.stats[topic].add_message()

    def display_stats(self):
        """Display current statistics"""
        # Clear screen
        print("\033[2J\033[H", end="")

        print(f"Zenoh Topic Frequency Monitor - Update every {self.update_interval}s")
        print("=" * 80)

        with self.stats_lock:
            if not self.stats:
                print("No topics detected yet...")
                return

            # Sort topics by name for consistent display
            sorted_topics = sorted(self.stats.items())

            print(f"{'Topic':<50} {'Hz':<10} {'Total Messages':<15}")
            print("-" * 80)

            for topic, topic_stats in sorted_topics:
                hz = topic_stats.calculate_hz()
                total = topic_stats.total_messages

                # Format Hz with appropriate precision
                if hz < 0.01:
                    hz_str = "0.00"
                elif hz < 1:
                    hz_str = f"{hz:.3f}"
                elif hz < 10:
                    hz_str = f"{hz:.2f}"
                else:
                    hz_str = f"{hz:.1f}"

                print(f"{topic:<50} {hz_str:<10} {total:<15}")

        print("\nPress Ctrl+C to stop")


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(
        description="Monitor message rates on zenoh topics",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument(
        "-t",
        "--topic",
        default="**",
        help="Topic pattern to monitor (default: ** for all topics)",
    )

    parser.add_argument(
        "-r",
        "--rate",
        type=float,
        default=1.0,
        help="Display update rate in seconds (default: 1.0)",
    )

    args = parser.parse_args()

    # Create and start monitor
    with ZenohHzMonitor(topic_filter=args.topic, update_interval=args.rate) as monitor:
        monitor.spin()


if __name__ == "__main__":
    main()
