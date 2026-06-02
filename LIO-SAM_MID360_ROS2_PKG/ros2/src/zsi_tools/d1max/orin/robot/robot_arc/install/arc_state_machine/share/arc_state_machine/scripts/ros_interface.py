#!/usr/bin/env python3

"""
ROS2 integration module for ARC State Machine testing.

This module provides ROS2 communication capabilities for:
- Subscribing to /arc/arc_state to monitor current state
- Publishing to /arc/arc_change_flag_test to trigger transitions
- Managing ROS2 node lifecycle for testing operations

Usage:
    from ros_interface import ArcTestInterface

    # Initialize interface
    interface = ArcTestInterface()

    # Wait for current state
    current_state = interface.wait_for_current_state(timeout=5.0)

    # Trigger transition
    success = interface.trigger_transition(transition_flag)
"""

import threading
import time
from typing import Optional, Callable

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from robots_dog_msgs.msg import ArcState, ArcChangeFlag


class ArcStateMonitor(Node):
    """Monitor current ARC state via /arc/arc_state subscription."""

    def __init__(self, state_callback: Optional[Callable[[int], None]] = None):
        """
        Initialize state monitor.

        Args:
            state_callback: Optional callback function called when state changes
        """
        super().__init__('arc_state_monitor')

        self.current_state: Optional[int] = None
        self.last_update_time: Optional[float] = None
        self.state_callback = state_callback
        self._lock = threading.Lock()

        # Create subscription with matching QoS (best effort + volatile)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.state_sub = self.create_subscription(
            ArcState,
            '/arc/arc_state',
            self._state_callback,
            qos_profile
        )

        self.get_logger().info("ArcStateMonitor initialized, waiting for state messages...")

    def _state_callback(self, msg: ArcState) -> None:
        """Handle incoming ArcState messages."""
        with self._lock:
            previous_state = self.current_state
            self.current_state = msg.state
            self.last_update_time = time.time()

            self.get_logger().debug(f"State updated: {previous_state} -> {self.current_state}")

            # Call user callback if provided and state changed
            if self.state_callback and previous_state != self.current_state:
                try:
                    self.state_callback(self.current_state)
                except Exception as e:
                    self.get_logger().error(f"State callback error: {e}")

    def get_current_state(self) -> Optional[int]:
        """Get the current state (thread-safe)."""
        with self._lock:
            return self.current_state

    def get_last_update_time(self) -> Optional[float]:
        """Get timestamp of last state update."""
        with self._lock:
            return self.last_update_time

    def wait_for_state(self, timeout: float = 5.0) -> Optional[int]:
        """
        Wait for a state message to arrive.

        Args:
            timeout: Maximum time to wait in seconds

        Returns:
            Current state ID if received, None if timeout
        """
        start_time = time.time()

        while time.time() - start_time < timeout:
            current = self.get_current_state()
            if current is not None:
                return current
            time.sleep(0.1)

        self.get_logger().warn(f"Timeout waiting for state message after {timeout}s")
        return None


class TransitionPublisher(Node):
    """Publish transition commands via /arc/arc_change_flag_test."""

    def __init__(self):
        """Initialize transition publisher."""
        super().__init__('arc_transition_publisher')

        # Create publisher with reliable QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.flag_pub = self.create_publisher(
            ArcChangeFlag,
            '/arc/arc_change_flag_test',
            qos_profile
        )

        self.get_logger().info("TransitionPublisher initialized")

    def publish_transition(self, transition_flag: int) -> bool:
        """
        Publish a transition command.

        Args:
            transition_flag: Transition flag ID to publish

        Returns:
            True if published successfully, False otherwise
        """
        try:
            msg = ArcChangeFlag()
            msg.stamp = self.get_clock().now().to_msg()
            msg.change_flag = transition_flag

            self.flag_pub.publish(msg)

            self.get_logger().info(f"Published transition flag: {transition_flag}")
            return True

        except Exception as e:
            self.get_logger().error(f"Failed to publish transition {transition_flag}: {e}")
            return False


class ArcTestInterface:
    """Combined interface for ARC state machine testing."""

    def __init__(self, state_callback: Optional[Callable[[int], None]] = None):
        """
        Initialize the test interface.

        Args:
            state_callback: Optional callback for state changes
        """
        self.monitor = ArcStateMonitor(state_callback)
        self.publisher = TransitionPublisher()
        self._executor = None
        self._spin_thread = None
        self._running = False

    def start(self) -> None:
        """Start the ROS2 interface (spinning nodes)."""
        if self._running:
            return

        # Create executor and add nodes
        from rclpy.executors import MultiThreadedExecutor
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self.monitor)
        self._executor.add_node(self.publisher)

        # Start spinning in background thread
        self._running = True
        self._spin_thread = threading.Thread(target=self._spin_executor)
        self._spin_thread.daemon = True
        self._spin_thread.start()

        print("ArcTestInterface started")

    def stop(self) -> None:
        """Stop the ROS2 interface."""
        if not self._running:
            return

        self._running = False

        if self._executor:
            self._executor.shutdown()

        if self._spin_thread:
            self._spin_thread.join(timeout=2.0)

        print("ArcTestInterface stopped")

    def _spin_executor(self) -> None:
        """Spin the executor (runs in background thread)."""
        try:
            while self._running and rclpy.ok():
                self._executor.spin_once(timeout_sec=0.1)
        except Exception as e:
            print(f"Executor error: {e}")

    def wait_for_current_state(self, timeout: float = 5.0) -> Optional[int]:
        """
        Wait for current state information.

        Args:
            timeout: Maximum time to wait in seconds

        Returns:
            Current state ID if available, None if timeout
        """
        return self.monitor.wait_for_state(timeout)

    def get_current_state(self) -> Optional[int]:
        """Get current state (immediate, may be None if no data yet)."""
        return self.monitor.get_current_state()

    def trigger_transition(self, transition_flag: int) -> bool:
        """
        Trigger a state transition.

        Args:
            transition_flag: Transition flag ID

        Returns:
            True if command was published successfully
        """
        return self.publisher.publish_transition(transition_flag)

    def wait_for_state_change(self, initial_state: int, timeout: float = 10.0) -> Optional[int]:
        """
        Wait for state to change from initial_state.

        Args:
            initial_state: Expected current state
            timeout: Maximum time to wait for change

        Returns:
            New state ID if changed, None if timeout or error
        """
        start_time = time.time()

        while time.time() - start_time < timeout:
            current = self.get_current_state()
            if current is not None and current != initial_state:
                return current
            time.sleep(0.1)

        return None

    def __enter__(self):
        """Context manager entry."""
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.stop()


def main():
    """Test the ROS2 interface functionality."""
    import sys

    rclpy.init()

    try:
        def on_state_change(new_state: int):
            print(f"State changed to: {new_state}")

        with ArcTestInterface(state_callback=on_state_change) as interface:
            print("Waiting for current state...")
            current_state = interface.wait_for_current_state(timeout=10.0)

            if current_state is not None:
                print(f"Current state: {current_state}")

                # Test publishing a transition (T00_KEEP_THE_SAME)
                print("Testing transition publication...")
                success = interface.trigger_transition(0)
                print(f"Transition published: {success}")

                # Keep running for a bit to see any responses
                print("Monitoring for 5 seconds...")
                time.sleep(5.0)
            else:
                print("Failed to get current state")
                return 1

    except KeyboardInterrupt:
        print("Interrupted by user")
    except Exception as e:
        print(f"Error: {e}")
        return 1
    finally:
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    exit(main())