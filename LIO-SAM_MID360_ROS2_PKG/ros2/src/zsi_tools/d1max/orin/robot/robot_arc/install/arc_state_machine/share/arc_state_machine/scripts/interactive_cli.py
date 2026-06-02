#!/usr/bin/env python3

"""
Interactive command-line interface for ARC State Machine testing.

This module provides a user-friendly interactive interface for:
- Displaying current state information
- Showing available transitions with descriptions
- Previewing target states
- Confirming transition execution
- Providing help and status information

Usage:
    from interactive_cli import ArcTestCLI
    from config_parser import ArcStateMachineConfig
    from ros_interface import ArcTestInterface

    config = ArcStateMachineConfig("config.conf")
    interface = ArcTestInterface()
    cli = ArcTestCLI(config, interface)
    cli.run()
"""

import os
import sys
import time
import select
from typing import List, Tuple, Optional

from config_parser import ArcStateMachineConfig
from ros_interface import ArcTestInterface


class TerminalColors:
    """ANSI color codes and formatting optimized for dark/black terminal backgrounds."""

    # Standard colors - optimized for dark background
    RED = '\033[91m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    MAGENTA = '\033[95m'
    CYAN = '\033[96m'
    WHITE = '\033[97m'
    GRAY = '\033[37m'  # Lighter gray for better contrast on black

    # Bright colors - ideal for black backgrounds
    BRIGHT_RED = '\033[1;91m'
    BRIGHT_GREEN = '\033[1;92m'
    BRIGHT_YELLOW = '\033[1;93m'
    BRIGHT_BLUE = '\033[1;94m'
    BRIGHT_MAGENTA = '\033[1;95m'
    BRIGHT_CYAN = '\033[1;96m'
    BRIGHT_WHITE = '\033[1;97m'

    # Additional colors for black background
    ORANGE = '\033[38;5;208m'  # Orange for variety
    PURPLE = '\033[38;5;135m'  # Purple for accents
    LIGHT_BLUE = '\033[38;5;117m'  # Light blue for contrast

    # Formatting
    BOLD = '\033[1m'
    DIM = '\033[2m'
    UNDERLINE = '\033[4m'
    RESET = '\033[0m'

    # Background colors
    BG_BLACK = '\033[40m'
    BG_BLUE = '\033[44m'
    BG_GREEN = '\033[42m'
    BG_DARK_GRAY = '\033[100m'  # Dark gray background for highlights

    @staticmethod
    def colorize(text: str, color: str, bold: bool = False) -> str:
        """Apply color and formatting to text."""
        prefix = f"{TerminalColors.BOLD}{color}" if bold else color
        return f"{prefix}{text}{TerminalColors.RESET}"

    @staticmethod
    def header(text: str) -> str:
        """Format text as a header with bold blue color."""
        return TerminalColors.colorize(text, TerminalColors.BRIGHT_BLUE, bold=True)

    @staticmethod
    def success(text: str) -> str:
        """Format text as success message with green color."""
        return TerminalColors.colorize(text, TerminalColors.BRIGHT_GREEN, bold=True)

    @staticmethod
    def warning(text: str) -> str:
        """Format text as warning message with yellow color."""
        return TerminalColors.colorize(text, TerminalColors.BRIGHT_YELLOW, bold=True)

    @staticmethod
    def error(text: str) -> str:
        """Format text as error message with red color."""
        return TerminalColors.colorize(text, TerminalColors.BRIGHT_RED, bold=True)

    @staticmethod
    def info(text: str) -> str:
        """Format text as info message with cyan color."""
        return TerminalColors.colorize(text, TerminalColors.BRIGHT_CYAN, bold=True)

    @staticmethod
    def highlight(text: str) -> str:
        """Format text with white bold highlighting."""
        return TerminalColors.colorize(text, TerminalColors.BRIGHT_WHITE, bold=True)


class ArcTestCLI:
    """Interactive command-line interface for ARC state machine testing."""

    def __init__(self, config: ArcStateMachineConfig, interface: ArcTestInterface):
        """
        Initialize the CLI.

        Args:
            config: Parsed state machine configuration
            interface: ROS2 communication interface
        """
        self.config = config
        self.interface = interface
        self.running = False
        self.current_state: Optional[int] = None
        self.last_update_time: Optional[float] = None
        self.last_displayed_age: Optional[str] = None  # Track last displayed age to avoid unnecessary updates

    def clear_screen(self) -> None:
        """Clear the terminal screen."""
        os.system('clear' if os.name == 'posix' else 'cls')

    def print_header(self) -> None:
        """Print the application header with enhanced visual styling optimized for black background."""
        print("\n")
        header_line = "═" * 70
        print(TerminalColors.colorize(header_line, TerminalColors.BRIGHT_CYAN))
        print()

        title = "🤖 ARC State Machine Testing Tool"
        print(f"    {TerminalColors.colorize(title, TerminalColors.BRIGHT_WHITE, bold=True)}")
        print()
        print(TerminalColors.colorize(header_line, TerminalColors.BRIGHT_CYAN))
        print("\n")

    def print_current_state_info(self) -> None:
        """Print current state information with enhanced styling."""
        section_header = TerminalColors.colorize("📍 CURRENT STATE", TerminalColors.BRIGHT_CYAN, bold=True)
        print(f"  {section_header}")
        print(f"  {TerminalColors.colorize('─' * 30, TerminalColors.CYAN)}")
        print()

        if self.current_state is not None:
            state_name = self.config.get_state_name(self.current_state)

            # Main state display with larger, bold text
            state_display = f"    🎯 {TerminalColors.colorize(state_name, TerminalColors.BRIGHT_GREEN, bold=True)}"
            state_id = f"    {TerminalColors.colorize(f'   State ID: {self.current_state}', TerminalColors.BRIGHT_WHITE)}"

            print(state_display)
            print(state_id)

            if self.last_update_time:
                age = time.time() - self.last_update_time
                if age < 5.0:
                    status_color = TerminalColors.BRIGHT_GREEN
                    status_icon = "🟢"
                elif age < 10.0:
                    status_color = TerminalColors.BRIGHT_YELLOW
                    status_icon = "🟡"
                else:
                    status_color = TerminalColors.BRIGHT_RED
                    status_icon = "🔴"

                # Format age properly - show milliseconds for very recent updates
                if age < 0.1:
                    age_str = f"{age*1000:.0f}ms ago"
                elif age < 1.0:
                    age_str = f"{age:.2f}s ago"
                else:
                    age_str = f"{age:.1f}s ago"

                time_info = f"    {status_icon} {TerminalColors.colorize(f'Last Update: {age_str}', status_color)}"
                print(time_info)
        else:
            error_msg = f"    {TerminalColors.error('❌ State Unknown (waiting for data...)')}"
            print(error_msg)

        print()
        print()

    def print_available_transitions(self) -> List[Tuple[int, str, int, str]]:
        """
        Print available transitions and return them.

        Returns:
            List of available transitions: (flag_id, flag_name, next_state_id, next_state_name)
        """
        if self.current_state is None:
            warning_msg = f"  {TerminalColors.warning('⚠️  Cannot show transitions - current state unknown')}"
            print(warning_msg)
            print()
            return []

        transitions = self.config.get_available_transitions(self.current_state)

        if not transitions:
            warning_msg = f"  {TerminalColors.warning('⚠️  No transitions available from current state')}"
            print(warning_msg)
            print()
            return []

        # Header for transitions section
        section_header = TerminalColors.colorize("🔄 AVAILABLE TRANSITIONS", TerminalColors.BRIGHT_MAGENTA, bold=True)
        print(f"  {section_header}")
        print(f"  {TerminalColors.colorize('─' * 40, TerminalColors.MAGENTA)}")
        print()

        filtered_transitions = [t for t in transitions if t[0] != 0]  # Filter out T00_KEEP_THE_SAME

        for i, (flag_id, flag_name, next_state_id, next_state_name) in enumerate(filtered_transitions, 1):
            # Number with dark gray background for black terminal
            number_display = f" {i:2d} "
            number_bg = f"{TerminalColors.BG_DARK_GRAY}{TerminalColors.BRIGHT_WHITE}{TerminalColors.BOLD}{number_display}{TerminalColors.RESET}"

            # Transition name with bright yellow for high contrast
            transition_name = TerminalColors.colorize(flag_name, TerminalColors.BRIGHT_YELLOW, bold=True)

            # Arrow with bright cyan
            arrow = TerminalColors.colorize("➤", TerminalColors.BRIGHT_CYAN, bold=True)

            # Target state with bright green
            target_state = TerminalColors.colorize(next_state_name, TerminalColors.BRIGHT_GREEN, bold=True)
            state_id = TerminalColors.colorize(f"(ID: {next_state_id})", TerminalColors.GRAY)

            print(f"    {number_bg}  {transition_name}")
            print(f"        {arrow} {target_state} {state_id}")
            print()

        return filtered_transitions

    def print_help(self) -> None:
        """Print help information."""
        section_header = TerminalColors.colorize("📚 COMMAND REFERENCE", TerminalColors.BRIGHT_WHITE, bold=True)
        print(f"  {section_header}")
        print(f"  {TerminalColors.colorize('─' * 35, TerminalColors.BRIGHT_CYAN)}")
        print()

        commands = [
            ("[number]", "Execute transition by number", "🚀"),
            ("r, refresh", "Refresh display", "🔄"),
            ("h, help", "Show this help", "❓"),
            ("q, quit", "Exit application", "👋"),
            ("c, config", "Show configuration summary", "⚙️"),
            ("s, status", "Show detailed status", "📊")
        ]

        for cmd, desc, icon in commands:
            cmd_display = TerminalColors.colorize(f"{cmd:12s}", TerminalColors.BRIGHT_YELLOW, bold=True)
            desc_display = TerminalColors.colorize(desc, TerminalColors.BRIGHT_WHITE)
            print(f"    {icon} {cmd_display} - {desc_display}")

        print()
        print(f"  {TerminalColors.colorize('💡 TIP:', TerminalColors.ORANGE, bold=True)} {TerminalColors.colorize('Press Ctrl+C to exit at any time', TerminalColors.GRAY)}")
        print()

    def print_config_summary(self) -> None:
        """Print configuration summary."""
        section_header = TerminalColors.colorize("⚙️  CONFIGURATION SUMMARY", TerminalColors.BRIGHT_WHITE, bold=True)
        print(f"  {section_header}")
        print(f"  {TerminalColors.colorize('─' * 40, TerminalColors.BRIGHT_CYAN)}")
        print()

        config_items = [
            ("📄 Config file:", os.path.basename(self.config.config_path)),
            ("🔧 States:", str(len(self.config.states))),
            ("🚩 Transition flags:", str(len(self.config.flags))),
            ("✅ Valid transitions:", str(len(self.config.transitions)))
        ]

        for label, value in config_items:
            label_display = TerminalColors.colorize(f"{label:20s}", TerminalColors.BRIGHT_WHITE, bold=True)
            value_display = TerminalColors.colorize(value, TerminalColors.BRIGHT_YELLOW, bold=True)
            print(f"    {label_display} {value_display}")

        print()

    def print_status(self) -> None:
        """Print detailed status information."""
        section_header = TerminalColors.colorize("📊 SYSTEM STATUS", TerminalColors.BRIGHT_CYAN, bold=True)
        print(f"  {section_header}")
        print(f"  {TerminalColors.colorize('─' * 30, TerminalColors.CYAN)}")
        print()

        # ROS2 interface status
        try:
            state = self.interface.get_current_state()
            if state is not None:
                status_icon = "🟢"
                connection_status = TerminalColors.colorize("Active", TerminalColors.BRIGHT_GREEN, bold=True)
                data_status = TerminalColors.colorize("Receiving data", TerminalColors.BRIGHT_GREEN, bold=True)
            else:
                status_icon = "🟡"
                connection_status = TerminalColors.colorize("Connected", TerminalColors.BRIGHT_YELLOW, bold=True)
                data_status = TerminalColors.colorize("No state data", TerminalColors.BRIGHT_YELLOW, bold=True)

            print(f"    {status_icon} {TerminalColors.colorize('ROS2 Connection:', TerminalColors.BRIGHT_WHITE, bold=True):20s} {connection_status}")
            print(f"    📡 {TerminalColors.colorize('State Monitoring:', TerminalColors.BRIGHT_WHITE, bold=True):19s} {data_status}")

        except Exception as e:
            status_icon = "🔴"
            error_status = TerminalColors.colorize(f"Error - {e}", TerminalColors.BRIGHT_RED, bold=True)
            print(f"    {status_icon} {TerminalColors.colorize('ROS2 Connection:', TerminalColors.BRIGHT_WHITE, bold=True):20s} {error_status}")

        # Configuration status
        if self.config.states:
            config_icon = "🟢"
            config_status = TerminalColors.colorize("Loaded", TerminalColors.BRIGHT_GREEN, bold=True)
        else:
            config_icon = "🔴"
            config_status = TerminalColors.colorize("Not loaded", TerminalColors.BRIGHT_RED, bold=True)

        print(f"    {config_icon} {TerminalColors.colorize('Configuration:', TerminalColors.BRIGHT_WHITE, bold=True):20s} {config_status}")

        print()

    def get_user_input_non_blocking(self, timeout: float = 1.0) -> Optional[str]:
        """Get user input with timeout (non-blocking)."""
        try:
            # Check if input is available within timeout
            if select.select([sys.stdin], [], [], timeout)[0]:
                line = sys.stdin.readline()
                if line:
                    return line.strip().lower()
                else:
                    # EOF reached
                    return "quit"
            else:
                # Timeout - no input available
                return None
        except (EOFError, KeyboardInterrupt):
            return "quit"

    def get_user_input(self, prompt: str = "Enter command") -> str:
        """Get user input with prompt (blocking - for compatibility)."""
        try:
            return input(f"{prompt}: ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            return "quit"

    def execute_transition(self, flag_id: int, flag_name: str) -> bool:
        """
        Execute a transition and wait for result.

        Args:
            flag_id: Transition flag ID
            flag_name: Transition flag name

        Returns:
            True if transition was executed successfully
        """
        print(f"🚀 Executing transition: {flag_name}")

        # Store initial state to detect changes
        initial_state = self.current_state

        # Publish transition command
        success = self.interface.trigger_transition(flag_id)

        if not success:
            print("❌ Failed to publish transition command")
            return False

        print("✅ Transition command sent")
        print("⏳ Waiting for state change...")

        # Wait for state change
        new_state = self.interface.wait_for_state_change(initial_state, timeout=5.0)

        if new_state is not None:
            new_state_name = self.config.get_state_name(new_state)
            print(f"✅ State changed to: {new_state_name} (ID: {new_state})")
            return True
        else:
            print("⚠️  No state change detected within timeout")
            return False

    def update_current_state(self) -> None:
        """Update current state from ROS2 interface."""
        new_state = self.interface.get_current_state()
        ros_last_update_time = self.interface.monitor.get_last_update_time()

        # Only update our timestamp if ROS interface received a new message
        if new_state is not None:
            self.current_state = new_state
            # Only update time if ROS interface has a newer timestamp than what we have
            if ros_last_update_time is not None and (self.last_update_time is None or ros_last_update_time != self.last_update_time):
                self.last_update_time = ros_last_update_time

    def update_time_display_only(self) -> bool:
        """
        Update only the time display line without full screen refresh.

        Returns:
            True if display was updated, False if no update needed
        """
        if self.last_update_time is None:
            return False

        age = time.time() - self.last_update_time

        # Format age string
        if age < 0.1:
            age_str = f"{age*1000:.0f}ms ago"
        elif age < 1.0:
            age_str = f"{age:.2f}s ago"
        else:
            age_str = f"{age:.1f}s ago"

        # Only update if age string changed
        if age_str == self.last_displayed_age:
            return False

        self.last_displayed_age = age_str

        # Determine color and icon
        if age < 5.0:
            status_color = TerminalColors.BRIGHT_GREEN
            status_icon = "🟢"
        elif age < 10.0:
            status_color = TerminalColors.BRIGHT_YELLOW
            status_icon = "🟡"
        else:
            status_color = TerminalColors.BRIGHT_RED
            status_icon = "🔴"

        # Temporarily disable partial refresh due to positioning complexity
        # Return False to trigger full refresh instead
        return False

    def display_main_interface(self, force_full_refresh: bool = True) -> None:
        """Display the main interactive interface."""
        if force_full_refresh:
            self.clear_screen()
            self.print_header()
            self.print_current_state_info()
            transitions = self.print_available_transitions()

            if transitions:
                print("💡 Tip: Enter transition number, or 'h' for help")
            else:
                print("💡 Tip: Enter 'r' to refresh, 'h' for help")

            print()
            # Reset the age tracking for fresh display
            self.last_displayed_age = None
        else:
            # Only update time if needed
            self.update_time_display_only()

    def run(self) -> None:
        """Run the interactive CLI main loop."""
        self.running = True

        print("🚀 Starting ARC State Machine Testing Tool...")
        print("⏳ Waiting for initial state information...")

        # Wait for initial state
        initial_state = self.interface.wait_for_current_state(timeout=10.0)

        if initial_state is None:
            print("❌ Failed to receive initial state information")
            print("   Make sure ArcStateMachineNode is running and publishing to /arc/arc_state")
            return

        self.current_state = initial_state
        self.last_update_time = time.time()

        print(f"✅ Connected! Initial state: {self.config.get_state_name(initial_state)}")
        time.sleep(1)

        # Main interaction loop with simplified logic
        force_refresh = True
        previous_state = None
        last_refresh_time = 0

        while self.running:
            try:
                # Update state information
                self.update_current_state()

                # Determine if we need a refresh
                current_time = time.time()
                state_changed = self.current_state != previous_state
                time_to_refresh = (current_time - last_refresh_time) > 10.0  # Only refresh every 10 seconds for time updates

                if force_refresh or state_changed or time_to_refresh:
                    # Display interface
                    self.display_main_interface(force_full_refresh=True)

                    # Show prompt immediately after interface
                    print("Enter command: ", end="", flush=True)

                    # Update tracking
                    force_refresh = False
                    previous_state = self.current_state
                    last_refresh_time = current_time

                # Get user input with generous timeout to reduce CPU usage
                command = self.get_user_input_non_blocking(timeout=3.0)

                # If no command (timeout), continue without any output
                if command is None:
                    continue

                # Handle empty command (user just pressed Enter)
                if command == '':
                    continue  # Simply continue the loop without printing anything

                # User entered a command - handle it
                if command in ['q', 'quit', 'exit']:
                    self.running = False
                    break

                elif command in ['h', 'help']:
                    self.clear_screen()
                    self.print_header()
                    self.print_help()
                    print("\nPress Enter to continue...")
                    self.get_user_input_non_blocking(timeout=30.0)  # Wait for user
                    force_refresh = True

                elif command in ['r', 'refresh']:
                    force_refresh = True

                elif command in ['c', 'config']:
                    self.clear_screen()
                    self.print_header()
                    self.print_config_summary()
                    self.config.print_summary()
                    print("\nPress Enter to continue...")
                    self.get_user_input_non_blocking(timeout=30.0)  # Wait for user
                    force_refresh = True

                elif command in ['s', 'status']:
                    self.clear_screen()
                    self.print_header()
                    self.print_status()
                    print("\nPress Enter to continue...")
                    self.get_user_input_non_blocking(timeout=30.0)  # Wait for user
                    force_refresh = True

                elif command.isdigit():
                    # Execute numbered transition
                    transitions = self.config.get_available_transitions(self.current_state)
                    filtered_transitions = [t for t in transitions if t[0] != 0]

                    choice = int(command)
                    if 1 <= choice <= len(filtered_transitions):
                        flag_id, flag_name, next_state_id, next_state_name = filtered_transitions[choice - 1]

                        print(f"\n🚀 Executing: {TerminalColors.colorize(flag_name, TerminalColors.BRIGHT_YELLOW, bold=True)} → {TerminalColors.colorize(next_state_name, TerminalColors.BRIGHT_GREEN, bold=True)}")
                        self.execute_transition(flag_id, flag_name)
                        print("\nPress Enter to continue...")
                        self.get_user_input_non_blocking(timeout=10.0)  # Brief wait to show result
                        force_refresh = True
                    else:
                        print(f"\n❌ Invalid choice: {choice}")
                        print("Press Enter to continue...")
                        self.get_user_input_non_blocking(timeout=5.0)
                        force_refresh = True

                else:
                    print(f"\n❌ Unknown command: '{command}'. Type 'h' for help.")
                    print("Press Enter to continue...")
                    self.get_user_input_non_blocking(timeout=5.0)
                    force_refresh = True

            except KeyboardInterrupt:
                print("\n\n👋 Exiting...")
                break
            except Exception as e:
                print(f"\n❌ Error: {e}")
                print("Press Enter to continue...")
                self.get_user_input_non_blocking(timeout=5.0)
                force_refresh = True

        print("👋 Goodbye!")


def main():
    """Test the CLI module."""
    print("This module is designed to be used with the main testing script.")
    print("Run arc_test_tool.py instead.")
    return 0


if __name__ == "__main__":
    exit(main())