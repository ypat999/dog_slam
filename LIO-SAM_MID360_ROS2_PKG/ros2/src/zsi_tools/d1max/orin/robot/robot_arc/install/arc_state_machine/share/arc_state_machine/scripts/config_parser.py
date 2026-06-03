#!/usr/bin/env python3

"""
Configuration parser for ARC State Machine testing.

This module parses the arc_state_machine.conf file to extract:
- State definitions and mappings
- Transition flag definitions
- State transition rules (from_state, flag, to_state)

Usage:
    from config_parser import ArcStateMachineConfig

    config = ArcStateMachineConfig("/path/to/arc_state_machine.conf")
    available_transitions = config.get_available_transitions(current_state)
    next_state = config.get_next_state(current_state, transition_flag)
"""

import os
from typing import Dict, List, Tuple, Optional


class ArcStateMachineConfig:
    """Parser and interface for ARC state machine configuration."""

    def __init__(self, config_path: str):
        """
        Initialize configuration parser.

        Args:
            config_path: Path to arc_state_machine.conf file
        """
        self.config_path = config_path
        self.states: Dict[str, int] = {}  # state_name -> state_id
        self.state_names: Dict[int, str] = {}  # state_id -> state_name
        self.flags: Dict[str, int] = {}  # flag_name -> flag_id
        self.flag_names: Dict[int, str] = {}  # flag_id -> flag_name
        self.transitions: Dict[Tuple[int, int], int] = {}  # (from_state, flag) -> to_state

        self._parse_config()

    def _parse_config(self) -> None:
        """Parse the configuration file and populate internal data structures."""
        if not os.path.exists(self.config_path):
            raise FileNotFoundError(f"Configuration file not found: {self.config_path}")

        with open(self.config_path, 'r') as f:
            content = f.read().strip()

        lines = [line.strip() for line in content.split('\n') if line.strip()]

        # Parse states section
        self._parse_states_section(lines)

        # Parse change flags section
        self._parse_flags_section(lines)

        # Parse transitions section
        self._parse_transitions_section(lines)

        print(f"Loaded configuration: {len(self.states)} states, {len(self.flags)} flags, {len(self.transitions)} transitions")

    def _parse_states_section(self, lines: List[str]) -> None:
        """Parse the states section from begin_state to end_state."""
        start_idx = None
        end_idx = None

        for i, line in enumerate(lines):
            if line == "begin_state":
                start_idx = i
            elif line == "end_state":
                end_idx = i
                break

        if start_idx is None or end_idx is None:
            raise ValueError("Invalid config: missing begin_state or end_state markers")

        # First line after begin_state should be the count
        state_count = int(lines[start_idx + 1])

        # Parse state names
        for i, state_name in enumerate(lines[start_idx + 2:start_idx + 2 + state_count]):
            self.states[state_name] = i
            self.state_names[i] = state_name

    def _parse_flags_section(self, lines: List[str]) -> None:
        """Parse the change flags section from begin_change_flag to end_change_flag."""
        start_idx = None
        end_idx = None

        for i, line in enumerate(lines):
            if line == "begin_change_flag":
                start_idx = i
            elif line == "end_change_flag":
                end_idx = i
                break

        if start_idx is None or end_idx is None:
            raise ValueError("Invalid config: missing begin_change_flag or end_change_flag markers")

        # First line after begin_change_flag should be the count
        flag_count = int(lines[start_idx + 1])

        # Parse flag names
        for i, flag_name in enumerate(lines[start_idx + 2:start_idx + 2 + flag_count]):
            self.flags[flag_name] = i
            self.flag_names[i] = flag_name

    def _parse_transitions_section(self, lines: List[str]) -> None:
        """Parse the transitions section from begin_transition to end_transition."""
        start_idx = None
        end_idx = None

        for i, line in enumerate(lines):
            if line == "begin_transition":
                start_idx = i
            elif line == "end_transition":
                end_idx = i
                break

        if start_idx is None or end_idx is None:
            raise ValueError("Invalid config: missing begin_transition or end_transition markers")

        # First line after begin_transition should be the count
        transition_count = int(lines[start_idx + 1])

        # Parse transitions: "FROM_STATE FLAG_NAME TO_STATE"
        for line in lines[start_idx + 2:start_idx + 2 + transition_count]:
            parts = line.split()
            if len(parts) != 3:
                continue

            from_state_name, flag_name, to_state_name = parts

            # Convert names to IDs
            from_state_id = self.states.get(from_state_name)
            flag_id = self.flags.get(flag_name)
            to_state_id = self.states.get(to_state_name)

            if from_state_id is not None and flag_id is not None and to_state_id is not None:
                self.transitions[(from_state_id, flag_id)] = to_state_id

    def get_available_transitions(self, current_state: int) -> List[Tuple[int, str, int, str]]:
        """
        Get all available transitions from the current state.

        Args:
            current_state: Current state ID

        Returns:
            List of tuples: (flag_id, flag_name, next_state_id, next_state_name)
        """
        available = []

        for (from_state, flag_id), to_state in self.transitions.items():
            if from_state == current_state:
                flag_name = self.flag_names.get(flag_id, f"UNKNOWN_{flag_id}")
                to_state_name = self.state_names.get(to_state, f"UNKNOWN_{to_state}")
                available.append((flag_id, flag_name, to_state, to_state_name))

        return sorted(available, key=lambda x: x[0])  # Sort by flag_id

    def get_next_state(self, current_state: int, transition_flag: int) -> Optional[int]:
        """
        Get the next state for a given current state and transition flag.

        Args:
            current_state: Current state ID
            transition_flag: Transition flag ID

        Returns:
            Next state ID if transition is valid, None otherwise
        """
        return self.transitions.get((current_state, transition_flag))

    def is_valid_transition(self, current_state: int, transition_flag: int) -> bool:
        """
        Check if a transition is valid from the current state.

        Args:
            current_state: Current state ID
            transition_flag: Transition flag ID

        Returns:
            True if transition is valid, False otherwise
        """
        return (current_state, transition_flag) in self.transitions

    def get_state_name(self, state_id: int) -> str:
        """Get state name by ID."""
        return self.state_names.get(state_id, f"UNKNOWN_STATE_{state_id}")

    def get_flag_name(self, flag_id: int) -> str:
        """Get flag name by ID."""
        return self.flag_names.get(flag_id, f"UNKNOWN_FLAG_{flag_id}")

    def get_state_id(self, state_name: str) -> Optional[int]:
        """Get state ID by name."""
        return self.states.get(state_name)

    def get_flag_id(self, flag_name: str) -> Optional[int]:
        """Get flag ID by name."""
        return self.flags.get(flag_name)

    def print_summary(self) -> None:
        """Print a summary of the loaded configuration."""
        print(f"\n=== ARC State Machine Configuration Summary ===")
        print(f"Config file: {self.config_path}")
        print(f"States: {len(self.states)}")
        for state_id, state_name in sorted(self.state_names.items()):
            print(f"  {state_id}: {state_name}")

        print(f"\nTransition Flags: {len(self.flags)}")
        for flag_id, flag_name in sorted(self.flag_names.items()):
            print(f"  {flag_id}: {flag_name}")

        print(f"\nTransitions: {len(self.transitions)}")
        for (from_state, flag), to_state in sorted(self.transitions.items()):
            from_name = self.get_state_name(from_state)
            flag_name = self.get_flag_name(flag)
            to_name = self.get_state_name(to_state)
            print(f"  {from_name} --[{flag_name}]--> {to_name}")


def main():
    """Test the configuration parser with the default config file."""
    import sys

    if len(sys.argv) > 1:
        config_path = sys.argv[1]
    else:
        # Default path
        config_path = "/home/zy/JSZR/jszr_workspace/src/arc/src/arc_state_machine/config/arc_state_machine.conf"

    try:
        config = ArcStateMachineConfig(config_path)
        config.print_summary()

        # Test with STANDBY state (ID 0)
        print(f"\n=== Available transitions from STANDBY (0) ===")
        transitions = config.get_available_transitions(0)
        for flag_id, flag_name, next_state, next_state_name in transitions:
            print(f"  {flag_id}: {flag_name} --> {next_state_name}")

    except Exception as e:
        print(f"Error: {e}")
        return 1

    return 0


if __name__ == "__main__":
    exit(main())