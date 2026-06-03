#!/usr/bin/env python3

"""
ARC State Machine Testing Tool

Interactive testing tool for ArcStateMachineNode that allows users to:
- Monitor current state in real-time
- View available state transitions
- Trigger transitions manually
- Test state machine behavior

This tool connects to a running ArcStateMachineNode via ROS2 topics:
- Subscribes to /arc/arc_state for current state monitoring
- Publishes to /arc/arc_change_flag_test for transition triggering

Prerequisites:
1. ArcStateMachineNode must be running
2. ROS2 environment must be sourced
3. robots_dog_msgs package must be available

Usage:
    ./arc_test_tool.py [options]

Options:
    -c, --config PATH    Path to arc_state_machine.conf file
    -h, --help          Show help message
    -v, --version       Show version information
    --non-interactive   Run in non-interactive mode (for testing)

Examples:
    # Use default configuration
    ./arc_test_tool.py

    # Use custom configuration file
    ./arc_test_tool.py -c /path/to/custom/config.conf

    # Test configuration parsing only
    ./arc_test_tool.py --non-interactive
"""

import argparse
import os
import sys
import time
from pathlib import Path

import rclpy
from rclpy.exceptions import ROSInterruptException

# Add current directory to Python path for local imports
script_dir = Path(__file__).parent
sys.path.insert(0, str(script_dir))

try:
    from config_parser import ArcStateMachineConfig
    from ros_interface import ArcTestInterface
    from interactive_cli import ArcTestCLI
except ImportError as e:
    print(f"❌ Import error: {e}")
    print("Make sure all required modules are in the same directory")
    sys.exit(1)


# Version information
VERSION = "1.0.0"
TOOL_NAME = "ARC State Machine Testing Tool"


def find_default_config() -> str:
    """
    Find the default configuration file.

    Returns:
        Path to default configuration file

    Raises:
        FileNotFoundError: If default config cannot be found
    """
    # Try multiple possible locations
    possible_paths = [
        # Relative to script directory
        script_dir.parent / "config" / "arc_state_machine.conf",
        # Installed package location
        Path("/home/zy/JSZR/jszr_workspace/src/arc/src/arc_state_machine/config/arc_state_machine.conf"),
        # Build/install directory
        Path("/home/zy/JSZR/jszr_workspace/install/arc_state_machine/share/arc_state_machine/config/arc_state_machine.conf"),
    ]

    for path in possible_paths:
        if path.exists():
            return str(path)

    raise FileNotFoundError(
        "Could not find default arc_state_machine.conf file. "
        "Please specify the path using -c option."
    )


def print_version() -> None:
    """Print version information."""
    print(f"{TOOL_NAME} v{VERSION}")
    print("Interactive testing tool for ArcStateMachineNode")


def print_help() -> None:
    """Print detailed help information."""
    print(__doc__)


def validate_environment() -> bool:
    """
    Validate that the required environment is available.

    Returns:
        True if environment is valid, False otherwise
    """
    print("🔍 Validating environment...")

    # Check if ROS2 is available
    try:
        import rclpy
        print("✅ ROS2 Python bindings available")
    except ImportError:
        print("❌ ROS2 Python bindings not available")
        print("   Make sure ROS2 is installed and sourced")
        return False

    # Check if robots_dog_msgs is available
    try:
        from robots_dog_msgs.msg import ArcState, ArcChangeFlag
        print("✅ robots_dog_msgs package available")
    except ImportError:
        print("❌ robots_dog_msgs package not available")
        print("   Make sure the workspace is built and sourced")
        return False

    return True


def test_configuration(config_path: str) -> bool:
    """
    Test configuration file parsing.

    Args:
        config_path: Path to configuration file

    Returns:
        True if configuration is valid, False otherwise
    """
    print(f"🔍 Testing configuration: {config_path}")

    try:
        config = ArcStateMachineConfig(config_path)
        print("✅ Configuration parsed successfully")

        # Basic validation
        if len(config.states) == 0:
            print("❌ No states found in configuration")
            return False

        if len(config.flags) == 0:
            print("❌ No transition flags found in configuration")
            return False

        if len(config.transitions) == 0:
            print("❌ No transitions found in configuration")
            return False

        print(f"✅ Configuration valid: {len(config.states)} states, {len(config.transitions)} transitions")
        return True

    except Exception as e:
        print(f"❌ Configuration error: {e}")
        return False


def run_interactive_mode(config_path: str) -> int:
    """
    Run the tool in interactive mode.

    Args:
        config_path: Path to configuration file

    Returns:
        Exit code (0 for success, non-zero for error)
    """
    print(f"🚀 Starting {TOOL_NAME}")

    # Load configuration
    try:
        config = ArcStateMachineConfig(config_path)
        print(f"✅ Configuration loaded: {os.path.basename(config_path)}")
    except Exception as e:
        print(f"❌ Failed to load configuration: {e}")
        return 1

    # Initialize ROS2
    try:
        rclpy.init()
        print("✅ ROS2 initialized")
    except Exception as e:
        print(f"❌ Failed to initialize ROS2: {e}")
        return 1

    try:
        # Create ROS2 interface
        def on_state_change(new_state: int):
            state_name = config.get_state_name(new_state)
            print(f"🔄 State changed to: {state_name} (ID: {new_state})")

        interface = ArcTestInterface(state_callback=on_state_change)
        interface.start()

        # Create and run CLI
        cli = ArcTestCLI(config, interface)
        cli.run()

        return 0

    except KeyboardInterrupt:
        print("\n👋 Interrupted by user")
        return 0
    except ROSInterruptException:
        print("\n👋 ROS interrupted")
        return 0
    except Exception as e:
        print(f"❌ Runtime error: {e}")
        return 1
    finally:
        try:
            if 'interface' in locals():
                interface.stop()
            rclpy.shutdown()
        except Exception:
            pass


def run_non_interactive_mode(config_path: str) -> int:
    """
    Run the tool in non-interactive mode (for testing).

    Args:
        config_path: Path to configuration file

    Returns:
        Exit code (0 for success, non-zero for error)
    """
    print(f"🧪 Testing mode: {TOOL_NAME}")

    # Test configuration
    if not test_configuration(config_path):
        return 1

    # Initialize and test ROS2 interface briefly
    try:
        rclpy.init()

        interface = ArcTestInterface()
        interface.start()

        print("⏳ Testing ROS2 connection (5 seconds)...")
        time.sleep(5)

        # Try to get current state
        current_state = interface.get_current_state()
        if current_state is not None:
            print(f"✅ Received state data: {current_state}")
        else:
            print("⚠️  No state data received (this is normal if ArcStateMachineNode is not running)")

        interface.stop()
        print("✅ Non-interactive test completed")
        return 0

    except Exception as e:
        print(f"❌ Test error: {e}")
        return 1
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


def main() -> int:
    """
    Main entry point.

    Returns:
        Exit code (0 for success, non-zero for error)
    """
    parser = argparse.ArgumentParser(
        description=f"{TOOL_NAME} - Interactive testing for ArcStateMachineNode",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s                     # Use default configuration
  %(prog)s -c custom.conf      # Use custom configuration
  %(prog)s --non-interactive   # Run in test mode
        """
    )

    parser.add_argument(
        '-c', '--config',
        metavar='PATH',
        help='Path to arc_state_machine.conf file'
    )

    parser.add_argument(
        '--non-interactive',
        action='store_true',
        help='Run in non-interactive mode for testing'
    )

    parser.add_argument(
        '-v', '--version',
        action='store_true',
        help='Show version information'
    )

    args = parser.parse_args()

    # Handle version request
    if args.version:
        print_version()
        return 0

    # Validate environment
    if not validate_environment():
        return 1

    # Determine configuration file path
    if args.config:
        config_path = args.config
        if not os.path.exists(config_path):
            print(f"❌ Configuration file not found: {config_path}")
            return 1
    else:
        try:
            config_path = find_default_config()
            print(f"📁 Using default configuration: {os.path.basename(config_path)}")
        except FileNotFoundError as e:
            print(f"❌ {e}")
            return 1

    # Run in appropriate mode
    if args.non_interactive:
        return run_non_interactive_mode(config_path)
    else:
        return run_interactive_mode(config_path)


if __name__ == "__main__":
    exit(main())