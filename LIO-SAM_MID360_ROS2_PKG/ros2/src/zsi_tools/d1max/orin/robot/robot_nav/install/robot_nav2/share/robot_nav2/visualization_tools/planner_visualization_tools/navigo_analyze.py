#!/usr/bin/env python3
"""
Navigo Unified Analysis Tool

Single command-line tool for analyzing Navigo planning data.
Replaces the multi-step manual workflow with a unified interface.

Usage:
    navigo-analyze --latest                      # Analyze latest session
    navigo-analyze --session 20241218_15         # Analyze specific session
    navigo-analyze --list                        # List available sessions
    navigo-analyze --cleanup --days 1            # Cleanup old data
    navigo-analyze --auto-cleanup --latest       # Cleanup + analyze
    navigo-analyze --info                        # Show storage info
"""

import argparse
import sys
from pathlib import Path

# Add core module to path
sys.path.insert(0, str(Path(__file__).parent))

from core import (DataCleaner, DataCollector, DataProcessor, NavigoConfig,
                  VisualizationOrchestrator, logger)


class NavigoAnalyzeCLI:
    """Main CLI application for Navigo data analysis"""

    def __init__(self):
        """Initialize CLI application"""
        self.collector = DataCollector()
        self.processor = DataProcessor(self.collector)
        self.visualizer = VisualizationOrchestrator(self.processor)
        self.cleaner = DataCleaner()

    def analyze_latest(self, auto_cleanup: bool = False) -> int:
        """
        Analyze the most recent session

        Args:
            auto_cleanup: Whether to cleanup old data first

        Returns:
            Exit code (0 = success, 1 = error)
        """
        try:
            # Optional cleanup
            if auto_cleanup:
                logger.info("🧹 Running automatic cleanup...")
                self.cleaner.cleanup(dry_run=False)
                logger.info("")

            # Get latest session
            logger.info("🔍 Finding latest session...")
            session = self.collector.get_latest_session()

            if not session:
                logger.error("❌ No sessions found")
                logger.info("\n💡 Tips:")
                logger.info("  1. Check if data directory exists: /tmp/zsibot/navigo_debug_data")
                logger.info("  2. Run the planner to generate debug data")
                logger.info("  3. Verify data retention period (currently: {} days)".format(
                    NavigoConfig.DATA_RETENTION_DAYS))
                return 1

            # Show session summary
            logger.info("")
            logger.info(self.processor.get_session_summary(session))
            logger.info("")

            # Analyze session
            result = self.visualizer.visualize_session(session)

            # Report results
            if result.success_count > 0:
                logger.info(f"\n✅ Analysis complete! Generated {result.success_count} visualizations")
                logger.info(f"📁 Output directory: {NavigoConfig.OUTPUT_DIR}")
                return 0
            else:
                logger.error(f"\n❌ Analysis failed: no successful visualizations")
                return 1

        except Exception as e:
            logger.exception(f"❌ Error during analysis: {e}")
            return 1

    def analyze_session(self, session_id: str) -> int:
        """
        Analyze a specific session

        Args:
            session_id: Session ID to analyze

        Returns:
            Exit code
        """
        try:
            logger.info(f"🔍 Finding session: {session_id}")
            session = self.collector.get_session_by_id(session_id)

            if not session:
                logger.error(f"❌ Session not found: {session_id}")
                logger.info("\n💡 Use --list to see available sessions")
                return 1

            # Show session summary
            logger.info("")
            logger.info(self.processor.get_session_summary(session))
            logger.info("")

            # Analyze session
            result = self.visualizer.visualize_session(session)

            if result.success_count > 0:
                logger.info(f"\n✅ Analysis complete! Generated {result.success_count} visualizations")
                logger.info(f"📁 Output directory: {NavigoConfig.OUTPUT_DIR}")
                return 0
            else:
                logger.error(f"\n❌ Analysis failed: no successful visualizations")
                return 1

        except Exception as e:
            logger.exception(f"❌ Error during analysis: {e}")
            return 1

    def list_sessions(self, limit_days: int = None) -> int:
        """
        List all available sessions

        Args:
            limit_days: Limit to sessions from last N days

        Returns:
            Exit code
        """
        try:
            logger.section("Available Sessions")

            sessions = self.collector.get_sessions(limit_days)

            if not sessions:
                logger.info("No sessions found")
                return 0

            logger.info(f"Found {len(sessions)} sessions:\n")

            for i, session in enumerate(sessions, 1):
                timestamp_str = session.timestamp.strftime('%Y-%m-%d %H:%M:%S')
                logger.info(f"{i}. Session: {session.session_id}")
                logger.info(f"   Time:  {timestamp_str}")
                logger.info(f"   Files: {session.total_files} total")
                logger.info(f"          {session.file_summary['frontend']} frontend, "
                          f"{session.file_summary['corridor']} corridor, "
                          f"{session.file_summary['backend']} backend")

                # Data quality
                quality = self.processor.analyze_data_quality(session)
                logger.info(f"   Quality: {quality['data_quality'].upper()} "
                          f"(completeness: {quality['completeness']:.0f}%)")
                logger.info("")

            logger.info("💡 Use --session <id> to analyze a specific session")
            logger.info("💡 Use --latest to analyze the most recent session")

            return 0

        except Exception as e:
            logger.exception(f"❌ Error listing sessions: {e}")
            return 1

    def cleanup_data(self, days: int, dry_run: bool = False) -> int:
        """
        Cleanup old data files

        Args:
            days: Days to retain
            dry_run: If True, show what would be deleted without deleting

        Returns:
            Exit code
        """
        try:
            cleaner = DataCleaner(retention_days=days)
            result = cleaner.cleanup(dry_run=dry_run)

            if result['deleted'] > 0 or dry_run:
                return 0
            else:
                logger.info("✅ No cleanup needed")
                return 0

        except Exception as e:
            logger.exception(f"❌ Error during cleanup: {e}")
            return 1

    def show_info(self) -> int:
        """
        Show storage and configuration information

        Returns:
            Exit code
        """
        try:
            logger.section("Navigo Analysis Tool - System Information")

            # Configuration
            logger.info("\n📋 Configuration:")
            config = NavigoConfig.get_config_summary()
            for key, value in config.items():
                logger.info(f"  {key}: {value}")

            # Storage info
            logger.info("\n💾 Storage Information:")
            storage = self.cleaner.get_storage_info()
            logger.info(f"  Total Files: {storage['total_files']}")
            logger.info(f"  Total Size:  {storage['total_size_mb']:.2f} MB")
            if storage['oldest_file']:
                logger.info(f"  Oldest File: {storage['oldest_file'].strftime('%Y-%m-%d %H:%M:%S')}")
            if storage['newest_file']:
                logger.info(f"  Newest File: {storage['newest_file'].strftime('%Y-%m-%d %H:%M:%S')}")

            # Statistics
            logger.info("\n📊 Statistics:")
            stats = self.collector.get_statistics()
            logger.info(f"  Sessions:    {stats['total_sessions']}")
            logger.info(f"  Total Files: {stats['total_files']}")
            logger.info(f"  By Type:")
            for dtype, count in stats['by_type'].items():
                logger.info(f"    {dtype}: {count}")

            logger.info("")
            logger.info("💡 Use --list to see all sessions")
            logger.info("💡 Use --latest to analyze the most recent session")

            return 0

        except Exception as e:
            logger.exception(f"❌ Error showing info: {e}")
            return 1

    def validate_system(self) -> int:
        """
        Validate system configuration

        Returns:
            Exit code
        """
        try:
            logger.section("System Validation")

            try:
                NavigoConfig.validate_configuration()
                logger.info("✅ Configuration valid")
                return 0
            except RuntimeError as e:
                logger.error(f"❌ Configuration invalid:\n{e}")
                return 1

        except Exception as e:
            logger.exception(f"❌ Error during validation: {e}")
            return 1


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(
        description="Navigo Unified Analysis Tool - Analyze and visualize planning data",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --latest                    # Analyze latest session
  %(prog)s --latest --auto-cleanup     # Cleanup old data, then analyze
  %(prog)s --session 20241218_15       # Analyze specific session
  %(prog)s --list                      # List all sessions
  %(prog)s --list --days 7             # List sessions from last 7 days
  %(prog)s --cleanup --days 1          # Delete data older than 1 day
  %(prog)s --cleanup --days 1 --dry-run  # Show what would be deleted
  %(prog)s --info                      # Show system information
  %(prog)s --validate                  # Validate configuration

For more information, see the documentation in docs/USER_GUIDE.md
        """
    )

    # Analysis modes
    analysis_group = parser.add_mutually_exclusive_group()
    analysis_group.add_argument(
        '--latest',
        action='store_true',
        help='Analyze the most recent session'
    )
    analysis_group.add_argument(
        '--session',
        metavar='ID',
        help='Analyze specific session (e.g., 20241218_15)'
    )
    analysis_group.add_argument(
        '--list',
        action='store_true',
        help='List all available sessions'
    )
    analysis_group.add_argument(
        '--cleanup',
        action='store_true',
        help='Clean up old data files'
    )
    analysis_group.add_argument(
        '--info',
        action='store_true',
        help='Show system information and statistics'
    )
    analysis_group.add_argument(
        '--validate',
        action='store_true',
        help='Validate system configuration'
    )

    # Options
    parser.add_argument(
        '--auto-cleanup',
        action='store_true',
        help='Automatically cleanup old data before analysis'
    )
    parser.add_argument(
        '--days',
        type=int,
        metavar='N',
        help='Number of days to retain data (for --cleanup or --list)'
    )
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Show what would be deleted without actually deleting (for --cleanup)'
    )
    parser.add_argument(
        '--verbose',
        '-v',
        action='store_true',
        help='Enable verbose output'
    )

    args = parser.parse_args()

    # Handle no arguments
    if len(sys.argv) == 1:
        parser.print_help()
        return 0

    # Set verbose mode
    if args.verbose:
        NavigoConfig.LOG_LEVEL = "DEBUG"

    # Initialize CLI
    try:
        cli = NavigoAnalyzeCLI()
    except Exception as e:
        logger.error(f"❌ Failed to initialize: {e}")
        logger.info("\n💡 Try running with --validate to check configuration")
        return 1

    # Execute command
    if args.latest:
        return cli.analyze_latest(auto_cleanup=args.auto_cleanup)

    elif args.session:
        return cli.analyze_session(args.session)

    elif args.list:
        return cli.list_sessions(limit_days=args.days)

    elif args.cleanup:
        days = args.days or NavigoConfig.DATA_RETENTION_DAYS
        return cli.cleanup_data(days=days, dry_run=args.dry_run)

    elif args.info:
        return cli.show_info()

    elif args.validate:
        return cli.validate_system()

    else:
        parser.print_help()
        return 0


if __name__ == "__main__":
    sys.exit(main())
