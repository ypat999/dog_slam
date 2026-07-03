#!/usr/bin/env python3
"""
Navigo Logging Module

Provides structured logging with both file and console output.
Supports different log levels and log rotation.
"""

import logging
import sys
from datetime import datetime
from pathlib import Path
from typing import Optional

from .config import NavigoConfig


class NavigoLogger:
    """Structured logger for Navigo visualization system"""

    _instance: Optional['NavigoLogger'] = None
    _logger: Optional[logging.Logger] = None

    def __new__(cls):
        """Singleton pattern to ensure only one logger instance"""
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self):
        """Initialize the logger if not already initialized"""
        if self._logger is None:
            self._setup_logger()

    def _setup_logger(self) -> None:
        """Setup the logging configuration"""
        # Create logger
        self._logger = logging.getLogger('navigo_analyze')
        self._logger.setLevel(getattr(logging, NavigoConfig.LOG_LEVEL))

        # Prevent duplicate handlers
        if self._logger.handlers:
            return

        # Create formatters
        detailed_formatter = logging.Formatter(
            fmt='%(asctime)s | %(levelname)-8s | %(name)s | %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )

        simple_formatter = logging.Formatter(
            fmt='%(levelname)s: %(message)s'
        )

        # Console handler (INFO and above, simple format)
        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setLevel(logging.INFO)
        console_handler.setFormatter(simple_formatter)
        self._logger.addHandler(console_handler)

        # File handler (DEBUG and above, detailed format)
        try:
            log_file = self._get_log_file()
            file_handler = logging.FileHandler(log_file, encoding='utf-8')
            file_handler.setLevel(logging.DEBUG)
            file_handler.setFormatter(detailed_formatter)
            self._logger.addHandler(file_handler)
        except Exception as e:
            self._logger.warning(f"Failed to setup file logging: {e}")

    def _get_log_file(self) -> Path:
        """
        Get the log file path for today

        Returns:
            Path: Log file path
        """
        NavigoConfig.LOG_DIR.mkdir(parents=True, exist_ok=True)
        date_str = datetime.now().strftime('%Y%m%d')
        log_file = NavigoConfig.LOG_DIR / NavigoConfig.LOG_FILE_PATTERN.format(date=date_str)
        return log_file

    def debug(self, message: str, *args, **kwargs) -> None:
        """Log debug message"""
        if self._logger:
            self._logger.debug(message, *args, **kwargs)

    def info(self, message: str, *args, **kwargs) -> None:
        """Log info message"""
        if self._logger:
            self._logger.info(message, *args, **kwargs)

    def warning(self, message: str, *args, **kwargs) -> None:
        """Log warning message"""
        if self._logger:
            self._logger.warning(message, *args, **kwargs)

    def error(self, message: str, *args, **kwargs) -> None:
        """Log error message"""
        if self._logger:
            self._logger.error(message, *args, **kwargs)

    def critical(self, message: str, *args, **kwargs) -> None:
        """Log critical message"""
        if self._logger:
            self._logger.critical(message, *args, **kwargs)

    def exception(self, message: str, *args, **kwargs) -> None:
        """Log exception with traceback"""
        if self._logger:
            self._logger.exception(message, *args, **kwargs)

    def section(self, title: str, symbol: str = "=", width: int = 60) -> None:
        """
        Log a section header

        Args:
            title: Section title
            symbol: Symbol to use for the line
            width: Width of the line
        """
        self.info("")
        self.info(symbol * width)
        self.info(title)
        self.info(symbol * width)

    def progress(self, current: int, total: int, prefix: str = "Progress") -> None:
        """
        Log progress information

        Args:
            current: Current item number
            total: Total items
            prefix: Prefix for the progress message
        """
        percentage = (current / total * 100) if total > 0 else 0
        self.info(f"{prefix}: {current}/{total} ({percentage:.1f}%)")

    def cleanup_old_logs(self, days: int = None) -> int:
        """
        Remove log files older than specified days

        Args:
            days: Number of days to retain (default: from config)

        Returns:
            int: Number of files deleted
        """
        if days is None:
            days = NavigoConfig.LOG_RETENTION_DAYS

        deleted_count = 0
        log_dir = NavigoConfig.LOG_DIR

        if not log_dir.exists():
            return 0

        cutoff_time = datetime.now().timestamp() - (days * 24 * 3600)

        for log_file in log_dir.glob("*.log"):
            try:
                if log_file.stat().st_mtime < cutoff_time:
                    log_file.unlink()
                    deleted_count += 1
                    self.debug(f"Deleted old log file: {log_file.name}")
            except Exception as e:
                self.warning(f"Failed to delete log file {log_file.name}: {e}")

        return deleted_count


# Global logger instance
logger = NavigoLogger()
