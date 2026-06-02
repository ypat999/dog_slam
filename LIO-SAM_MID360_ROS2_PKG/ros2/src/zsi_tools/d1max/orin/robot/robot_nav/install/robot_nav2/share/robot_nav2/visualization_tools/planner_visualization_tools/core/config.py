#!/usr/bin/env python3
"""
Navigo Configuration Module

Centralized configuration for all paths, settings, and constants.
Provides single source of truth for the visualization system.
"""

import os
from pathlib import Path
from typing import Dict, Optional


class NavigoConfig:
    """Centralized configuration for Navigo visualization system"""

    # ==================== Data Source Configuration ====================

    # Primary data source (C++ planner output)
    DATA_SOURCE_DIR = Path("/tmp/zsibot/navigo_debug_data")

    # Alternative data source (if primary not available)
    ALT_DATA_SOURCE_DIR: Optional[Path] = None

    # ==================== Tools Directory Configuration ====================

    # Base directory for visualization tools (auto-detected)
    TOOLS_BASE_DIR = Path(__file__).parent.parent.resolve()

    # Visualization scripts directory
    VISUALIZATION_DIR = TOOLS_BASE_DIR / "visualization"

    # Utils directory
    UTILS_DIR = TOOLS_BASE_DIR / "utils"

    # Config directory
    CONFIG_DIR = TOOLS_BASE_DIR / "config"

    # ==================== Output Configuration ====================

    # Main output directory
    OUTPUT_DIR = TOOLS_BASE_DIR / "output"

    # Output subdirectories by type
    OUTPUT_FRONTEND_DIR = OUTPUT_DIR / "frontend"
    OUTPUT_CORRIDOR_DIR = OUTPUT_DIR / "corridor"
    OUTPUT_BACKEND_DIR = OUTPUT_DIR / "backend"
    OUTPUT_COMBINED_DIR = OUTPUT_DIR / "combined"

    # ==================== Data Retention Configuration ====================

    # Default data retention period (days)
    DATA_RETENTION_DAYS = 1

    # Grace period before cleanup (hours)
    CLEANUP_GRACE_HOURS = 2

    # ==================== Visualization Scripts Mapping ====================

    VISUALIZERS = {
        'frontend': 'visualize_frontend_search.py',
        'corridor': 'visualize_corridor.py',
        'backend': 'visualize_backend_optimization.py',
        'combined': 'visualize_combined.py'
    }

    # ==================== Data Type Patterns ====================

    DATA_TYPE_PATTERNS = {
        'frontend': r'.*frontend.*\.json$',
        'corridor': r'.*(corridor|corridor_seg).*\.json$',
        'backend': r'.*backend.*\.json$',
        'combined': r'.*combined.*\.json$',
    }

    # ==================== Session Configuration ====================

    # Session grouping time window (minutes)
    SESSION_TIME_WINDOW = 60  # 1 hour

    # Maximum files per session for visualization
    MAX_FILES_PER_SESSION = 100

    # ==================== Performance Configuration ====================

    # Enable parallel processing
    ENABLE_PARALLEL = True

    # Maximum parallel workers (None = auto-detect)
    MAX_WORKERS: Optional[int] = None

    # Visualization timeout per file (seconds)
    VISUALIZATION_TIMEOUT = 300  # 5 minutes

    # ==================== Logging Configuration ====================

    # Log directory
    LOG_DIR = TOOLS_BASE_DIR / "logs"

    # Log file name pattern
    LOG_FILE_PATTERN = "navigo_analyze_{date}.log"

    # Log level (DEBUG, INFO, WARNING, ERROR)
    LOG_LEVEL = "INFO"

    # Log retention (days)
    LOG_RETENTION_DAYS = 7

    # ==================== CLI Configuration ====================

    # Default CLI behavior
    AUTO_APPROVE = False  # Require user confirmation by default
    VERBOSE = False       # Verbose output
    DRY_RUN = False       # Dry-run mode by default

    # ==================== Helper Methods ====================

    @classmethod
    def get_data_source(cls) -> Path:
        """
        Get the primary or alternative data source directory

        Returns:
            Path: Data source directory

        Raises:
            FileNotFoundError: If no valid data source found
        """
        if cls.DATA_SOURCE_DIR.exists():
            return cls.DATA_SOURCE_DIR
        elif cls.ALT_DATA_SOURCE_DIR and cls.ALT_DATA_SOURCE_DIR.exists():
            return cls.ALT_DATA_SOURCE_DIR
        else:
            raise FileNotFoundError(
                f"Data source directory not found: {cls.DATA_SOURCE_DIR}\n"
                f"Alternative: {cls.ALT_DATA_SOURCE_DIR or 'Not configured'}"
            )

    @classmethod
    def get_visualizer_path(cls, data_type: str) -> Path:
        """
        Get the full path to a visualization script

        Args:
            data_type: Type of data (frontend, corridor, backend, combined)

        Returns:
            Path: Full path to visualization script

        Raises:
            ValueError: If data type is invalid
            FileNotFoundError: If script not found
        """
        if data_type not in cls.VISUALIZERS:
            raise ValueError(
                f"Invalid data type: {data_type}. "
                f"Valid types: {list(cls.VISUALIZERS.keys())}"
            )

        script_name = cls.VISUALIZERS[data_type]
        script_path = cls.VISUALIZATION_DIR / script_name

        if not script_path.exists():
            raise FileNotFoundError(f"Visualization script not found: {script_path}")

        return script_path

    @classmethod
    def get_output_dir(cls, data_type: str) -> Path:
        """
        Get the output directory for a specific data type

        Args:
            data_type: Type of data (frontend, corridor, backend, combined)

        Returns:
            Path: Output directory for the data type
        """
        output_dirs = {
            'frontend': cls.OUTPUT_FRONTEND_DIR,
            'corridor': cls.OUTPUT_CORRIDOR_DIR,
            'backend': cls.OUTPUT_BACKEND_DIR,
            'combined': cls.OUTPUT_COMBINED_DIR,
        }
        return output_dirs.get(data_type, cls.OUTPUT_DIR)

    @classmethod
    def ensure_directories(cls) -> None:
        """Create all necessary directories if they don't exist"""
        directories = [
            cls.OUTPUT_DIR,
            cls.OUTPUT_FRONTEND_DIR,
            cls.OUTPUT_CORRIDOR_DIR,
            cls.OUTPUT_BACKEND_DIR,
            cls.OUTPUT_COMBINED_DIR,
            cls.LOG_DIR,
        ]

        for directory in directories:
            directory.mkdir(parents=True, exist_ok=True)

    @classmethod
    def validate_configuration(cls) -> bool:
        """
        Validate the configuration

        Returns:
            bool: True if configuration is valid

        Raises:
            RuntimeError: If configuration is invalid
        """
        errors = []

        # Check data source
        try:
            cls.get_data_source()
        except FileNotFoundError as e:
            errors.append(str(e))

        # Check visualization directory
        if not cls.VISUALIZATION_DIR.exists():
            errors.append(f"Visualization directory not found: {cls.VISUALIZATION_DIR}")

        # Check visualization scripts
        for data_type, script_name in cls.VISUALIZERS.items():
            script_path = cls.VISUALIZATION_DIR / script_name
            if not script_path.exists():
                errors.append(f"Visualization script missing: {script_path}")

        if errors:
            raise RuntimeError(
                "Configuration validation failed:\n" +
                "\n".join(f"  - {err}" for err in errors)
            )

        return True

    @classmethod
    def get_config_summary(cls) -> Dict[str, str]:
        """
        Get a summary of current configuration

        Returns:
            Dict: Configuration summary
        """
        return {
            "Data Source": str(cls.get_data_source()),
            "Tools Base": str(cls.TOOLS_BASE_DIR),
            "Output Dir": str(cls.OUTPUT_DIR),
            "Retention Days": str(cls.DATA_RETENTION_DAYS),
            "Parallel Processing": str(cls.ENABLE_PARALLEL),
            "Log Level": cls.LOG_LEVEL,
        }


# Initialize directories on module import
try:
    NavigoConfig.ensure_directories()
except Exception as e:
    import warnings
    warnings.warn(f"Failed to create directories: {e}")
