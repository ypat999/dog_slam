"""
Navigo Visualization Core Modules

This package contains the core functionality for the unified Navigo visualization system.
It provides direct data access, processing, and orchestration capabilities.

Modules:
    config: Centralized configuration management
    logger: Structured logging system
    data_collector: Direct access to debug data
    processor: In-memory data organization
    visualizer: Visualization orchestration
    cleaner: Data lifecycle management
    models: Data structures and models
"""

from .cleaner import DataCleaner
from .config import NavigoConfig
from .data_collector import DataCollector
from .logger import NavigoLogger, logger
from .models import (DataFile, FileGroup, Session, SessionAnalysisResult,
                     VisualizationResult)
from .processor import DataProcessor
from .visualizer import VisualizationOrchestrator

__version__ = "1.0.0"
__all__ = [
    # Config and Logger
    "NavigoConfig",
    "NavigoLogger",
    "logger",
    # Core Components
    "DataCollector",
    "DataProcessor",
    "VisualizationOrchestrator",
    "DataCleaner",
    # Models
    "DataFile",
    "FileGroup",
    "Session",
    "VisualizationResult",
    "SessionAnalysisResult",
]
