#!/usr/bin/env python3
"""
Navigo Data Models

Data structures for representing sessions, files, and analysis results.
"""

import json
import re
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional


@dataclass
class DataFile:
    """Represents a single data file"""

    path: Path
    data_type: str  # frontend, corridor, backend, combined
    timestamp: datetime
    session_id: str
    file_size: int = 0
    metadata: Dict = field(default_factory=dict)

    def __post_init__(self):
        """Initialize computed fields"""
        if self.file_size == 0:
            try:
                self.file_size = self.path.stat().st_size
            except:
                self.file_size = 0

    @property
    def name(self) -> str:
        """Get file name"""
        return self.path.name

    def load_json(self) -> Optional[Dict]:
        """
        Load JSON data from file

        Returns:
            Dict or None if failed
        """
        try:
            with open(self.path, 'r', encoding='utf-8') as f:
                return json.load(f)
        except Exception:
            return None

    def __str__(self) -> str:
        return f"DataFile({self.data_type}, {self.name})"

    def __repr__(self) -> str:
        return self.__str__()


@dataclass
class FileGroup:
    """Represents a group of related files (frontend + corridor + backend)"""

    session_id: str
    timestamp: datetime
    frontend: Optional[DataFile] = None
    corridor: Optional[DataFile] = None
    backend: Optional[DataFile] = None

    @property
    def is_complete(self) -> bool:
        """Check if group has all three types"""
        return all([self.frontend, self.corridor, self.backend])

    @property
    def file_count(self) -> int:
        """Count non-None files"""
        return sum([
            self.frontend is not None,
            self.corridor is not None,
            self.backend is not None
        ])

    @property
    def files(self) -> List[DataFile]:
        """Get list of all files in group"""
        files = []
        if self.frontend:
            files.append(self.frontend)
        if self.corridor:
            files.append(self.corridor)
        if self.backend:
            files.append(self.backend)
        return files

    def __str__(self) -> str:
        return f"FileGroup({self.session_id}, files={self.file_count})"


@dataclass
class Session:
    """Represents a planning session with multiple data files"""

    session_id: str
    timestamp: datetime
    frontend_files: List[DataFile] = field(default_factory=list)
    corridor_files: List[DataFile] = field(default_factory=list)
    backend_files: List[DataFile] = field(default_factory=list)
    combined_files: List[DataFile] = field(default_factory=list)

    @property
    def total_files(self) -> int:
        """Get total number of files"""
        return (len(self.frontend_files) + len(self.corridor_files) +
                len(self.backend_files) + len(self.combined_files))

    @property
    def all_files(self) -> List[DataFile]:
        """Get all files as a single list"""
        return (self.frontend_files + self.corridor_files +
                self.backend_files + self.combined_files)

    @property
    def file_summary(self) -> Dict[str, int]:
        """Get file count by type"""
        return {
            'frontend': len(self.frontend_files),
            'corridor': len(self.corridor_files),
            'backend': len(self.backend_files),
            'combined': len(self.combined_files),
            'total': self.total_files
        }

    def get_file_groups(self) -> List[FileGroup]:
        """
        Match related files into groups

        Returns:
            List of FileGroup objects
        """
        groups = []

        # Simple strategy: match by timestamp proximity
        # Files within 5 seconds are considered part of same group
        time_threshold = 5  # seconds

        # Start with frontend files as anchors
        for frontend in self.frontend_files:
            group = FileGroup(
                session_id=self.session_id,
                timestamp=frontend.timestamp,
                frontend=frontend
            )

            # Find closest corridor file
            closest_corridor = None
            min_corridor_diff = float('inf')

            for corridor in self.corridor_files:
                time_diff = abs((corridor.timestamp - frontend.timestamp).total_seconds())
                # Prefer non-segment files
                if '_seg_' not in corridor.name and time_diff < min_corridor_diff:
                    closest_corridor = corridor
                    min_corridor_diff = time_diff

            if closest_corridor and min_corridor_diff <= time_threshold:
                group.corridor = closest_corridor

            # Find closest backend file
            closest_backend = None
            min_backend_diff = float('inf')

            for backend in self.backend_files:
                time_diff = abs((backend.timestamp - frontend.timestamp).total_seconds())
                if time_diff < min_backend_diff:
                    closest_backend = backend
                    min_backend_diff = time_diff

            if closest_backend and min_backend_diff <= time_threshold:
                group.backend = closest_backend

            groups.append(group)

        return groups

    def __str__(self) -> str:
        return f"Session({self.session_id}, {self.total_files} files)"


@dataclass
class VisualizationResult:
    """Result of visualization operation"""

    success: bool
    data_type: str
    input_file: Path
    output_file: Optional[Path] = None
    error_message: Optional[str] = None
    execution_time: float = 0.0

    def __str__(self) -> str:
        status = "✅" if self.success else "❌"
        return f"{status} {self.data_type}: {self.input_file.name}"


@dataclass
class SessionAnalysisResult:
    """Result of analyzing a session"""

    session: Session
    visualizations: List[VisualizationResult] = field(default_factory=list)
    start_time: Optional[datetime] = None
    end_time: Optional[datetime] = None

    @property
    def success_count(self) -> int:
        """Count successful visualizations"""
        return sum(1 for v in self.visualizations if v.success)

    @property
    def total_count(self) -> int:
        """Total visualizations attempted"""
        return len(self.visualizations)

    @property
    def success_rate(self) -> float:
        """Calculate success rate"""
        return (self.success_count / self.total_count * 100) if self.total_count > 0 else 0

    @property
    def execution_time(self) -> float:
        """Total execution time in seconds"""
        if self.start_time and self.end_time:
            return (self.end_time - self.start_time).total_seconds()
        return 0.0

    @property
    def output_files(self) -> List[Path]:
        """Get list of generated output files"""
        return [v.output_file for v in self.visualizations
                if v.success and v.output_file]

    def __str__(self) -> str:
        return (f"SessionAnalysisResult({self.session.session_id}, "
                f"success={self.success_count}/{self.total_count})")
