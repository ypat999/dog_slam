#!/usr/bin/env python3
"""
Navigo Data Collector Module

Provides direct access to debug data without copying files.
Scans, classifies, and organizes data files by session.
"""

import json
import re
from collections import defaultdict
from datetime import datetime, timedelta
from pathlib import Path
from typing import Dict, List, Optional

from .config import NavigoConfig
from .logger import logger
from .models import DataFile, Session


class DataCollector:
    """Collects and organizes data files from source directory"""

    def __init__(self, source_dir: Optional[Path] = None):
        """
        Initialize data collector

        Args:
            source_dir: Source directory (default: from config)
        """
        self.source_dir = source_dir or NavigoConfig.get_data_source()
        logger.debug(f"DataCollector initialized with source: {self.source_dir}")

    def scan_data_files(self, limit_days: Optional[int] = None) -> Dict[str, List[DataFile]]:
        """
        Scan and classify all JSON files in source directory

        Args:
            limit_days: Only include files from last N days (None = all)

        Returns:
            Dict mapping data type to list of DataFile objects
        """
        logger.info(f"Scanning data files from: {self.source_dir}")

        if not self.source_dir.exists():
            logger.error(f"Data source directory does not exist: {self.source_dir}")
            return {'frontend': [], 'corridor': [], 'backend': [], 'combined': [], 'unknown': []}

        # Calculate cutoff time if limit specified
        cutoff_time = None
        if limit_days is not None:
            cutoff_time = datetime.now().timestamp() - (limit_days * 24 * 3600)
            logger.debug(f"Limiting to files from last {limit_days} days")

        # Find all JSON files
        json_files = list(self.source_dir.rglob("*.json"))
        logger.debug(f"Found {len(json_files)} JSON files")

        # Classify files
        classified_files = {
            'frontend': [],
            'corridor': [],
            'backend': [],
            'combined': [],
            'unknown': []
        }

        for file_path in json_files:
            try:
                # Skip if too old
                if cutoff_time and file_path.stat().st_mtime < cutoff_time:
                    continue

                # Classify file
                data_type = self._classify_file(file_path)
                timestamp = self._extract_timestamp(file_path)
                session_id = self._extract_session_id(file_path, timestamp)

                data_file = DataFile(
                    path=file_path,
                    data_type=data_type,
                    timestamp=timestamp,
                    session_id=session_id
                )

                classified_files[data_type].append(data_file)
                logger.debug(f"Classified: {file_path.name} -> {data_type}")

            except Exception as e:
                logger.warning(f"Failed to process file {file_path.name}: {e}")
                continue

        # Log summary
        total_files = sum(len(files) for dtype, files in classified_files.items()
                         if dtype != 'unknown')
        logger.info(f"Classified {total_files} files:")
        for dtype, files in classified_files.items():
            if files:
                logger.info(f"  {dtype}: {len(files)} files")

        return classified_files

    def get_sessions(self, limit_days: Optional[int] = None) -> List[Session]:
        """
        Get all sessions from data files

        Args:
            limit_days: Only include sessions from last N days

        Returns:
            List of Session objects, sorted by timestamp (newest first)
        """
        logger.info("Organizing files into sessions...")

        # Scan files
        classified_files = self.scan_data_files(limit_days)

        # Group by session ID
        sessions_dict = defaultdict(lambda: {
            'frontend': [],
            'corridor': [],
            'backend': [],
            'combined': [],
            'timestamp': None
        })

        for data_type, files in classified_files.items():
            if data_type == 'unknown':
                continue

            for data_file in files:
                session_data = sessions_dict[data_file.session_id]
                session_data[data_type].append(data_file)

                # Update session timestamp (use most recent)
                if session_data['timestamp'] is None or data_file.timestamp > session_data['timestamp']:
                    session_data['timestamp'] = data_file.timestamp

        # Create Session objects
        sessions = []
        for session_id, session_data in sessions_dict.items():
            if session_data['timestamp'] is None:
                continue

            session = Session(
                session_id=session_id,
                timestamp=session_data['timestamp'],
                frontend_files=session_data['frontend'],
                corridor_files=session_data['corridor'],
                backend_files=session_data['backend'],
                combined_files=session_data['combined']
            )
            sessions.append(session)

        # Sort by timestamp (newest first)
        sessions.sort(key=lambda s: s.timestamp, reverse=True)

        logger.info(f"Found {len(sessions)} sessions")
        return sessions

    def get_latest_session(self) -> Optional[Session]:
        """
        Get the most recent session

        Returns:
            Session object or None if no sessions found
        """
        sessions = self.get_sessions(limit_days=NavigoConfig.DATA_RETENTION_DAYS)

        if not sessions:
            logger.warning("No sessions found")
            return None

        latest = sessions[0]
        logger.info(f"Latest session: {latest.session_id} ({latest.total_files} files)")
        return latest

    def get_session_by_id(self, session_id: str) -> Optional[Session]:
        """
        Get a specific session by ID

        Args:
            session_id: Session ID to find

        Returns:
            Session object or None if not found
        """
        sessions = self.get_sessions()

        for session in sessions:
            if session.session_id == session_id:
                logger.info(f"Found session: {session_id}")
                return session

        logger.warning(f"Session not found: {session_id}")
        return None

    def _classify_file(self, file_path: Path) -> str:
        """
        Classify file by data type based on filename pattern

        Args:
            file_path: Path to file

        Returns:
            Data type string (frontend, corridor, backend, combined, unknown)
        """
        file_name = file_path.name.lower()

        for data_type, pattern in NavigoConfig.DATA_TYPE_PATTERNS.items():
            if re.match(pattern, file_name, re.IGNORECASE):
                return data_type

        return 'unknown'

    def _extract_timestamp(self, file_path: Path) -> datetime:
        """
        Extract timestamp from file

        Priority:
        1. Timestamp from filename (YYYYMMDD_HHMMSS)
        2. Timestamp from JSON content
        3. File modification time

        Args:
            file_path: Path to file

        Returns:
            datetime object
        """
        file_name = file_path.name

        # Try to extract from filename
        timestamp_match = re.search(r'(\d{8}_\d{6})', file_name)
        if timestamp_match:
            try:
                timestamp_str = timestamp_match.group(1)
                return datetime.strptime(timestamp_str, '%Y%m%d_%H%M%S')
            except ValueError:
                pass

        # Try to extract from JSON content
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
                timestamp_str = data.get('timestamp', '')

                if timestamp_str:
                    # Try multiple formats
                    formats = ['%Y-%m-%d %H:%M:%S', '%Y%m%d_%H%M%S']
                    for fmt in formats:
                        try:
                            return datetime.strptime(timestamp_str, fmt)
                        except ValueError:
                            continue
        except:
            pass

        # Fallback to file modification time
        return datetime.fromtimestamp(file_path.stat().st_mtime)

    def _extract_session_id(self, file_path: Path, timestamp: datetime) -> str:
        """
        Extract or generate session ID

        Session ID is based on time window (1 hour by default)

        Args:
            file_path: Path to file
            timestamp: File timestamp

        Returns:
            Session ID string (format: YYYYMMDD_HH)
        """
        # Use timestamp truncated to hour as session ID
        return timestamp.strftime('%Y%m%d_%H')

    def get_statistics(self) -> Dict:
        """
        Get statistics about data files

        Returns:
            Dict with statistics
        """
        sessions = self.get_sessions()

        stats = {
            'total_sessions': len(sessions),
            'total_files': sum(s.total_files for s in sessions),
            'by_type': defaultdict(int),
            'oldest_session': None,
            'newest_session': None,
        }

        if sessions:
            stats['newest_session'] = sessions[0].session_id
            stats['oldest_session'] = sessions[-1].session_id

            for session in sessions:
                for dtype, count in session.file_summary.items():
                    if dtype != 'total':
                        stats['by_type'][dtype] += count

        return dict(stats)
