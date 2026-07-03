#!/usr/bin/env python3
"""
Navigo Data Processor Module

Processes and validates data sessions without moving files.
Performs in-memory organization and quality checks.
"""

import json
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from .config import NavigoConfig
from .data_collector import DataCollector
from .logger import logger
from .models import DataFile, FileGroup, Session


class DataProcessor:
    """Processes data sessions for visualization"""

    def __init__(self, collector: Optional[DataCollector] = None):
        """
        Initialize data processor

        Args:
            collector: DataCollector instance (creates new if None)
        """
        self.collector = collector or DataCollector()
        logger.debug("DataProcessor initialized")

    def validate_session(self, session: Session) -> Tuple[bool, List[str]]:
        """
        Validate session data quality

        Args:
            session: Session to validate

        Returns:
            Tuple of (is_valid, list of warnings/errors)
        """
        logger.debug(f"Validating session: {session.session_id}")

        issues = []
        is_valid = True

        # Check if session has any files
        if session.total_files == 0:
            issues.append("Session has no data files")
            is_valid = False
            return is_valid, issues

        # Check file counts
        summary = session.file_summary
        if summary['frontend'] == 0:
            issues.append("No frontend search data")

        if summary['corridor'] == 0:
            issues.append("No corridor generation data")

        if summary['backend'] == 0:
            issues.append("No backend optimization data")

        # Validate JSON structure of each file
        for data_file in session.all_files:
            try:
                data = data_file.load_json()
                if data is None:
                    issues.append(f"Failed to parse JSON: {data_file.name}")
                    continue

                # Basic structure validation
                if data_file.data_type == 'frontend':
                    if 'search_result' not in data:
                        issues.append(f"Missing 'search_result' in frontend file: {data_file.name}")

                elif data_file.data_type == 'corridor':
                    if 'generation_result' not in data:
                        issues.append(f"Missing 'generation_result' in corridor file: {data_file.name}")

                elif data_file.data_type == 'backend':
                    if 'optimization_result' not in data:
                        issues.append(f"Missing 'optimization_result' in backend file: {data_file.name}")

            except Exception as e:
                issues.append(f"Validation error for {data_file.name}: {e}")

        # Check file size anomalies
        for data_file in session.all_files:
            if data_file.file_size < 100:  # Suspiciously small
                issues.append(f"File may be empty or corrupted: {data_file.name} ({data_file.file_size} bytes)")

        logger.debug(f"Validation complete: {len(issues)} issues found")

        return is_valid, issues

    def match_file_groups(self, session: Session) -> List[FileGroup]:
        """
        Match related frontend/corridor/backend files into groups

        Uses the Session's get_file_groups() method with additional validation

        Args:
            session: Session to process

        Returns:
            List of FileGroup objects
        """
        logger.debug(f"Matching file groups for session: {session.session_id}")

        groups = session.get_file_groups()

        # Log statistics
        complete_groups = [g for g in groups if g.is_complete]
        logger.info(f"Found {len(groups)} file groups ({len(complete_groups)} complete)")

        for i, group in enumerate(groups, 1):
            logger.debug(f"  Group {i}: {group.file_count}/3 files")

        return groups

    def prepare_for_visualization(self, session: Session) -> Dict[str, List[Path]]:
        """
        Prepare file lists for visualization

        Args:
            session: Session to prepare

        Returns:
            Dict mapping data type to list of file paths
        """
        logger.debug(f"Preparing visualization data for session: {session.session_id}")

        result = {
            'frontend': [f.path for f in session.frontend_files],
            'corridor': [f.path for f in session.corridor_files],
            'backend': [f.path for f in session.backend_files],
            'combined_groups': []
        }

        # Add combined visualization groups
        groups = self.match_file_groups(session)
        complete_groups = [g for g in groups if g.is_complete]

        result['combined_groups'] = [
            {
                'frontend': g.frontend.path,
                'corridor': g.corridor.path,
                'backend': g.backend.path
            }
            for g in complete_groups
        ]

        logger.info(f"Prepared {len(result['frontend'])} frontend, "
                   f"{len(result['corridor'])} corridor, "
                   f"{len(result['backend'])} backend files, "
                   f"{len(result['combined_groups'])} combined groups")

        return result

    def filter_corridor_files(self, corridor_files: List[DataFile]) -> List[DataFile]:
        """
        Filter corridor files, preferring non-segment files

        Args:
            corridor_files: List of corridor files

        Returns:
            Filtered list of corridor files
        """
        # Separate segment and non-segment files
        non_segment = [f for f in corridor_files if '_seg_' not in f.name]
        segment = [f for f in corridor_files if '_seg_' in f.name]

        # Prefer non-segment files
        if non_segment:
            logger.debug(f"Using {len(non_segment)} non-segment corridor files")
            return non_segment
        else:
            logger.debug(f"Using {len(segment)} segment corridor files")
            return segment

    def analyze_data_quality(self, session: Session) -> Dict:
        """
        Analyze data quality metrics for a session

        Args:
            session: Session to analyze

        Returns:
            Dict with quality metrics
        """
        logger.debug(f"Analyzing data quality for session: {session.session_id}")

        metrics = {
            'total_files': session.total_files,
            'file_counts': session.file_summary,
            'completeness': 0.0,
            'data_quality': 'unknown',
            'issues': []
        }

        # Validate session
        is_valid, issues = self.validate_session(session)
        metrics['issues'] = issues

        # Calculate completeness score
        # Score based on having data from all three stages
        has_frontend = len(session.frontend_files) > 0
        has_corridor = len(session.corridor_files) > 0
        has_backend = len(session.backend_files) > 0

        completeness = sum([has_frontend, has_corridor, has_backend]) / 3.0 * 100
        metrics['completeness'] = completeness

        # Determine quality rating
        if completeness == 100 and len(issues) == 0:
            metrics['data_quality'] = 'excellent'
        elif completeness >= 66 and len(issues) <= 2:
            metrics['data_quality'] = 'good'
        elif completeness >= 33:
            metrics['data_quality'] = 'fair'
        else:
            metrics['data_quality'] = 'poor'

        logger.info(f"Data quality: {metrics['data_quality']} "
                   f"(completeness: {completeness:.0f}%, issues: {len(issues)})")

        return metrics

    def get_session_summary(self, session: Session) -> str:
        """
        Generate human-readable summary of session

        Args:
            session: Session to summarize

        Returns:
            Summary string
        """
        summary_lines = [
            f"Session: {session.session_id}",
            f"Timestamp: {session.timestamp.strftime('%Y-%m-%d %H:%M:%S')}",
            f"Total Files: {session.total_files}",
            ""
        ]

        # File breakdown
        summary = session.file_summary
        summary_lines.append("File Breakdown:")
        summary_lines.append(f"  Frontend:  {summary['frontend']} files")
        summary_lines.append(f"  Corridor:  {summary['corridor']} files")
        summary_lines.append(f"  Backend:   {summary['backend']} files")
        summary_lines.append(f"  Combined:  {summary['combined']} files")
        summary_lines.append("")

        # File groups
        groups = self.match_file_groups(session)
        complete = len([g for g in groups if g.is_complete])
        summary_lines.append(f"File Groups: {len(groups)} total ({complete} complete)")
        summary_lines.append("")

        # Data quality
        quality = self.analyze_data_quality(session)
        summary_lines.append(f"Data Quality: {quality['data_quality'].upper()}")
        summary_lines.append(f"Completeness: {quality['completeness']:.0f}%")

        if quality['issues']:
            summary_lines.append(f"\nIssues ({len(quality['issues'])}):")
            for issue in quality['issues'][:5]:  # Show first 5
                summary_lines.append(f"  - {issue}")
            if len(quality['issues']) > 5:
                summary_lines.append(f"  ... and {len(quality['issues']) - 5} more")

        return "\n".join(summary_lines)
