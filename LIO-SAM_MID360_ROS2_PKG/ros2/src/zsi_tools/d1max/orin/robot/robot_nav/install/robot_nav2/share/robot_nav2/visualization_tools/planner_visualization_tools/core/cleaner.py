#!/usr/bin/env python3
"""
Navigo Data Cleaner Module

Manages data lifecycle with automatic cleanup of old files.
Provides safe deletion with verification and dry-run mode.
"""

from datetime import datetime, timedelta
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from .config import NavigoConfig
from .logger import logger


class DataCleaner:
    """Manages data lifecycle and cleanup operations"""

    def __init__(self, retention_days: Optional[int] = None):
        """
        Initialize data cleaner

        Args:
            retention_days: Days to retain data (default: from config)
        """
        self.retention_days = retention_days or NavigoConfig.DATA_RETENTION_DAYS
        self.grace_hours = NavigoConfig.CLEANUP_GRACE_HOURS
        logger.debug(f"DataCleaner initialized (retention: {self.retention_days} days)")

    def cleanup(self, source_dir: Optional[Path] = None,
               dry_run: bool = False) -> Dict:
        """
        Clean up old data files

        Args:
            source_dir: Directory to clean (default: from config)
            dry_run: If True, only identify files without deleting

        Returns:
            Dict with cleanup statistics
        """
        source_dir = source_dir or NavigoConfig.get_data_source()

        logger.section(f"Data Cleanup {'(DRY RUN)' if dry_run else ''}")
        logger.info(f"Source Directory: {source_dir}")
        logger.info(f"Retention Policy: {self.retention_days} days")
        logger.info(f"Grace Period: {self.grace_hours} hours")

        # Get cleanup candidates
        candidates = self.get_cleanup_candidates(source_dir)

        if not candidates:
            logger.info("✅ No files to clean up")
            return {
                'scanned': 0,
                'identified': 0,
                'deleted': 0,
                'failed': 0,
                'space_freed': 0
            }

        logger.info(f"🔍 Found {len(candidates)} files to clean up")

        # Show sample files
        if candidates:
            logger.info("\nSample files to be deleted:")
            for file_info in candidates[:5]:
                age_days = file_info['age_days']
                size_mb = file_info['size'] / (1024 * 1024)
                logger.info(f"  {file_info['name']} "
                          f"(age: {age_days:.1f}d, size: {size_mb:.2f}MB)")
            if len(candidates) > 5:
                logger.info(f"  ... and {len(candidates) - 5} more")

        # Calculate total size
        total_size = sum(f['size'] for f in candidates)
        total_size_mb = total_size / (1024 * 1024)
        logger.info(f"\nTotal space to be freed: {total_size_mb:.2f} MB")

        # Perform deletion
        if dry_run:
            logger.info("\n⚠️  DRY RUN: No files will be deleted")
            return {
                'scanned': len(candidates),
                'identified': len(candidates),
                'deleted': 0,
                'failed': 0,
                'space_freed': 0
            }

        deleted, failed, space_freed = self._delete_files(candidates)

        logger.info(f"\n✅ Cleanup complete:")
        logger.info(f"  Deleted: {deleted} files")
        logger.info(f"  Failed:  {failed} files")
        logger.info(f"  Space Freed: {space_freed / (1024 * 1024):.2f} MB")

        return {
            'scanned': len(candidates),
            'identified': len(candidates),
            'deleted': deleted,
            'failed': failed,
            'space_freed': space_freed
        }

    def get_cleanup_candidates(self, source_dir: Path) -> List[Dict]:
        """
        Identify files eligible for cleanup

        Args:
            source_dir: Directory to scan

        Returns:
            List of dicts with file information
        """
        if not source_dir.exists():
            logger.warning(f"Directory does not exist: {source_dir}")
            return []

        candidates = []
        cutoff_time = self._get_cutoff_timestamp()

        logger.debug(f"Scanning for files older than {cutoff_time}")

        # Find all JSON files
        for json_file in source_dir.rglob("*.json"):
            try:
                mtime = json_file.stat().st_mtime

                if mtime < cutoff_time:
                    file_info = {
                        'path': json_file,
                        'name': json_file.name,
                        'mtime': mtime,
                        'size': json_file.stat().st_size,
                        'age_days': (datetime.now().timestamp() - mtime) / (24 * 3600)
                    }
                    candidates.append(file_info)

            except Exception as e:
                logger.warning(f"Failed to process {json_file.name}: {e}")
                continue

        # Sort by age (oldest first)
        candidates.sort(key=lambda x: x['mtime'])

        return candidates

    def cleanup_output_directory(self, output_dir: Optional[Path] = None,
                                days: int = 7, dry_run: bool = False) -> Dict:
        """
        Clean up old visualization outputs

        Args:
            output_dir: Output directory to clean (default: from config)
            days: Days to retain (default: 7)
            dry_run: If True, only identify files without deleting

        Returns:
            Dict with cleanup statistics
        """
        output_dir = output_dir or NavigoConfig.OUTPUT_DIR

        logger.section(f"Output Cleanup {'(DRY RUN)' if dry_run else ''}")
        logger.info(f"Output Directory: {output_dir}")
        logger.info(f"Retention: {days} days")

        if not output_dir.exists():
            logger.info("✅ Output directory does not exist, nothing to clean")
            return {'deleted': 0, 'failed': 0, 'space_freed': 0}

        cutoff_time = datetime.now().timestamp() - (days * 24 * 3600)
        candidates = []

        # Find PNG files older than cutoff
        for png_file in output_dir.rglob("*.png"):
            try:
                mtime = png_file.stat().st_mtime
                if mtime < cutoff_time:
                    candidates.append({
                        'path': png_file,
                        'name': png_file.name,
                        'mtime': mtime,
                        'size': png_file.stat().st_size,
                        'age_days': (datetime.now().timestamp() - mtime) / (24 * 3600)
                    })
            except:
                continue

        if not candidates:
            logger.info("✅ No old output files to clean up")
            return {'deleted': 0, 'failed': 0, 'space_freed': 0}

        logger.info(f"🔍 Found {len(candidates)} old output files")

        if dry_run:
            logger.info("⚠️  DRY RUN: No files will be deleted")
            return {'deleted': 0, 'failed': 0, 'space_freed': 0}

        deleted, failed, space_freed = self._delete_files(candidates)

        logger.info(f"✅ Cleaned {deleted} output files ({space_freed / (1024 * 1024):.2f} MB)")

        return {
            'deleted': deleted,
            'failed': failed,
            'space_freed': space_freed
        }

    def _get_cutoff_timestamp(self) -> float:
        """
        Calculate cutoff timestamp based on retention policy

        Returns:
            Timestamp (seconds since epoch)
        """
        # Current time minus retention period minus grace period
        now = datetime.now()
        cutoff = now - timedelta(days=self.retention_days, hours=self.grace_hours)
        return cutoff.timestamp()

    def _delete_files(self, file_infos: List[Dict]) -> Tuple[int, int, int]:
        """
        Delete files with verification

        Args:
            file_infos: List of file information dicts

        Returns:
            Tuple of (deleted_count, failed_count, space_freed)
        """
        deleted = 0
        failed = 0
        space_freed = 0

        for file_info in file_infos:
            try:
                file_path = file_info['path']
                file_size = file_info['size']

                # Verify file still exists and is old enough
                if file_path.exists():
                    current_mtime = file_path.stat().st_mtime
                    cutoff = self._get_cutoff_timestamp()

                    if current_mtime < cutoff:
                        file_path.unlink()
                        deleted += 1
                        space_freed += file_size
                        logger.debug(f"Deleted: {file_path.name}")
                    else:
                        logger.debug(f"Skipped (modified): {file_path.name}")
                else:
                    logger.debug(f"Skipped (not found): {file_info['name']}")

            except Exception as e:
                failed += 1
                logger.warning(f"Failed to delete {file_info['name']}: {e}")

        return deleted, failed, space_freed

    def get_storage_info(self, source_dir: Optional[Path] = None) -> Dict:
        """
        Get storage information for data directory

        Args:
            source_dir: Directory to analyze (default: from config)

        Returns:
            Dict with storage statistics
        """
        source_dir = source_dir or NavigoConfig.get_data_source()

        if not source_dir.exists():
            return {
                'total_files': 0,
                'total_size': 0,
                'total_size_mb': 0,
                'oldest_file': None,
                'newest_file': None
            }

        files = list(source_dir.rglob("*.json"))
        total_size = sum(f.stat().st_size for f in files if f.exists())

        oldest_mtime = None
        newest_mtime = None

        for f in files:
            try:
                mtime = f.stat().st_mtime
                if oldest_mtime is None or mtime < oldest_mtime:
                    oldest_mtime = mtime
                if newest_mtime is None or mtime > newest_mtime:
                    newest_mtime = mtime
            except:
                continue

        return {
            'total_files': len(files),
            'total_size': total_size,
            'total_size_mb': total_size / (1024 * 1024),
            'oldest_file': datetime.fromtimestamp(oldest_mtime) if oldest_mtime else None,
            'newest_file': datetime.fromtimestamp(newest_mtime) if newest_mtime else None
        }
