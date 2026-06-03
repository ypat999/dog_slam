#!/usr/bin/env python3
"""
Navigo Visualization Orchestrator Module

Coordinates execution of all visualization scripts.
Supports parallel processing and progress tracking.
"""

import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

from .config import NavigoConfig
from .logger import logger
from .models import Session, SessionAnalysisResult, VisualizationResult
from .processor import DataProcessor


class VisualizationOrchestrator:
    """Orchestrates visualization script execution"""

    def __init__(self, processor: Optional[DataProcessor] = None):
        """
        Initialize visualization orchestrator

        Args:
            processor: DataProcessor instance (creates new if None)
        """
        self.processor = processor or DataProcessor()
        self.visualization_dir = NavigoConfig.VISUALIZATION_DIR
        logger.debug("VisualizationOrchestrator initialized")

    def visualize_session(self, session: Session,
                         output_base_dir: Optional[Path] = None) -> SessionAnalysisResult:
        """
        Run all visualizations for a session

        Args:
            session: Session to visualize
            output_base_dir: Base output directory (default: from config)

        Returns:
            SessionAnalysisResult with visualization results
        """
        start_time = datetime.now()
        logger.section(f"Visualizing Session: {session.session_id}")

        output_base_dir = output_base_dir or NavigoConfig.OUTPUT_DIR
        results = []

        # Prepare visualization data
        vis_data = self.processor.prepare_for_visualization(session)

        # Visualize each type
        for data_type in ['frontend', 'corridor', 'backend']:
            files = vis_data.get(data_type, [])
            if not files:
                logger.info(f"⏭️  Skipping {data_type}: no data files")
                continue

            logger.info(f"📊 Processing {data_type} ({len(files)} files)...")
            type_results = self._visualize_type(data_type, files)
            results.extend(type_results)

        # Visualize combined (if complete groups available)
        combined_groups = vis_data.get('combined_groups', [])
        if combined_groups:
            logger.info(f"📊 Processing combined visualizations ({len(combined_groups)} groups)...")
            combined_results = self._visualize_combined(combined_groups)
            results.extend(combined_results)
        else:
            logger.info("⏭️  Skipping combined: no complete file groups")

        # Create result
        end_time = datetime.now()
        analysis_result = SessionAnalysisResult(
            session=session,
            visualizations=results,
            start_time=start_time,
            end_time=end_time
        )

        # Log summary
        logger.section("Visualization Summary")
        logger.info(f"Session: {session.session_id}")
        logger.info(f"Success Rate: {analysis_result.success_count}/{analysis_result.total_count} "
                   f"({analysis_result.success_rate:.1f}%)")
        logger.info(f"Execution Time: {analysis_result.execution_time:.2f}s")

        if analysis_result.output_files:
            logger.info(f"\n📁 Generated Files ({len(analysis_result.output_files)}):")
            for output_file in analysis_result.output_files[:10]:  # Show first 10
                rel_path = self._get_relative_path(output_file)
                logger.info(f"  {rel_path}")
            if len(analysis_result.output_files) > 10:
                logger.info(f"  ... and {len(analysis_result.output_files) - 10} more")

        return analysis_result

    def _visualize_type(self, data_type: str, files: List[Path]) -> List[VisualizationResult]:
        """
        Visualize files of a specific type

        Args:
            data_type: Type of data (frontend, corridor, backend)
            files: List of file paths to visualize

        Returns:
            List of VisualizationResult objects
        """
        results = []
        output_dir = NavigoConfig.get_output_dir(data_type)
        output_dir.mkdir(parents=True, exist_ok=True)

        # Use parallel processing if enabled
        if NavigoConfig.ENABLE_PARALLEL and len(files) > 1:
            results = self._visualize_parallel(data_type, files, output_dir)
        else:
            results = self._visualize_sequential(data_type, files, output_dir)

        # Log results
        success_count = sum(1 for r in results if r.success)
        logger.info(f"  ✅ {data_type}: {success_count}/{len(results)} successful")

        return results

    def _visualize_sequential(self, data_type: str, files: List[Path],
                             output_dir: Path) -> List[VisualizationResult]:
        """
        Visualize files sequentially

        Args:
            data_type: Type of data
            files: List of files
            output_dir: Output directory

        Returns:
            List of VisualizationResult objects
        """
        results = []

        for i, file_path in enumerate(files, 1):
            logger.progress(i, len(files), f"Processing {data_type}")
            result = self._run_visualization_script(data_type, file_path, output_dir)
            results.append(result)

        return results

    def _visualize_parallel(self, data_type: str, files: List[Path],
                           output_dir: Path) -> List[VisualizationResult]:
        """
        Visualize files in parallel

        Args:
            data_type: Type of data
            files: List of files
            output_dir: Output directory

        Returns:
            List of VisualizationResult objects
        """
        results = []
        max_workers = NavigoConfig.MAX_WORKERS or min(4, len(files))

        with ThreadPoolExecutor(max_workers=max_workers) as executor:
            # Submit all tasks
            futures = {
                executor.submit(self._run_visualization_script, data_type, file_path, output_dir): file_path
                for file_path in files
            }

            # Collect results as they complete
            completed = 0
            for future in as_completed(futures):
                completed += 1
                logger.progress(completed, len(files), f"Processing {data_type}")
                try:
                    result = future.result(timeout=NavigoConfig.VISUALIZATION_TIMEOUT)
                    results.append(result)
                except Exception as e:
                    file_path = futures[future]
                    logger.error(f"Failed to visualize {file_path.name}: {e}")
                    results.append(VisualizationResult(
                        success=False,
                        data_type=data_type,
                        input_file=file_path,
                        error_message=str(e)
                    ))

        return results

    def _visualize_combined(self, groups: List[Dict]) -> List[VisualizationResult]:
        """
        Visualize combined data groups

        Args:
            groups: List of dicts with frontend/corridor/backend paths

        Returns:
            List of VisualizationResult objects
        """
        results = []
        output_dir = NavigoConfig.OUTPUT_COMBINED_DIR
        output_dir.mkdir(parents=True, exist_ok=True)

        for i, group in enumerate(groups, 1):
            logger.progress(i, len(groups), "Processing combined")
            result = self._run_combined_visualization(group, output_dir)
            results.append(result)

        return results

    def _run_visualization_script(self, data_type: str, input_file: Path,
                                 output_dir: Path) -> VisualizationResult:
        """
        Run a single visualization script

        Args:
            data_type: Type of data
            input_file: Input file path
            output_dir: Output directory

        Returns:
            VisualizationResult
        """
        start_time = datetime.now()

        try:
            # Get script path
            script_path = NavigoConfig.get_visualizer_path(data_type)

            # Generate output filename
            file_basename = input_file.stem
            output_file = output_dir / f"{data_type}_analysis_{file_basename}.png"

            # Build command
            cmd = [
                sys.executable,
                str(script_path),
                str(input_file),
                '--save', str(output_file),
                '--no-show'
            ]

            # Execute script
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=NavigoConfig.VISUALIZATION_TIMEOUT,
                cwd=str(NavigoConfig.TOOLS_BASE_DIR)
            )

            execution_time = (datetime.now() - start_time).total_seconds()

            if result.returncode == 0:
                logger.debug(f"✅ Generated: {output_file.name}")
                return VisualizationResult(
                    success=True,
                    data_type=data_type,
                    input_file=input_file,
                    output_file=output_file,
                    execution_time=execution_time
                )
            else:
                error_msg = result.stderr.strip() or "Script returned non-zero exit code"
                logger.debug(f"❌ Failed: {input_file.name} - {error_msg}")
                return VisualizationResult(
                    success=False,
                    data_type=data_type,
                    input_file=input_file,
                    error_message=error_msg,
                    execution_time=execution_time
                )

        except subprocess.TimeoutExpired:
            error_msg = f"Timeout after {NavigoConfig.VISUALIZATION_TIMEOUT}s"
            logger.debug(f"❌ Timeout: {input_file.name}")
            return VisualizationResult(
                success=False,
                data_type=data_type,
                input_file=input_file,
                error_message=error_msg
            )

        except Exception as e:
            execution_time = (datetime.now() - start_time).total_seconds()
            logger.debug(f"❌ Exception: {input_file.name} - {e}")
            return VisualizationResult(
                success=False,
                data_type=data_type,
                input_file=input_file,
                error_message=str(e),
                execution_time=execution_time
            )

    def _run_combined_visualization(self, group: Dict, output_dir: Path) -> VisualizationResult:
        """
        Run combined visualization for a file group

        Args:
            group: Dict with frontend/corridor/backend paths
            output_dir: Output directory

        Returns:
            VisualizationResult
        """
        start_time = datetime.now()

        try:
            script_path = NavigoConfig.get_visualizer_path('combined')

            # Generate output filename from frontend file
            frontend_name = Path(group['frontend']).stem
            output_file = output_dir / f"combined_analysis_{frontend_name}.png"

            # Build command
            cmd = [
                sys.executable,
                str(script_path),
                '--frontend', str(group['frontend']),
                '--corridor', str(group['corridor']),
                '--backend', str(group['backend']),
                '--save', str(output_file),
                '--no-show'
            ]

            # Execute script
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=NavigoConfig.VISUALIZATION_TIMEOUT,
                cwd=str(NavigoConfig.TOOLS_BASE_DIR)
            )

            execution_time = (datetime.now() - start_time).total_seconds()

            if result.returncode == 0:
                logger.debug(f"✅ Generated: {output_file.name}")
                return VisualizationResult(
                    success=True,
                    data_type='combined',
                    input_file=Path(group['frontend']),
                    output_file=output_file,
                    execution_time=execution_time
                )
            else:
                error_msg = result.stderr.strip() or "Script returned non-zero exit code"
                logger.debug(f"❌ Failed: combined - {error_msg}")
                return VisualizationResult(
                    success=False,
                    data_type='combined',
                    input_file=Path(group['frontend']),
                    error_message=error_msg,
                    execution_time=execution_time
                )

        except Exception as e:
            execution_time = (datetime.now() - start_time).total_seconds()
            logger.debug(f"❌ Exception: combined - {e}")
            return VisualizationResult(
                success=False,
                data_type='combined',
                input_file=Path(group['frontend']),
                error_message=str(e),
                execution_time=execution_time
            )

    def _get_relative_path(self, path: Path) -> str:
        """
        Get path relative to tools base directory

        Args:
            path: Absolute path

        Returns:
            Relative path string
        """
        try:
            return str(path.relative_to(NavigoConfig.TOOLS_BASE_DIR))
        except ValueError:
            return str(path)
