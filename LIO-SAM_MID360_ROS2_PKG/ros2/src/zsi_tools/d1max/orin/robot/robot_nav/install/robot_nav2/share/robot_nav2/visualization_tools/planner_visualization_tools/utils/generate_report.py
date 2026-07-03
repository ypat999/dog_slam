#!/usr/bin/env python3
"""
Navigo Data Summary Report Generator

Function:
- Generate detailed data summary reports
- Provide visualization statistics
- Support HTML and text format output
"""

import os
import json
import glob
from datetime import datetime
from collections import defaultdict
import matplotlib.pyplot as plt
import numpy as np
import matplotlib
import warnings

# Disable font warnings
warnings.filterwarnings('ignore', category=UserWarning, module='matplotlib')
warnings.filterwarnings('ignore', message='.*Glyph.*missing.*')
warnings.filterwarnings('ignore', message='.*font.*')

# Set up font support (use English to avoid font issues)
matplotlib.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial Unicode MS', 'sans-serif']
matplotlib.rcParams['axes.unicode_minus'] = False

class NavigoReportGenerator:
    def __init__(self, data_dir=None):
        # If no data_dir is specified, use the default path relative to the script location
        if data_dir is None:
            # Get the directory where the current script is located
            script_dir = os.path.dirname(os.path.abspath(__file__))
            # Navigate to the parent directory's navigo_debug_data
            self.data_dir = os.path.abspath(os.path.join(script_dir, "..", "navigo_debug_data"))
        else:
            self.data_dir = os.path.abspath(data_dir)
            
        # If this is the base navigo_debug_data directory, check for an organized subdirectory
        if os.path.basename(self.data_dir) == "navigo_debug_data":
            organized_path = os.path.join(self.data_dir, "organized")
            if os.path.exists(organized_path):
                self.data_dir = organized_path
                
        self.output_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "output")
        os.makedirs(self.output_dir, exist_ok=True)

    def scan_data_files(self):
        """Scan the data directory and classify files by type"""
        data_files = {
            'frontend': [],
            'corridor': [],
            'backend': [],
            'unknown': []
        }

        # Scan all JSON files
        json_files = glob.glob(os.path.join(self.data_dir, "**/*.json"), recursive=True)

        for file_path in json_files:
            file_name = os.path.basename(file_path)
            classified = False

            # Classify files
            if 'frontend' in file_name.lower():
                data_files['frontend'].append(file_path)
                classified = True
            elif 'corridor' in file_name.lower():
                data_files['corridor'].append(file_path)
                classified = True
            elif 'backend' in file_name.lower():
                data_files['backend'].append(file_path)
                classified = True
            else:
                data_files['unknown'].append(file_path)

        return data_files

    def extract_file_stats(self, file_path):
        """Extract file statistics"""
        stats = {
            'file_size': os.path.getsize(file_path),
            'modified_time': datetime.fromtimestamp(os.path.getmtime(file_path)),
            'data_points': 0,
            'duration_ms': 0,
            'success': False
        }
        
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
                
            # Extract general information
            stats['success'] = data.get('success', False) or data.get('generation_result', {}).get('success', False)
            
            # Extract specific information based on file type
            if 'frontend' in file_path.lower():
                trajectory = data.get('search_result', {}).get('path_points', [])
                stats['data_points'] = len(trajectory)
                stats['duration_ms'] = data.get('search_result', {}).get('search_time_ms', 0)
                
            elif 'corridor' in file_path.lower():
                # New format
                corridor_per_point = data.get('corridor_per_point', [])
                # Old format
                corridor_segments = data.get('corridor_segments', [])
                stats['data_points'] = len(corridor_per_point) if corridor_per_point else len(corridor_segments)
                stats['duration_ms'] = data.get('generation_result', {}).get('processing_time_ms', 0)
                
            elif 'backend' in file_path.lower():
                # New format
                trajectory_segment = data.get('trajectory_segment', [])
                # Old format
                optimized_path = data.get('optimized_path', [])
                stats['data_points'] = len(trajectory_segment) if trajectory_segment else len(optimized_path)
                stats['duration_ms'] = data.get('optimization_result', {}).get('optimization_time_ms', 0)
                
        except (json.JSONDecodeError, FileNotFoundError, Exception):
            pass
            
        return stats

    def generate_text_report(self):
        """Generate text format report"""
        data_files = self.scan_data_files()
        
        report = []
        report.append("=" * 80)
        report.append("📊 Navigo Data Analysis Report")
        report.append("=" * 80)
        report.append(f"📅 Report Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        report.append(f"📁 Data Directory: {self.data_dir}")
        report.append("")
        
        # Overall statistics
        total_files = sum(len(files) for files in data_files.values())
        report.append("📈 Overall Statistics:")
        report.append(f"   Total Files: {total_files}")
        report.append("")
        
        # Statistics by type
        type_names = {
            'frontend': 'FRONTEND',
            'corridor': 'CORRIDOR',
            'backend': 'BACKEND',
            'unknown': 'UNKNOWN'
        }
        
        for data_type, files in data_files.items():
            if not files:
                continue
                
            report.append(f"📂 {type_names[data_type]} Data ({len(files)} files):")
            
            # Extract statistics
            stats_list = [self.extract_file_stats(f) for f in files]
            successful_files = [s for s in stats_list if s['success']]
            
            # File size statistics
            sizes = [s['file_size'] for s in stats_list]
            report.append(f"   File Size: {min(sizes)/1024:.1f}KB - {max(sizes)/1024:.1f}KB (Avg: {np.mean(sizes)/1024:.1f}KB)")
            
            # Data points statistics
            data_points = [s['data_points'] for s in stats_list if s['data_points'] > 0]
            if data_points:
                report.append(f"   Data Points: {min(data_points)} - {max(data_points)} (Avg: {np.mean(data_points):.1f})")
            
            # Processing time statistics
            durations = [s['duration_ms'] for s in stats_list if s['duration_ms'] > 0]
            if durations:
                report.append(f"   Processing Time: {min(durations):.2f}ms - {max(durations):.2f}ms (Avg: {np.mean(durations):.2f}ms)")
            
            # Success rate
            if stats_list:
                success_rate = len(successful_files) / len(stats_list) * 100
                report.append(f"   Success Rate: {success_rate:.1f}% ({len(successful_files)}/{len(stats_list)})")
            
            report.append("")
        
        # Recent files
        all_files_with_time = []
        for files in data_files.values():
            for file_path in files:
                all_files_with_time.append((file_path, os.path.getmtime(file_path)))
        
        # Sort by time and show the 10 most recent files
        recent_files = sorted(all_files_with_time, key=lambda x: x[1], reverse=True)[:10]
        report.append("🕒 Recent Files:")
        for file_path, timestamp in recent_files:
            file_time = datetime.fromtimestamp(timestamp)
            report.append(f"   {file_time.strftime('%Y-%m-%d %H:%M:%S')} - {os.path.basename(file_path)}")
        
        report.append("")
        report.append("=" * 80)
        report.append("Report End")
        report.append("=" * 80)
        
        return "\n".join(report)

    def generate_charts(self):
        """Generate charts"""
        data_files = self.scan_data_files()
        
        # Create charts
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        fig.suptitle('Navigo Data Analysis Report', fontsize=16)
        
        # 1. File type distribution pie chart
        file_counts = [len(files) for files in data_files.values() if files]
        file_labels = [k.upper() for k, v in data_files.items() if v]
        
        if file_counts:
            axes[0, 0].pie(file_counts, labels=file_labels, autopct='%1.1f%%')
            axes[0, 0].set_title('File Type Distribution')
        
        # 2. File size distribution histogram
        all_sizes = []
        for files in data_files.values():
            for file_path in files:
                all_sizes.append(os.path.getsize(file_path) / 1024)  # KB
        
        if all_sizes:
            axes[0, 1].hist(all_sizes, bins=20, alpha=0.7)
            axes[0, 1].set_xlabel('File Size (KB)')
            axes[0, 1].set_ylabel('Number of Files')
            axes[0, 1].set_title('File Size Distribution')
        
        # 3. Data points distribution
        all_data_points = []
        for files in data_files.values():
            for file_path in files:
                stats = self.extract_file_stats(file_path)
                if stats['data_points'] > 0:
                    all_data_points.append(stats['data_points'])
        
        if all_data_points:
            axes[1, 0].hist(all_data_points, bins=20, alpha=0.7)
            axes[1, 0].set_xlabel('Data Points')
            axes[1, 0].set_ylabel('Number of Files')
            axes[1, 0].set_title('Data Points Distribution')
        
        # 4. Processing time distribution
        all_durations = []
        for files in data_files.values():
            for file_path in files:
                stats = self.extract_file_stats(file_path)
                if stats['duration_ms'] > 0:
                    all_durations.append(stats['duration_ms'])
        
        if all_durations:
            axes[1, 1].hist(all_durations, bins=20, alpha=0.7)
            axes[1, 1].set_xlabel('Processing Time (ms)')
            axes[1, 1].set_ylabel('Number of Files')
            axes[1, 1].set_title('Processing Time Distribution')
        
        plt.tight_layout()
        
        # Save chart
        chart_path = os.path.join(self.output_dir, f"data_analysis_chart_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png")
        plt.savefig(chart_path, dpi=300, bbox_inches='tight')
        plt.close()
        
        return chart_path

    def generate_report(self, format='text'):
        """Generate report"""
        if format == 'text':
            report_content = self.generate_text_report()
            report_path = os.path.join(self.output_dir, f"data_analysis_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt")
            
            with open(report_path, 'w', encoding='utf-8') as f:
                f.write(report_content)
                
            print(f"✅ Report generated: {report_path}")
            return report_path
            
        elif format == 'chart':
            chart_path = self.generate_charts()
            print(f"✅ Chart report generated: {chart_path}")
            return chart_path

def main():
    """Main function"""
    import argparse
    
    # Calculate default data directory path
    script_dir = os.path.dirname(os.path.abspath(__file__))
    default_data_dir = os.path.abspath(os.path.join(script_dir, "..", "navigo_debug_data"))
    
    parser = argparse.ArgumentParser(description='Navigo Data Summary Report Generator')
    parser.add_argument('--data-dir', default=default_data_dir,
                       help='Data directory path')
    parser.add_argument('--format', choices=['text', 'chart'], default='text',
                       help='Report format (text: text report, chart: chart report)')
    
    args = parser.parse_args()
    
    generator = NavigoReportGenerator(args.data_dir)
    generator.generate_report(args.format)

if __name__ == "__main__":
    main()