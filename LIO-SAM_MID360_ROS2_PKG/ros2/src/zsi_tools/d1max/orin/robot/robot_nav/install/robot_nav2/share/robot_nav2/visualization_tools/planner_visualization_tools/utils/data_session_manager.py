#!/usr/bin/env python3
"""
Navigo 调试数据会话管理器

功能：
- 统一管理前端搜索、走廊生成、后端优化三种数据
- 自动按时间戳和会话组织数据
- 提供一键运行所有可视化脚本的功能
- 支持数据清理和归档
"""

import json
import os
import glob
import re
import shutil
from datetime import datetime
from collections import defaultdict
import subprocess
import sys

class NavigoDataSessionManager:
    def __init__(self, data_dir=None):
        # 如果没有指定data_dir，则使用相对于脚本位置的默认路径
        if data_dir is None:
            # 获取当前脚本所在的目录
            script_dir = os.path.dirname(os.path.abspath(__file__))
            # 导航到父目录的navigo_debug_data
            self.data_dir = os.path.abspath(os.path.join(script_dir, "..", "navigo_debug_data"))
        else:
            self.data_dir = os.path.abspath(data_dir)
            
        # 如果data_dir是planner_visualization_tools目录，则查找其下的navigo_debug_data目录
        if os.path.basename(self.data_dir) == "planner_visualization_tools":
            debug_data_path = os.path.join(self.data_dir, "navigo_debug_data")
            organized_path = os.path.join(debug_data_path, "organized")
            if os.path.exists(organized_path):
                self.data_dir = organized_path
            elif os.path.exists(debug_data_path):
                self.data_dir = debug_data_path
                
        # 如果是基础的navigo_debug_data目录，检查是否有organized子目录
        elif os.path.basename(self.data_dir) == "navigo_debug_data":
            organized_path = os.path.join(self.data_dir, "organized")
            if os.path.exists(organized_path):
                self.data_dir = organized_path
                
        self.visualization_tools_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "visualization")

        # 数据类型映射
        self.data_types = {
            'frontend': {
                'pattern': r'.*frontend.*\.json$',
                'script': 'visualize_frontend_search.py',
                'description': '前端搜索数据'
            },
            'corridor': {
                'pattern': r'.*(corridor|corridor_seg).*\.json$',
                'script': 'visualize_corridor.py',
                'description': '走廊生成数据'
            },
            'backend': {
                'pattern': r'.*backend.*\.json$',
                'script': 'visualize_backend_optimization.py',
                'description': '后端优化数据'
            },
            'combined': {
                'pattern': r'.*combined.*\.json$',
                'script': 'visualize_combined.py',
                'description': '组合规划数据'
            }
        }

    def scan_data_files(self):
        """扫描数据目录，按类型分类文件"""
        data_files = {
            'frontend': [],
            'corridor': [],
            'backend': [],
            'unknown': []
        }

        # 扫描所有JSON文件
        json_files = glob.glob(os.path.join(self.data_dir, "**/*.json"), recursive=True)

        for file_path in json_files:
            file_name = os.path.basename(file_path)
            classified = False

            for data_type, config in self.data_types.items():
                if re.match(config['pattern'], file_name, re.IGNORECASE):
                    data_files[data_type].append(file_path)
                    classified = True
                    break

            if not classified:
                data_files['unknown'].append(file_path)

        return data_files

    def get_file_timestamp(self, file_path):
        """从文件名或文件内容提取时间戳"""
        file_name = os.path.basename(file_path)

        # 尝试从文件名提取时间戳 (格式: 20241217_141211)
        timestamp_match = re.search(r'(\d{8}_\d{6})', file_name)
        if timestamp_match:
            try:
                return datetime.strptime(timestamp_match.group(1), '%Y%m%d_%H%M%S')
            except ValueError:
                pass

        # 尝试从文件内容提取时间戳
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
                timestamp_str = data.get('timestamp', '')
                if timestamp_str:
                    # 尝试多种时间格式
                    formats = ['%Y-%m-%d %H:%M:%S', '%Y%m%d_%H%M%S']
                    for fmt in formats:
                        try:
                            return datetime.strptime(timestamp_str, fmt)
                        except ValueError:
                            continue
        except (json.JSONDecodeError, FileNotFoundError):
            pass

        # 使用文件修改时间作为备用
        return datetime.fromtimestamp(os.path.getmtime(file_path))

    def organize_by_sessions(self):
        """按会话组织数据文件"""
        data_files = self.scan_data_files()
        sessions = defaultdict(lambda: {'frontend': [], 'corridor': [], 'backend': [], 'timestamp': None})

        # 按类型处理文件
        for data_type, files in data_files.items():
            if data_type == 'unknown':
                continue

            for file_path in files:
                timestamp = self.get_file_timestamp(file_path)

                # 使用时间戳的日期和小时作为会话标识
                session_key = timestamp.strftime('%Y%m%d_%H')

                sessions[session_key][data_type].append(file_path)

                # 更新会话时间戳（使用最新的时间）
                if sessions[session_key]['timestamp'] is None or timestamp > sessions[session_key]['timestamp']:
                    sessions[session_key]['timestamp'] = timestamp

        return dict(sessions)

    def print_session_summary(self):
        """打印会话摘要"""
        sessions = self.organize_by_sessions()

        print("=" * 80)
        print("🔍 Navigo 调试数据会话摘要")
        print("=" * 80)

        if not sessions:
            print("❌ 没有找到任何数据文件")
            return

        # 按时间戳排序会话
        sorted_sessions = sorted(sessions.items(),
                               key=lambda x: x[1]['timestamp'],
                               reverse=True)

        for session_key, session_data in sorted_sessions:
            timestamp = session_data['timestamp']
            print(f"\n📅 会话: {session_key} ({timestamp.strftime('%Y-%m-%d %H:%M:%S')})")

            total_files = 0
            for data_type in ['frontend', 'corridor', 'backend']:
                count = len(session_data[data_type])
                total_files += count
                if count > 0:
                    type_desc = self.data_types[data_type]['description']
                    print(f"   📊 {type_desc}: {count} 个文件")

            print(f"   📁 总计: {total_files} 个文件")

        print(f"\n📈 统计:")
        print(f"   总会话数: {len(sessions)}")
        all_files = self.scan_data_files()
        total_count = sum(len(files) for data_type, files in all_files.items() if data_type != 'unknown')
        print(f"   总文件数: {total_count}")

        if all_files['unknown']:
            print(f"   ❓ 未分类文件: {len(all_files['unknown'])}")

    def match_related_files(self, session_data):
        """匹配相关的前端、走廊和后端文件，确保它们属于同一规划任务"""
        if not all(session_data[dt] for dt in ['frontend', 'corridor', 'backend']):
            return None
            
        # 获取所有文件的时间戳
        file_timestamps = {}
        for data_type in ['frontend', 'corridor', 'backend']:
            for file_path in session_data[data_type]:
                timestamp = self.get_file_timestamp(file_path)
                file_timestamps[file_path] = {
                    'timestamp': timestamp,
                    'type': data_type
                }
        
        # 按时间戳排序所有文件
        sorted_files = sorted(file_timestamps.items(), key=lambda x: x[1]['timestamp'])
        
        # 尝试匹配最接近的文件组
        matched_groups = []
        used_files = set()
        
        # 对于每个前端文件，寻找最近的走廊和后端文件
        for file_path, info in sorted_files:
            if info['type'] == 'frontend' and file_path not in used_files:
                frontend_time = info['timestamp']
                
                # 寻找最近的走廊文件
                closest_corridor = None
                min_corridor_diff = float('inf')
                
                # 寻找最近的后端文件
                closest_backend = None
                min_backend_diff = float('inf')
                
                for other_path, other_info in sorted_files:
                    if other_path not in used_files and other_path != file_path:
                        time_diff = abs((other_info['timestamp'] - frontend_time).total_seconds())
                        
                        if other_info['type'] == 'corridor' and time_diff < min_corridor_diff:
                            # 对于走廊文件，优先选择不包含"_seg_"的文件
                            if '_seg_' not in os.path.basename(other_path):
                                closest_corridor = other_path
                                min_corridor_diff = time_diff
                                
                        elif other_info['type'] == 'backend' and time_diff < min_backend_diff:
                            closest_backend = other_path
                            min_backend_diff = time_diff
                
                # 如果找到了匹配的文件，添加到结果中
                if closest_corridor and closest_backend:
                    matched_groups.append({
                        'frontend': file_path,
                        'corridor': closest_corridor,
                        'backend': closest_backend,
                        'timestamp': frontend_time
                    })
                    used_files.add(file_path)
                    used_files.add(closest_corridor)
                    used_files.add(closest_backend)
        
        return matched_groups

    def run_visualization_for_session(self, session_key, auto_approve=False):
        """为指定会话运行所有可视化脚本"""
        sessions = self.organize_by_sessions()

        if session_key not in sessions:
            print(f"❌ 会话 {session_key} 不存在")
            return False

        session_data = sessions[session_key]
        timestamp = session_data['timestamp'].strftime('%Y%m%d_%H%M%S')

        print(f"🚀 开始处理会话: {session_key}")
        print(f"📅 时间: {session_data['timestamp'].strftime('%Y-%m-%d %H:%M:%S')}")

        results = {}

        # 尝试匹配相关文件
        matched_groups = self.match_related_files(session_data)
        
        if matched_groups:
            print(f"🔗 发现 {len(matched_groups)} 组相关文件")
            for i, group in enumerate(matched_groups):
                print(f"   组 {i+1}:")
                print(f"     前端: {os.path.basename(group['frontend'])}")
                print(f"     走廊: {os.path.basename(group['corridor'])}")
                print(f"     后端: {os.path.basename(group['backend'])}")
        else:
            print("⚠️  未找到明确的相关文件组，将使用最新的独立文件")

        # 为每种数据类型运行可视化（处理所有文件，不仅仅是最新的一個）
        for data_type in ['frontend', 'corridor', 'backend']:
            files = session_data[data_type]
            if not files:
                print(f"⏭️  跳过{self.data_types[data_type]['description']}: 没有数据文件")
                continue

            script_name = self.data_types[data_type]['script']
            script_path = os.path.join(self.visualization_tools_dir, script_name)

            if not os.path.exists(script_path):
                print(f"❌ 可视化脚本不存在: {script_path}")
                continue

            print(f"\n📊 处理{self.data_types[data_type]['description']} ({len(files)} 个文件)...")

            # 为每种数据类型创建独立的子目录
            output_subdir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "output", data_type)
            os.makedirs(output_subdir, exist_ok=True)

            # 处理所有文件（而不仅仅是最新的一個）
            processed_count = 0
            for file_path in files:
                # 获取文件的时间戳用于输出文件名
                file_timestamp = self.get_file_timestamp(file_path).strftime('%Y%m%d_%H%M%S')
                file_basename = os.path.splitext(os.path.basename(file_path))[0]
                
                print(f"   📄 处理文件: {os.path.basename(file_path)}")

                # 生成输出文件名
                output_file = f"{data_type}_analysis_{file_basename}.png"
                output_path = os.path.join(output_subdir, output_file)

                try:
                    # 运行可视化脚本
                    cmd = ['python3', script_path, file_path, '--save', output_path, '--no-show']
                    print(f"   🔧 执行: {' '.join(cmd)}")

                    result = subprocess.run(cmd, capture_output=True, text=True, cwd=os.path.dirname(os.path.dirname(self.visualization_tools_dir)))

                    if result.returncode == 0:
                        print(f"   ✅ 成功生成: {data_type}/{output_file}")
                        results[f"{data_type}_{file_basename}"] = {'success': True, 'output_file': output_path, 'input_file': file_path}
                        processed_count += 1
                    else:
                        print(f"   ❌ 执行失败: {result.stderr}")
                        results[f"{data_type}_{file_basename}"] = {'success': False, 'error': result.stderr}

                except Exception as e:
                    print(f"   ❌ 执行异常: {e}")
                    results[f"{data_type}_{file_basename}"] = {'success': False, 'error': str(e)}
            
            print(f"   📊 {data_type} 类型共处理 {processed_count}/{len(files)} 个文件")

        # 运行组合可视化（如果有匹配的文件组）
        if matched_groups:
            print(f"\n📊 处理组合规划数据...")
            
            for i, group in enumerate(matched_groups):
                frontend_file = group['frontend']
                corridor_file = group['corridor']
                backend_file = group['backend']
                
                script_name = self.data_types['combined']['script']
                script_path = os.path.join(self.visualization_tools_dir, script_name)

                if os.path.exists(script_path):
                    # 为组合可视化创建独立的子目录
                    combined_subdir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "output", "combined")
                    os.makedirs(combined_subdir, exist_ok=True)
                    
                    # 为每组文件生成唯一的输出文件名
                    group_timestamp = group['timestamp'].strftime('%Y%m%d_%H%M%S')
                    # 使用组中所有文件的时间戳来生成更具体的文件名
                    frontend_name = os.path.splitext(os.path.basename(frontend_file))[0]
                    output_file = f"combined_analysis_{frontend_name}.png"
                    output_path = os.path.join(combined_subdir, output_file)
                    
                    print(f"   📄 前端文件: {os.path.basename(frontend_file)}")
                    print(f"   📄 走廊文件: {os.path.basename(corridor_file)}")
                    print(f"   📄 后端文件: {os.path.basename(backend_file)}")

                    try:
                        # 运行组合可视化脚本
                        cmd = ['python3', script_path, 
                              '--frontend', frontend_file,
                              '--corridor', corridor_file,
                              '--backend', backend_file,
                              '--save', output_path, '--no-show']
                        print(f"   🔧 执行: {' '.join(cmd)}")

                        result = subprocess.run(cmd, capture_output=True, text=True, cwd=os.path.dirname(os.path.dirname(self.visualization_tools_dir)))

                        if result.returncode == 0:
                            print(f"   ✅ 成功生成: combined/{output_file}")
                            results[f'combined_group_{i+1}'] = {'success': True, 'output_file': output_path}
                        else:
                            print(f"   ❌ 执行失败: {result.stderr}")
                            results[f'combined_group_{i+1}'] = {'success': False, 'error': result.stderr}

                    except Exception as e:
                        print(f"   ❌ 执行异常: {e}")
                        results[f'combined_group_{i+1}'] = {'success': False, 'error': str(e)}
        elif all(session_data[dt] for dt in ['frontend', 'corridor', 'backend']):
            # 回退到原始匹配逻辑（处理所有可能的组合）
            print(f"\n📊 处理组合规划数据 (回退模式)...")
            
            # 获取所有文件并尝试多种组合
            frontend_files = session_data['frontend']
            corridor_files = session_data['corridor']
            backend_files = session_data['backend']
            
            # 按时间戳排序
            frontend_files.sort(key=lambda x: self.get_file_timestamp(x))
            corridor_files.sort(key=lambda x: self.get_file_timestamp(x))
            backend_files.sort(key=lambda x: self.get_file_timestamp(x))
            
            # 尝试处理最多5组（或文件数较少的那个）
            max_groups = min(5, len(frontend_files), len(corridor_files), len(backend_files))
            
            script_name = self.data_types['combined']['script']
            script_path = os.path.join(self.visualization_tools_dir, script_name)
            
            if os.path.exists(script_path):
                # 为组合可视化创建独立的子目录
                combined_subdir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "output", "combined")
                os.makedirs(combined_subdir, exist_ok=True)
                
                for i in range(max_groups):
                    if i < len(frontend_files) and i < len(corridor_files) and i < len(backend_files):
                        frontend_file = frontend_files[i]
                        corridor_file = corridor_files[i]
                        backend_file = backend_files[i]
                        
                        # 生成输出文件名
                        frontend_name = os.path.splitext(os.path.basename(frontend_file))[0]
                        output_file = f"combined_analysis_{frontend_name}.png"
                        output_path = os.path.join(combined_subdir, output_file)
                        
                        print(f"   📄 组 {i+1} 前端文件: {os.path.basename(frontend_file)}")
                        print(f"   📄 组 {i+1} 走廊文件: {os.path.basename(corridor_file)}")
                        print(f"   📄 组 {i+1} 后端文件: {os.path.basename(backend_file)}")

                        try:
                            # 运行组合可视化脚本
                            cmd = ['python3', script_path, 
                                  '--frontend', frontend_file,
                                  '--corridor', corridor_file,
                                  '--backend', backend_file,
                                  '--save', output_path, '--no-show']
                            print(f"   🔧 执行: {' '.join(cmd)}")

                            result = subprocess.run(cmd, capture_output=True, text=True, cwd=os.path.dirname(os.path.dirname(self.visualization_tools_dir)))

                            if result.returncode == 0:
                                print(f"   ✅ 成功生成: combined/{output_file}")
                                results[f'combined_fallback_{i+1}'] = {'success': True, 'output_file': output_path}
                            else:
                                print(f"   ❌ 执行失败: {result.stderr}")
                                results[f'combined_fallback_{i+1}'] = {'success': False, 'error': result.stderr}

                        except Exception as e:
                            print(f"   ❌ 执行异常: {e}")
                            results[f'combined_fallback_{i+1}'] = {'success': False, 'error': str(e)}

        # 输出摘要
        print(f"\n📋 会话 {session_key} 处理完成:")
        success_count = sum(1 for r in results.values() if r.get('success', False))
        total_count = len(results)
        print(f"   成功: {success_count}/{total_count}")

        successful_outputs = [r['output_file'] for r in results.values() if r.get('success', False)]
        if successful_outputs:
            print(f"   📁 输出文件:")
            for output in successful_outputs:
                # 显示相对路径
                rel_path = os.path.relpath(output, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
                print(f"      {rel_path}")

        return success_count > 0

    def run_latest_session(self, auto_approve=False):
        """运行最新会话的可视化"""
        sessions = self.organize_by_sessions()

        if not sessions:
            print("❌ 没有找到任何数据会话")
            return False

        # 获取最新会话
        latest_session = max(sessions.items(), key=lambda x: x[1]['timestamp'])
        session_key = latest_session[0]

        print(f"🎯 处理最新会话: {session_key}")
        return self.run_visualization_for_session(session_key, auto_approve)

    def clean_old_data(self, keep_days=7):
        """清理旧的数据文件"""
        cutoff_time = datetime.now().timestamp() - (keep_days * 24 * 3600)

        data_files = self.scan_data_files()
        cleaned_count = 0

        print(f"🧹 清理 {keep_days} 天前的数据文件...")

        all_files = []
        for files in data_files.values():
            all_files.extend(files)

        for file_path in all_files:
            if os.path.getmtime(file_path) < cutoff_time:
                try:
                    os.remove(file_path)
                    print(f"   🗑️  删除: {os.path.basename(file_path)}")
                    cleaned_count += 1
                except OSError as e:
                    print(f"   ❌ 删除失败: {file_path} - {e}")

        print(f"✅ 清理完成，删除了 {cleaned_count} 个文件")
        return cleaned_count

def main():
    """主函数"""
    import argparse

    # 计算默认数据目录路径
    script_dir = os.path.dirname(os.path.abspath(__file__))
    default_data_dir = os.path.abspath(os.path.join(script_dir, "..", "navigo_debug_data"))

    parser = argparse.ArgumentParser(description='Navigo 调试数据会话管理器')
    parser.add_argument('--data-dir', default=default_data_dir,
                       help='数据目录路径')

    subparsers = parser.add_subparsers(dest='command', help='可用命令')

    # 扫描命令
    scan_parser = subparsers.add_parser('scan', help='扫描并显示数据会话摘要')

    # 可视化命令
    viz_parser = subparsers.add_parser('visualize', help='运行可视化脚本')
    viz_parser.add_argument('--session', help='指定会话ID (格式: 20241217_14)')
    viz_parser.add_argument('--latest', action='store_true', help='处理最新会话')
    viz_parser.add_argument('--auto-approve', action='store_true', help='自动确认所有操作')

    # 清理命令
    clean_parser = subparsers.add_parser('clean', help='清理旧的数据文件')
    clean_parser.add_argument('--keep-days', type=int, default=7, help='保留天数 (默认: 7天)')

    args = parser.parse_args()

    # 创建管理器
    manager = NavigoDataSessionManager(args.data_dir)

    if args.command == 'scan' or args.command is None:
        manager.print_session_summary()

    elif args.command == 'visualize':
        if args.session:
            manager.run_visualization_for_session(args.session, args.auto_approve)
        elif args.latest:
            manager.run_latest_session(args.auto_approve)
        else:
            print("请指定 --session <session_id> 或 --latest")
            sys.exit(1)

    elif args.command == 'clean':
        manager.clean_old_data(args.keep_days)

    else:
        parser.print_help()

if __name__ == "__main__":
    main()



















