#!/usr/bin/env python3
"""
Navigo 数据目录重组工具

⚠️  DEPRECATED: 此工具已被弃用
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
请使用新的统一CLI工具代替:
    python3 ../navigo_analyze.py --latest

新工具的优势:
    - 无需手动复制数据文件
    - 自动组织和处理数据
    - 更快的执行速度
    - 更好的错误处理

此脚本仅作为向后兼容保留
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

原功能：
- 按时间和类型重新组织混乱的数据文件
- 创建清晰的目录结构
- 支持数据归档和备份
"""

import warnings
warnings.warn(
    "organize_data.py is deprecated. Please use 'python3 navigo_analyze.py --latest' instead.",
    DeprecationWarning,
    stacklevel=2
)

import os
import shutil
import json
import glob
import re
from datetime import datetime
from collections import defaultdict

class NavigoDataOrganizer:
    def __init__(self, data_dir=None):
        # 如果没有指定data_dir，则使用相对于脚本位置的默认路径
        if data_dir is None:
            # 获取当前脚本所在的目录
            script_dir = os.path.dirname(os.path.abspath(__file__))
            # 导航到父目录，然后进入navigo_debug_data
            self.data_dir = os.path.abspath(os.path.join(script_dir, "..", "navigo_debug_data"))
        else:
            self.data_dir = os.path.abspath(data_dir)
            
        self.organized_dir = os.path.join(self.data_dir, "organized")
        self.archive_dir = os.path.join(self.data_dir, "archive")

    def get_file_info(self, file_path):
        """提取文件信息"""
        file_name = os.path.basename(file_path)

        # 确定数据类型
        data_type = 'unknown'
        if re.search(r'frontend', file_name, re.IGNORECASE):
            data_type = 'frontend'
        elif re.search(r'corridor', file_name, re.IGNORECASE):
            data_type = 'corridor'
        elif re.search(r'backend', file_name, re.IGNORECASE):
            data_type = 'backend'

        # 提取时间戳
        timestamp = None

        # 从文件名提取
        timestamp_match = re.search(r'(\d{8}_\d{6})', file_name)
        if timestamp_match:
            try:
                timestamp = datetime.strptime(timestamp_match.group(1), '%Y%m%d_%H%M%S')
            except ValueError:
                pass

        # 从文件内容提取（JSON文件）
        if not timestamp and file_path.endswith('.json'):
            try:
                with open(file_path, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    timestamp_str = data.get('timestamp', '')
                    if timestamp_str:
                        try:
                            timestamp = datetime.strptime(timestamp_str, '%Y-%m-%d %H:%M:%S')
                        except ValueError:
                            pass
            except (json.JSONDecodeError, FileNotFoundError, UnicodeDecodeError):
                pass

        # 使用文件修改时间作为备用
        if not timestamp:
            timestamp = datetime.fromtimestamp(os.path.getmtime(file_path))

        return {
            'type': data_type,
            'timestamp': timestamp,
            'original_path': file_path,
            'file_name': file_name
        }

    def get_file_session_id(self, file_path):
        """从文件名提取会话ID（用于匹配相关文件）"""
        file_name = os.path.basename(file_path)
        
        # 尝试提取会话ID（通常是时间戳后面的部分）
        # 例如: 20241217_141211_frontend_20241217_141211_frontend_1234.json
        # 会话ID可能是: 20241217_141211_frontend_1234
        
        # 查找两个连续的时间戳模式
        matches = re.findall(r'(\d{8}_\d{6})', file_name)
        if len(matches) >= 1:
            # 使用第一个时间戳作为会话标识（这是规划会话的时间戳）
            return matches[0]
        
        # 回退到使用完整文件名的一部分
        name_without_ext = os.path.splitext(file_name)[0]
        return name_without_ext

    def group_related_files(self, file_infos):
        """将相关文件分组到同一会话中"""
        # 按会话ID分组
        session_groups = defaultdict(list)
        
        for info in file_infos:
            session_id = self.get_file_session_id(info['original_path'])
            session_groups[session_id].append(info)
        
        # 进一步按时间窗口分组（同一小时内）
        time_window_groups = defaultdict(list)
        for session_id, infos in session_groups.items():
            # 使用第一个文件的时间作为代表
            representative_time = infos[0]['timestamp']
            time_key = representative_time.strftime('%Y%m%d_%H')
            time_window_groups[time_key].extend(infos)
        
        return time_window_groups

    def organize_files(self, dry_run=False):
        """重组文件"""
        print("🔍 扫描数据文件...")

        # 查找所有JSON文件
        json_files = glob.glob(os.path.join(self.data_dir, "**/*.json"), recursive=True)

        # 过滤掉已经在organized或archive目录中的文件
        json_files = [f for f in json_files
                     if not f.startswith(self.organized_dir)
                     and not f.startswith(self.archive_dir)]

        if not json_files:
            print("❌ 没有找到需要整理的JSON文件")
            return

        print(f"   📊 找到 {len(json_files)} 个文件")

        # 获取所有文件信息
        file_infos = [self.get_file_info(file_path) for file_path in json_files]

        # 按日期分组
        date_groups = defaultdict(list)
        for info in file_infos:
            date_key = info['timestamp'].strftime('%Y-%m-%d')
            date_groups[date_key].append(info)

        print(f"   📅 涵盖 {len(date_groups)} 个日期")

        # 重组文件
        moved_count = 0
        for date_key, file_infos in sorted(date_groups.items()):
            print(f"\n📅 处理日期: {date_key} ({len(file_infos)} 个文件)")

            date_obj = datetime.strptime(date_key, '%Y-%m-%d')

            # 创建日期目录
            date_dir = os.path.join(
                self.organized_dir,
                date_obj.strftime('%Y'),
                date_obj.strftime('%m'),
                date_obj.strftime('%d')
            )

            if not dry_run:
                os.makedirs(date_dir, exist_ok=True)

            # 将相关文件分组
            related_groups = self.group_related_files(file_infos)
            
            for group_key, group_infos in related_groups.items():
                if len(group_infos) > 1:
                    print(f"   🔗 发现相关文件组 ({len(group_infos)} 个文件):")
                    for info in group_infos:
                        print(f"      - {info['file_name']} ({info['type']})")
                
                # 按类型再分组，并创建相应的子目录
                type_groups = defaultdict(list)
                for info in group_infos:
                    type_groups[info['type']].append(info)

                for data_type, type_infos in type_groups.items():
                    # 为每种数据类型创建独立的子目录
                    type_dir = os.path.join(date_dir, data_type)

                    if not dry_run:
                        os.makedirs(type_dir, exist_ok=True)

                    print(f"   📂 {data_type}: {len(type_infos)} 个文件")

                    for info in type_infos:
                        # 生成新的文件名（包含完整时间戳和原始文件名信息）
                        timestamp_str = info['timestamp'].strftime('%Y%m%d_%H%M%S')
                        original_name = os.path.splitext(info['file_name'])[0]
                        # 确保文件名遵循统一的命名约定
                        new_file_name = f"{timestamp_str}_{data_type}_{original_name}.json"
                        new_file_path = os.path.join(type_dir, new_file_name)

                        print(f"      📄 {info['file_name']} -> {new_file_name}")

                        if not dry_run:
                            try:
                                shutil.move(info['original_path'], new_file_path)
                                moved_count += 1
                            except Exception as e:
                                print(f"         ❌ 移动失败: {e}")

        if dry_run:
            print(f"\n🔍 模拟运行完成，将移动 {len(json_files)} 个文件")
        else:
            print(f"\n✅ 重组完成，成功移动 {moved_count} 个文件")

    def create_organized_structure(self):
        """创建有序的目录结构"""
        print("📁 创建组织目录结构...")

        # 创建主目录
        os.makedirs(self.organized_dir, exist_ok=True)
        os.makedirs(self.archive_dir, exist_ok=True)

        # 按日期和类型创建子目录
        for year in range(2024, 2026):  # 可根据需要调整范围
            year_dir = os.path.join(self.organized_dir, str(year))
            for month in range(1, 13):
                month_dir = os.path.join(year_dir, f"{month:02d}")
                for day in range(1, 32):  # 简化处理，实际会根据文件创建
                    day_dir = os.path.join(month_dir, f"{day:02d}")
                    # 不预先创建所有目录，而是根据需要创建

        print(f"   ✅ 组织目录: {self.organized_dir}")
        print(f"   ✅ 归档目录: {self.archive_dir}")

    def create_index_file(self):
        """创建索引文件"""
        print("📋 创建数据索引...")

        index_data = {
            'created_at': datetime.now().isoformat(),
            'structure': {},
            'statistics': {
                'total_files': 0,
                'by_type': {},
                'by_date': {}
            }
        }

        # 遍历组织目录
        for root, dirs, files in os.walk(self.organized_dir):
            if files:
                rel_path = os.path.relpath(root, self.organized_dir)

                # 解析路径 (year/month/day/type)
                path_parts = rel_path.split(os.sep)
                if len(path_parts) == 4:
                    year, month, day, data_type = path_parts
                    date_key = f"{year}-{month}-{day}"

                    if date_key not in index_data['structure']:
                        index_data['structure'][date_key] = {}

                    index_data['structure'][date_key][data_type] = {
                        'file_count': len(files),
                        'files': files
                    }

                    # 更新统计
                    index_data['statistics']['total_files'] += len(files)
                    index_data['statistics']['by_type'][data_type] = \
                        index_data['statistics']['by_type'].get(data_type, 0) + len(files)
                    index_data['statistics']['by_date'][date_key] = \
                        index_data['statistics']['by_date'].get(date_key, 0) + len(files)

        # 保存索引
        index_file = os.path.join(self.organized_dir, 'data_index.json')
        with open(index_file, 'w', encoding='utf-8') as f:
            json.dump(index_data, f, indent=2, ensure_ascii=False)

        print(f"   ✅ 索引文件: {index_file}")
        print(f"   📊 总文件数: {index_data['statistics']['total_files']}")

    def print_summary(self):
        """打印重组摘要"""
        print("\n" + "=" * 60)
        print("📊 数据重组摘要")
        print("=" * 60)

        if not os.path.exists(self.organized_dir):
            print("❌ 组织目录不存在，请先运行重组")
            return

        index_file = os.path.join(self.organized_dir, 'data_index.json')
        if os.path.exists(index_file):
            with open(index_file, 'r', encoding='utf-8') as f:
                index_data = json.load(f)

            stats = index_data['statistics']
            print(f"📁 组织目录: {self.organized_dir}")
            print(f"📅 创建时间: {index_data['created_at']}")
            print(f"📊 总文件数: {stats['total_files']}")

            print("\n按类型统计:")
            for data_type, count in stats['by_type'].items():
                print(f"   📂 {data_type}: {count} 个文件")

            print(f"\n按日期统计 (显示前10个):")
            sorted_dates = sorted(stats['by_date'].items(),
                                key=lambda x: x[0], reverse=True)
            for date_key, count in sorted_dates[:10]:
                print(f"   📅 {date_key}: {count} 个文件")

            if len(sorted_dates) > 10:
                print(f"   ... 还有 {len(sorted_dates) - 10} 个日期")
        else:
            print("❌ 找不到索引文件")

def main():
    """主函数"""
    import argparse

    # 计算默认数据目录路径
    script_dir = os.path.dirname(os.path.abspath(__file__))
    default_data_dir = os.path.abspath(os.path.join(script_dir, "..", "navigo_debug_data"))

    parser = argparse.ArgumentParser(description='Navigo 数据目录重组工具')
    parser.add_argument('--data-dir', default=default_data_dir,
                       help='数据目录路径')
    parser.add_argument('--dry-run', action='store_true',
                       help='模拟运行，不实际移动文件')
    parser.add_argument('--summary', action='store_true',
                       help='只显示重组摘要')

    args = parser.parse_args()

    organizer = NavigoDataOrganizer(args.data_dir)

    if args.summary:
        organizer.print_summary()
    else:
        print("🗂️  Navigo 数据目录重组工具")
        print("=" * 40)

        organizer.create_organized_structure()
        organizer.organize_files(dry_run=args.dry_run)

        if not args.dry_run:
            organizer.create_index_file()
            print("")
            organizer.print_summary()

if __name__ == "__main__":
    main()