#!/usr/bin/env python3
"""
组合可视化工具

用途：在同一张图上显示前端搜索、走廊生成和后端优化的结果
输入：三个相关的JSON文件（前端、走廊、后端）
输出：组合可视化图表和分析报告
"""

import json
import matplotlib.pyplot as plt
import numpy as np
import argparse
import os
from datetime import datetime
import matplotlib
import warnings
from matplotlib.patches import Polygon

# 禁用matplotlib字体警告
warnings.filterwarnings('ignore', category=UserWarning, module='matplotlib')
warnings.filterwarnings('ignore', message='.*Glyph.*missing.*')
warnings.filterwarnings('ignore', message='.*font.*')

# 导入字体配置
try:
    import sys
    import os
    # 添加config目录到路径
    config_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config')
    if config_dir not in sys.path:
        sys.path.insert(0, config_dir)
    from font_config import CHINESE_FONT_AVAILABLE, LABELS
except ImportError:
    # 如果无法导入字体配置，使用英文配置
    CHINESE_FONT_AVAILABLE = False
    LABELS = {
        'combined_title': 'Combined Planning Results',
        'x_label': 'X (meters)',
        'y_label': 'Y (meters)',
        'frontend_label': 'Frontend Search',
        'corridor_label': 'Corridor',
        'backend_label': 'Backend Optimization',
        'start_label': 'Start',
        'end_label': 'Goal',
        'no_data': 'Warning: No data found',
        'saved_to': 'Combined plot saved to',
    }

class CombinedVisualizer:
    def __init__(self, frontend_file=None, corridor_file=None, backend_file=None):
        """初始化组合可视化器"""
        self.frontend_data = self.load_data(frontend_file) if frontend_file else None
        self.corridor_data = self.load_data(corridor_file) if corridor_file else None
        self.backend_data = self.load_data(backend_file) if backend_file else None
        
    def load_data(self, json_file):
        """加载JSON数据"""
        if not json_file or not os.path.exists(json_file):
            return None
        try:
            with open(json_file, 'r', encoding='utf-8') as f:
                return json.load(f)
        except Exception as e:
            print(f"无法加载JSON文件 {json_file}: {e}")
            return None

    def extract_trajectory(self, data, key='trajectory_points'):
        """从数据中提取轨迹点"""
        if not data:
            return []
        # 优先使用指定的键
        if key and key in data:
            return data[key]
        # 对于后端优化数据，优先使用optimized_path而不是input_trajectory
        if data.get('module') == 'backend_optimization':
            backend_keys = ['optimized_path', 'trajectory_points', 'optimized_trajectory', 'trajectory_segment', 'input_trajectory']
            for alt_key in backend_keys:
                if alt_key in data:
                    return data[alt_key]
        # 对于其他数据，尝试常见的轨迹键
        else:
            common_keys = ['trajectory_points', 'optimized_trajectory', 'input_trajectory', 'trajectory_segment']
            for alt_key in common_keys:
                if alt_key in data:
                    return data[alt_key]
        return []

    def plot_combined(self, save_path=None, no_show=False):
        """绘制组合图"""
        # 创建图形
        fig, ax = plt.subplots(1, 1, figsize=(15, 12))
        
        has_data = False
        
        # 绘制走廊数据
        corridor_polygon_added = False
        if self.corridor_data:
            corridor_per_point = self.corridor_data.get('corridor_per_point', [])
            if corridor_per_point:
                # 使用不同颜色绘制每个轨迹点的走廊
                colors = plt.cm.Set3(np.linspace(0, 1, max(len(corridor_per_point), 1)))
                
                # 显示所有走廊点（不再抽样）
                for i, point_corridor in enumerate(corridor_per_point):
                    vertices = point_corridor.get('corridor_vertices', [])
                    if len(vertices) >= 3:
                        # 走廊顶点已经是在世界坐标系下的坐标，直接使用
                        poly_vertices = np.array(vertices)
                        # 闭合多边形
                        if not np.allclose(poly_vertices[0], poly_vertices[-1]):
                            poly_vertices = np.vstack([poly_vertices, poly_vertices[0]])

                        # 只为第一个走廊添加图例标签
                        polygon_label = LABELS.get('corridor_label', 'Corridor') if not corridor_polygon_added else ""
                        polygon = Polygon(poly_vertices, alpha=0.15,
                                        facecolor=colors[i % len(colors)],
                                        edgecolor='darkblue', linewidth=0.2,
                                        label=polygon_label)
                        ax.add_patch(polygon)
                        corridor_polygon_added = True
                        
                        # 在每个走廊中心点添加方向箭头
                        traj_point = point_corridor.get('traj_point', {})
                        if traj_point:
                            px = traj_point.get('x', 0)
                            py = traj_point.get('y', 0)
                            theta = traj_point.get('theta', 0)
                            
                            # 绘制方向箭头
                            arrow_len = 0.5
                            ax.arrow(px, py, 
                                    arrow_len * np.cos(theta), arrow_len * np.sin(theta),
                                    head_width=0.1, head_length=0.15,
                                    fc='red', ec='red', alpha=0.7, zorder=4)
                has_data = True

        # 绘制前端搜索轨迹
        frontend_arrow_added = False
        frontend_start_added = False
        frontend_end_added = False
        frontend_start_heading_added = False
        frontend_end_heading_added = False
        if self.frontend_data:
            frontend_traj = self.extract_trajectory(self.frontend_data)
            if frontend_traj:
                x_coords = [point['x'] for point in frontend_traj]
                y_coords = [point['y'] for point in frontend_traj]
                theta_coords = [point.get('theta', 0) for point in frontend_traj]
                
                ax.plot(x_coords, y_coords, 'b-', linewidth=2, 
                       label=LABELS.get('frontend_label', 'Frontend Search'), alpha=0.8)
                ax.scatter(x_coords, y_coords, c='blue', s=20, alpha=0.6, zorder=3)
                
                # 添加方向箭头（每隔5个点显示一个）
                arrow_step = max(1, len(x_coords) // 20)
                for i in range(0, len(x_coords), arrow_step):
                    if i < len(x_coords):
                        if not frontend_arrow_added:
                            ax.arrow(x_coords[i], y_coords[i],
                                    0.3 * np.cos(theta_coords[i]), 0.3 * np.sin(theta_coords[i]),
                                    head_width=0.1, head_length=0.15,
                                    fc='blue', ec='blue', alpha=0.7, zorder=4,
                                    label='Frontend Trajectory Direction')
                            frontend_arrow_added = True
                        else:
                            ax.arrow(x_coords[i], y_coords[i],
                                    0.3 * np.cos(theta_coords[i]), 0.3 * np.sin(theta_coords[i]),
                                    head_width=0.1, head_length=0.15,
                                    fc='blue', ec='blue', alpha=0.7, zorder=4)
                
                # 特殊标记起点和终点的航向角
                if len(x_coords) > 0:
                    # 起点（绿色大圆点 + 长箭头）
                    start_x, start_y = x_coords[0], y_coords[0]
                    start_theta = theta_coords[0]
                    if not frontend_start_added:
                        ax.plot(start_x, start_y, 'go', markersize=18, markeredgecolor='black', markeredgewidth=3, zorder=8,
                               label='Frontend Start Point')
                        frontend_start_added = True
                    else:
                        ax.plot(start_x, start_y, 'go', markersize=18, markeredgecolor='black', markeredgewidth=3, zorder=8)
                    
                    # 更长更明显的起始方向箭头
                    if not frontend_start_heading_added:
                        ax.arrow(start_x, start_y,
                                1.0 * np.cos(start_theta), 1.0 * np.sin(start_theta),
                                head_width=0.25, head_length=0.4,
                                fc='lime', ec='darkgreen', linewidth=3, alpha=1.0, zorder=7,
                                label='Frontend Start Heading')
                        frontend_start_heading_added = True
                    else:
                        ax.arrow(start_x, start_y,
                                1.0 * np.cos(start_theta), 1.0 * np.sin(start_theta),
                                head_width=0.25, head_length=0.4,
                                fc='lime', ec='darkgreen', linewidth=3, alpha=1.0, zorder=7)
                
                if len(x_coords) > 1:
                    # 终点（红色大圆点 + 长箭头）
                    end_x, end_y = x_coords[-1], y_coords[-1]
                    end_theta = theta_coords[-1]
                    if not frontend_end_added:
                        ax.plot(end_x, end_y, 'ro', markersize=18, markeredgecolor='black', markeredgewidth=3, zorder=8,
                               label='Frontend End Point')
                        frontend_end_added = True
                    else:
                        ax.plot(end_x, end_y, 'ro', markersize=18, markeredgecolor='black', markeredgewidth=3, zorder=8)
                    
                    # 更长更明显的终点方向箭头
                    if not frontend_end_heading_added:
                        ax.arrow(end_x, end_y,
                                1.0 * np.cos(end_theta), 1.0 * np.sin(end_theta),
                                head_width=0.25, head_length=0.4,
                                fc='red', ec='darkred', linewidth=3, alpha=1.0, zorder=7,
                                label='Frontend End Heading')
                        frontend_end_heading_added = True
                    else:
                        ax.arrow(end_x, end_y,
                                1.0 * np.cos(end_theta), 1.0 * np.sin(end_theta),
                                head_width=0.25, head_length=0.4,
                                fc='red', ec='darkred', linewidth=3, alpha=1.0, zorder=7)
                
                has_data = True

        # 绘制后端优化轨迹
        backend_arrow_added = False
        backend_start_added = False
        backend_end_added = False
        backend_start_heading_added = False
        backend_end_heading_added = False
        if self.backend_data:
            # 对于后端数据，明确指定使用optimized_path
            backend_traj = self.extract_trajectory(self.backend_data, 'optimized_path')
            if not backend_traj:
                # 如果没有optimized_path，回退到通用方法
                backend_traj = self.extract_trajectory(self.backend_data)
                
            if backend_traj:
                x_coords = [point['x'] for point in backend_traj]
                y_coords = [point['y'] for point in backend_traj]
                # 后端数据可能有不同的字段名
                theta_coords = []
                for point in backend_traj:
                    theta = point.get('theta', point.get('yaw', 0))
                    theta_coords.append(theta)
                
                ax.plot(x_coords, y_coords, 'g-', linewidth=2, 
                       label=LABELS.get('backend_label', 'Backend Optimization'), alpha=0.8)
                ax.scatter(x_coords, y_coords, c='green', s=20, alpha=0.6, zorder=3)
                
                # 添加方向箭头（每隔5个点显示一个）
                arrow_step = max(1, len(x_coords) // 20)
                for i in range(0, len(x_coords), arrow_step):
                    if i < len(x_coords):
                        if not backend_arrow_added:
                            ax.arrow(x_coords[i], y_coords[i],
                                    0.3 * np.cos(theta_coords[i]), 0.3 * np.sin(theta_coords[i]),
                                    head_width=0.1, head_length=0.15,
                                    fc='green', ec='green', alpha=0.7, zorder=4,
                                    label='Backend Trajectory Direction')
                            backend_arrow_added = True
                        else:
                            ax.arrow(x_coords[i], y_coords[i],
                                    0.3 * np.cos(theta_coords[i]), 0.3 * np.sin(theta_coords[i]),
                                    head_width=0.1, head_length=0.15,
                                    fc='green', ec='green', alpha=0.7, zorder=4)
                
                # 特殊标记起点和终点的航向角（使用更醒目的样式以区分前端）
                if len(x_coords) > 0:
                    # 起点（青色大圆点 + 长箭头 + 星形标记）
                    start_x, start_y = x_coords[0], y_coords[0]
                    start_theta = theta_coords[0]
                    if not backend_start_added:
                        ax.plot(start_x, start_y, 'c*', markersize=20, markeredgecolor='black', markeredgewidth=2, zorder=8,
                               label='Backend Start Point')
                        backend_start_added = True
                    else:
                        ax.plot(start_x, start_y, 'c*', markersize=20, markeredgecolor='black', markeredgewidth=2, zorder=8)
                    
                    # 更长更明显的起始方向箭头
                    if not backend_start_heading_added:
                        ax.arrow(start_x, start_y,
                                1.2 * np.cos(start_theta), 1.2 * np.sin(start_theta),
                                head_width=0.3, head_length=0.5,
                                fc='cyan', ec='darkcyan', linewidth=4, alpha=1.0, zorder=7,
                                label='Backend Start Heading')
                        backend_start_heading_added = True
                    else:
                        ax.arrow(start_x, start_y,
                                1.2 * np.cos(start_theta), 1.2 * np.sin(start_theta),
                                head_width=0.3, head_length=0.5,
                                fc='cyan', ec='darkcyan', linewidth=4, alpha=1.0, zorder=7)
                
                if len(x_coords) > 1:
                    # 终点（洋红色大圆点 + 长箭头 + 星形标记）
                    end_x, end_y = x_coords[-1], y_coords[-1]
                    end_theta = theta_coords[-1]
                    if not backend_end_added:
                        ax.plot(end_x, end_y, 'm*', markersize=20, markeredgecolor='black', markeredgewidth=2, zorder=8,
                               label='Backend End Point')
                        backend_end_added = True
                    else:
                        ax.plot(end_x, end_y, 'm*', markersize=20, markeredgecolor='black', markeredgewidth=2, zorder=8)
                    
                    # 更长更明显的终点方向箭头
                    if not backend_end_heading_added:
                        ax.arrow(end_x, end_y,
                                1.2 * np.cos(end_theta), 1.2 * np.sin(end_theta),
                                head_width=0.3, head_length=0.5,
                                fc='magenta', ec='darkmagenta', linewidth=4, alpha=1.0, zorder=7,
                                label='Backend End Heading')
                        backend_end_heading_added = True
                    else:
                        ax.arrow(end_x, end_y,
                                1.2 * np.cos(end_theta), 1.2 * np.sin(end_theta),
                                head_width=0.3, head_length=0.5,
                                fc='magenta', ec='darkmagenta', linewidth=4, alpha=1.0, zorder=7)
                
                has_data = True

        if not has_data:
            print(LABELS.get('no_data', 'Warning: No data found'))
            return

        ax.set_xlabel(LABELS.get('x_label', 'X (meters)'))
        ax.set_ylabel(LABELS.get('y_label', 'Y (meters)'))
        ax.set_title(LABELS.get('combined_title', 'Combined Planning Results'))
        ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        ax.grid(True, alpha=0.3)
        ax.axis('equal')

        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"{LABELS.get('saved_to', 'Combined plot saved to')}: {save_path}")

        if not no_show:
            plt.show()

    def print_statistics(self):
        """打印组合统计信息"""
        print(f"\n{'='*60}")
        print(f"组合规划分析报告")
        print(f"{'='*60}")
        
        # 显示使用的文件
        if self.frontend_data:
            print(f"前端文件: {self.frontend_data.get('session_id', 'N/A')}")
        if self.corridor_data:
            print(f"走廊文件: {self.corridor_data.get('session_id', 'N/A')}")
        if self.backend_data:
            print(f"后端文件: {self.backend_data.get('session_id', 'N/A')}")
            
        print(f"\n时间戳: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        
        # 分别显示各部分的统计数据
        if self.frontend_data:
            print(f"\n--- 前端搜索统计 ---")
            result = self.frontend_data.get('search_result', {})
            print(f"  搜索时间: {result.get('search_time_ms', 0):.2f} ms")
            print(f"  路径长度: {result.get('path_length_m', 0):.3f} 米")
            trajectory = self.extract_trajectory(self.frontend_data)
            if trajectory:
                print(f"  轨迹点数: {len(trajectory)}")
                
        if self.backend_data:
            print(f"\n--- 后端优化统计 ---")
            result = self.backend_data.get('optimization_result', {})
            print(f"  优化时间: {result.get('optimization_time_ms', 0):.2f} ms")
            # 对于后端数据，优先使用optimized_path
            trajectory = self.extract_trajectory(self.backend_data, 'optimized_path')
            if not trajectory:
                trajectory = self.extract_trajectory(self.backend_data)
            if trajectory:
                print(f"  轨迹点数: {len(trajectory)}")
                
        if self.corridor_data:
            print(f"\n--- 走廊生成统计 ---")
            result = self.corridor_data.get('generation_result', {})
            print(f"  生成时间: {result.get('processing_time_ms', 0):.2f} ms")
            print(f"  走廊段数: {result.get('total_segments', 0)}")
            print(f"  总顶点数: {result.get('total_vertices', 0)}")

def main():
    parser = argparse.ArgumentParser(description='组合规划结果可视化工具')
    parser.add_argument('--frontend', help='前端搜索JSON文件路径')
    parser.add_argument('--corridor', help='走廊生成JSON文件路径')
    parser.add_argument('--backend', help='后端优化JSON文件路径')
    parser.add_argument('--save', help='保存图像的路径')
    parser.add_argument('--no-show', action='store_true', help='不显示图像，仅保存')
    
    args = parser.parse_args()
    
    # 创建可视化器
    visualizer = CombinedVisualizer(args.frontend, args.corridor, args.backend)
    
    # 打印统计信息
    visualizer.print_statistics()
    
    # 绘制组合图
    visualizer.plot_combined(args.save, args.no_show)

if __name__ == "__main__":
    main()