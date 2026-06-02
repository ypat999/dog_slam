#!/usr/bin/env python3
"""
前端搜索可视化工具

用途：分析前端搜索算法的轨迹结果
输入：前端搜索导出的JSON文件
输出：可视化图表和分析报告
"""

import json
import matplotlib.pyplot as plt
import numpy as np
import argparse
import os
from datetime import datetime
import matplotlib
import warnings

# 禁用matplotlib字体警告（在导入时就禁用）
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
        'trajectory_title': 'Frontend Search Trajectory',
        'angle_title': 'Angle Changes Along Trajectory',
        'x_label': 'X (meters)',
        'y_label': 'Y (meters)',
        'angle_label': 'Angle (degrees)',
        'distance_label': 'Cumulative Distance (meters)',
        'trajectory_label': 'Search Trajectory',
        'start_label': 'Start',
        'end_label': 'Goal',
        'no_trajectory': 'Warning: No trajectory data found',
        'saved_to': 'Trajectory plot saved to',
    }

class FrontendSearchVisualizer:
    def __init__(self, json_file):
        """初始化可视化器"""
        self.json_file = json_file
        self.data = self.load_data()

    def load_data(self):
        """加载JSON数据"""
        try:
            with open(self.json_file, 'r', encoding='utf-8') as f:
                return json.load(f)
        except Exception as e:
            raise ValueError(f"无法加载JSON文件 {self.json_file}: {e}")

    def plot_trajectory(self, save_path=None, no_show=False):
        """绘制搜索轨迹"""
        trajectory = self.data.get('trajectory_points', [])
        start_state = self.data.get('start_state', [])
        goal_state = self.data.get('goal_state', [])

        if not trajectory:
            print(LABELS.get('no_trajectory', 'Warning: No trajectory data found'))
            return

        # 提取x, y坐标和角度
        x_coords = [point['x'] for point in trajectory]
        y_coords = [point['y'] for point in trajectory]
        theta_coords = [point['theta'] for point in trajectory]

        # 创建子图
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))

        # 子图1: 轨迹路径
        ax1.plot(x_coords, y_coords, 'b-', linewidth=2, label=LABELS.get('trajectory_label', 'Trajectory'), alpha=0.8)
        # ax1.scatter(x_coords[::5], y_coords[::5], c='blue', s=30, alpha=0.6)
        ax1.scatter(x_coords, y_coords, c='blue', s=10, alpha=0.8)   # 增加透明度

        # ---------- 方向箭头 ----------
        # 箭头长度（可调，单位：米）
        arrow_len = 1.0
        step = 2 

        u = np.cos(theta_coords[::step]) * arrow_len
        v = np.sin(theta_coords[::step]) * arrow_len

        ax1.quiver(
            x_coords[::step],
            y_coords[::step],
            u,
            v,
            angles='xy',
            scale_units='xy',
            scale=1,
            color='red',
            width=0.005,
            alpha=0.8,
            label='Heading'
        )
        # 标记起点和终点
        if start_state:
            ax1.plot(start_state[0], start_state[1], 'go', markersize=10, label=LABELS.get('start_label', 'Start'))
        if goal_state:
            ax1.plot(goal_state[0], goal_state[1], 'ro', markersize=10, label=LABELS.get('end_label', 'Goal'))

        ax1.set_xlabel(LABELS.get('x_label', 'X (meters)'))
        ax1.set_ylabel(LABELS.get('y_label', 'Y (meters)'))
        ax1.set_title(LABELS.get('trajectory_title', 'Search Trajectory'))
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')

        # 子图2: 角度变化
        distances = [0]
        for i in range(1, len(trajectory)):
            dx = x_coords[i] - x_coords[i-1]
            dy = y_coords[i] - y_coords[i-1]
            distances.append(distances[-1] + np.sqrt(dx**2 + dy**2))

        ax2.plot(distances, np.degrees(theta_coords), 'r-', linewidth=2, label=LABELS.get('angle_label', 'Angle'))
        ax2.set_xlabel(LABELS.get('distance_label', 'Distance (meters)'))
        ax2.set_ylabel(LABELS.get('angle_label', 'Angle (degrees)'))
        ax2.set_title(LABELS.get('angle_title', 'Angle Changes'))
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"{LABELS.get('saved_to', 'Saved to')}: {save_path}")

        if not no_show:
            plt.show()

    def print_statistics(self):
        """打印搜索统计信息"""
        print(f"\n{'='*60}")
        print(f"前端搜索分析报告")
        print(f"{'='*60}")

        # 基本信息
        print(f"会话ID: {self.data.get('session_id', 'N/A')}")
        print(f"时间戳: {self.data.get('timestamp', 'N/A')}")
        print(f"搜索器名称: {self.data.get('searcher_name', 'N/A')}")
        print(f"功能ID: {self.data.get('function_id', 'N/A')}")

        # 搜索结果
        result = self.data.get('search_result', {})
        print(f"\n搜索结果:")
        print(f"  成功: {'✅' if result.get('success') else '❌'}")
        if not result.get('success'):
            print(f"  失败原因: {result.get('failure_reason', 'N/A')}")

        # 性能指标
        print(f"\n性能指标:")
        print(f"  搜索时间: {result.get('search_time_ms', 0):.2f} ms")
        print(f"  Shot时间: {result.get('shot_time_ms', 0):.2f} ms")
        print(f"  总时间: {result.get('total_time_ms', 0):.2f} ms")
        print(f"  扩展节点数: {result.get('nodes_expanded', 0)}")
        print(f"  迭代次数: {result.get('iteration_count', 0)}")

        # 轨迹质量
        print(f"\n轨迹质量:")
        print(f"  路径长度: {result.get('path_length_m', 0):.3f} 米")
        print(f"  直线距离: {result.get('straight_line_distance_m', 0):.3f} 米")
        straight_dist = result.get('straight_line_distance_m', 1)
        if straight_dist > 0:
            efficiency = straight_dist / max(result.get('path_length_m', 1), straight_dist)
            print(f"  路径效率: {efficiency:.3f} ({efficiency*100:.1f}%)")
        print(f"  平滑度: {result.get('path_smoothness', 0):.4f}")
        print(f"  平均曲率: {result.get('average_curvature', 0):.4f} (1/m)")
        print(f"  最大曲率: {result.get('max_curvature', 0):.4f} (1/m)")

        # 上下文信息
        context = self.data.get('context', {})
        print(f"\n上下文信息:")
        print(f"  目标预测路径大小: {context.get('goal_prediction_size', 0)}")
        print(f"  障碍物预测路径大小: {context.get('obstacle_prediction_size', 0)}")
        print(f"  搜索维度: {context.get('search_dimension', 'N/A')}")
        print(f"  运动学模型: {context.get('kinodynamic_model', 'N/A')}")

        # 轨迹统计
        trajectory = self.data.get('trajectory_points', [])
        if trajectory:
            print(f"\n轨迹统计:")
            print(f"  轨迹点数: {len(trajectory)}")
            x_coords = [p['x'] for p in trajectory]
            y_coords = [p['y'] for p in trajectory]
            print(f"  X范围: {min(x_coords):.3f} ~ {max(x_coords):.3f} 米")
            print(f"  Y范围: {min(y_coords):.3f} ~ {max(y_coords):.3f} 米")

    def analyze_performance(self):
        """性能分析"""
        result = self.data.get('search_result', {})

        print(f"\n{'='*60}")
        print(f"性能分析")
        print(f"{'='*60}")

        # 时间效率分析
        search_time = result.get('search_time_ms', 0)
        nodes_expanded = result.get('nodes_expanded', 1)

        if nodes_expanded > 0:
            time_per_node = search_time / nodes_expanded
            print(f"每节点搜索时间: {time_per_node:.4f} ms/node")

        # 空间效率分析
        path_length = result.get('path_length_m', 0)
        straight_dist = result.get('straight_line_distance_m', 1)

        if straight_dist > 0 and path_length > 0:
            detour_ratio = path_length / straight_dist
            print(f"绕路比例: {detour_ratio:.3f} (1.0表示最优)")

        # 搜索效率评级
        efficiency_score = 0
        if search_time < 50:
            efficiency_score += 25
        elif search_time < 100:
            efficiency_score += 15
        elif search_time < 200:
            efficiency_score += 10

        if nodes_expanded < 1000:
            efficiency_score += 25
        elif nodes_expanded < 5000:
            efficiency_score += 15
        elif nodes_expanded < 10000:
            efficiency_score += 10

        if path_length > 0 and straight_dist > 0:
            if detour_ratio < 1.2:
                efficiency_score += 25
            elif detour_ratio < 1.5:
                efficiency_score += 15
            elif detour_ratio < 2.0:
                efficiency_score += 10

        if result.get('success', False):
            efficiency_score += 25

        print(f"\n搜索效率评分: {efficiency_score}/100")
        if efficiency_score >= 80:
            grade = "优秀 ⭐⭐⭐"
        elif efficiency_score >= 60:
            grade = "良好 ⭐⭐"
        elif efficiency_score >= 40:
            grade = "中等 ⭐"
        else:
            grade = "待改进"
        print(f"评级: {grade}")

def main():
    parser = argparse.ArgumentParser(description='前端搜索轨迹可视化工具')
    parser.add_argument('json_file', help='前端搜索导出的JSON文件路径')
    parser.add_argument('--save', '-s', help='保存图表的路径')
    parser.add_argument('--no-plot', action='store_true', help='只显示统计信息，不绘制图表')
    parser.add_argument('--no-show', action='store_true', help='保存图像但不显示')

    args = parser.parse_args()

    if not os.path.exists(args.json_file):
        print(f"错误：文件 {args.json_file} 不存在")
        return

    try:
        visualizer = FrontendSearchVisualizer(args.json_file)

        # 显示统计信息
        visualizer.print_statistics()
        visualizer.analyze_performance()

        # 绘制轨迹图表
        if not args.no_plot:
            save_path = args.save
            if not save_path:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                save_path = f"frontend_search_analysis_{timestamp}.png"

            visualizer.plot_trajectory(save_path, args.no_show)

    except Exception as e:
        print(f"错误：{e}")

if __name__ == "__main__":
    main()