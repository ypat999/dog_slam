#!/usr/bin/env python3
"""
走廊可视化工具

用途：分析安全驾驶走廊生成结果
输入：走廊生成导出的JSON文件
输出：可视化图表和分析报告
"""

import json
import matplotlib.pyplot as plt
import numpy as np
import argparse
import os
from datetime import datetime
from matplotlib.patches import Polygon
import matplotlib.patches as mpatches
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
        'corridor_title': 'Safe Driving Corridor',
        'corridor_width_title': 'Corridor Width Changes',
        'x_label': 'X (meters)',
        'y_label': 'Y (meters)',
        'width_label': 'Width (meters)',
        'distance_label': 'Cumulative Distance (meters)',
        'input_trajectory_label': 'Input Trajectory',
        'corridor_label': 'Corridor',
        'start_label': 'Start',
        'end_label': 'Goal',
        'no_data': 'Warning: No data found',
        'saved_to': 'Corridor plot saved to',
    }

class CorridorVisualizer:
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

    def plot_corridor(self, save_path=None, no_show=False):
        """绘制走廊和轨迹"""
        # 支持新旧格式的数据
        # 新格式: trajectory_segment
        # 旧格式: input_trajectory
        trajectory_segment = self.data.get('trajectory_segment', [])
        input_trajectory = self.data.get('input_trajectory', [])
        trajectory = trajectory_segment if trajectory_segment else input_trajectory
        
        corridor_per_point = self.data.get('corridor_per_point', [])
        corridor_segments = self.data.get('corridor_segments', [])  # 旧格式兼容
        robot_dims = self.data.get('robot_dimensions', {})

        if not trajectory:
            print(LABELS.get('no_data', 'Warning: No trajectory data found'))
            return

        # 提取轨迹坐标和角度
        traj_x = [point['x'] for point in trajectory]
        traj_y = [point['y'] for point in trajectory]
        traj_theta = [point['theta'] for point in trajectory]

        # 创建图形
        fig, ax1 = plt.subplots(1, 1, figsize=(12, 8))

        # 子图1: 走廊和轨迹
        # 优先使用新格式的corridor_per_point数据
        corridor_polygon_added = False
        if corridor_per_point:
            # 绘制每个轨迹点的走廊
            colors = plt.cm.Set3(np.linspace(0, 1, max(len(corridor_per_point), 1)))

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
                    polygon = Polygon(poly_vertices, alpha=0.2,
                                    facecolor=colors[i % len(colors)],
                                    edgecolor='darkblue', linewidth=0.3,
                                    label=polygon_label)
                    ax1.add_patch(polygon)
                    corridor_polygon_added = True
        else:
            # 使用旧格式的corridor_segments数据
            colors = plt.cm.Set3(np.linspace(0, 1, max(len(corridor_segments), 1)))
            corridor_polygon_added = False

            for i, segment in enumerate(corridor_segments):
                vertices = segment.get('polygon_vertices', [])
                if len(vertices) >= 3:
                    poly_vertices = np.array(vertices)
                    # 闭合多边形
                    if not np.allclose(poly_vertices[0], poly_vertices[-1]):
                        poly_vertices = np.vstack([poly_vertices, poly_vertices[0]])

                    # 只为第一个走廊段添加图例标签
                    polygon_label = LABELS.get('corridor_label', 'Corridor') if not corridor_polygon_added else ""
                    polygon = Polygon(poly_vertices, alpha=0.3,
                                    facecolor=colors[i % len(colors)],
                                    edgecolor='black', linewidth=0.5,
                                    label=polygon_label)
                    ax1.add_patch(polygon)
                    corridor_polygon_added = True

        # 绘制输入轨迹
        ax1.plot(traj_x, traj_y, 'red', linewidth=2, label=LABELS.get('input_trajectory_label', 'Input Trajectory'), alpha=0.9)
        ax1.scatter(traj_x, traj_y, c='blue', s=10, alpha=0.6, zorder=5)
        
        # 添加方向箭头（每隔5个点显示一个）
        arrow_step = 1
        arrow_added = False
        for i in range(0, len(traj_x), arrow_step):
            if i < len(traj_x):
                if not arrow_added:
                    ax1.arrow(traj_x[i], traj_y[i],
                             0.3 * np.cos(traj_theta[i]), 0.3 * np.sin(traj_theta[i]),
                             head_width=0.1, head_length=0.15,
                             fc='red', ec='darkred', alpha=0.8, zorder=4,
                             label='Trajectory Direction')
                    arrow_added = True
                else:
                    ax1.arrow(traj_x[i], traj_y[i],
                             0.3 * np.cos(traj_theta[i]), 0.3 * np.sin(traj_theta[i]),
                             head_width=0.1, head_length=0.15,
                             fc='red', ec='darkred', alpha=0.8, zorder=4)

        # 标记起点和终点
        if traj_x and traj_y:
            ax1.plot(traj_x[0], traj_y[0], 'go', markersize=12, label=LABELS.get('start_label', 'Start'), zorder=6)
            ax1.plot(traj_x[-1], traj_y[-1], 'ro', markersize=12, label=LABELS.get('end_label', 'Goal'), zorder=6)

        ax1.set_xlabel(LABELS.get('x_label', 'X (meters)'))
        ax1.set_ylabel(LABELS.get('y_label', 'Y (meters)'))
        ax1.set_title(LABELS.get('corridor_title', 'Safe Driving Corridor'))
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')

        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"{LABELS.get('saved_to', 'Corridor plot saved to')}: {save_path}")

        if not no_show:
            plt.show()

    def print_statistics(self):
        """打印走廊统计信息"""
        print(f"\n{'='*60}")
        print(f"走廊生成分析报告")
        print(f"{'='*60}")

        # 基本信息
        print(f"会话ID: {self.data.get('session_id', 'N/A')}")
        print(f"时间戳: {self.data.get('timestamp', 'N/A')}")
        print(f"策略名称: {self.data.get('strategy_name', 'N/A')}")

        # 输入轨迹信息 - 支持新旧格式
        # 新格式: trajectory_segment
        # 旧格式: input_trajectory
        trajectory_segment = self.data.get('trajectory_segment', [])
        input_trajectory = self.data.get('input_trajectory', [])
        trajectory = trajectory_segment if trajectory_segment else input_trajectory
        
        if trajectory:
            print(f"\n输入轨迹:")
            traj_x = [p['x'] for p in trajectory]
            traj_y = [p['y'] for p in trajectory]
            print(f"  轨迹点数: {len(trajectory)}")
            print(f"  X范围: {min(traj_x):.3f} ~ {max(traj_x):.3f} 米")
            print(f"  Y范围: {min(traj_y):.3f} ~ {max(traj_y):.3f} 米")

            # 计算轨迹长度
            total_length = 0
            for i in range(1, len(trajectory)):
                dx = traj_x[i] - traj_x[i-1]
                dy = traj_y[i] - traj_y[i-1]
                total_length += np.sqrt(dx**2 + dy**2)
            print(f"  轨迹总长度: {total_length:.3f} 米")

        # 机器人尺寸
        robot_dims = self.data.get('robot_dimensions', {})
        print(f"\n机器人参数:")
        print(f"  宽度: {robot_dims.get('width', 0):.3f} 米")
        print(f"  长度: {robot_dims.get('length', 0):.3f} 米")
        print(f"  安全边距: {robot_dims.get('safety_margin', 0):.3f} 米")

        # 走廊生成结果
        result = self.data.get('generation_result', {})
        print(f"\n生成结果:")
        print(f"  成功: {'✅' if result.get('success') else '❌'}")
        print(f"  处理时间: {result.get('processing_time_ms', 0):.2f} ms")
        print(f"  走廊段数: {result.get('total_segments', 0)}")
        print(f"  总顶点数: {result.get('total_vertices', 0)}")

        # 走廊段统计
        corridor_segments = self.data.get('corridor_segments', [])
        if corridor_segments:
            print(f"\n走廊段详情:")
            total_area = 0
            min_area = float('inf')
            max_area = 0

            for i, segment in enumerate(corridor_segments):
                vertices = segment.get('polygon_vertices', [])
                vertices_count = len(vertices)

                # 计算面积 (简化计算)
                if vertices_count >= 3:
                    vertices_array = np.array(vertices)
                    area = 0.5 * abs(sum(vertices_array[i][0] * (vertices_array[(i+1) % vertices_count][1] - vertices_array[i-1][1])
                                       for i in range(vertices_count)))
                    total_area += area
                    min_area = min(min_area, area)
                    max_area = max(max_area, area)

                if i < 5:  # 只显示前5个段的详情
                    print(f"  段 {i+1}: {vertices_count} 个顶点, 面积 {area:.4f} m²")

            if corridor_segments:
                avg_area = total_area / len(corridor_segments)
                print(f"  平均面积: {avg_area:.4f} m²")
                print(f"  最小面积: {min_area:.4f} m²")
                print(f"  最大面积: {max_area:.4f} m²")
                print(f"  总面积: {total_area:.4f} m²")

    def analyze_corridor_quality(self):
        """走廊质量分析"""
        print(f"\n{'='*60}")
        print(f"走廊质量分析")
        print(f"{'='*60}")

        result = self.data.get('generation_result', {})
        corridor_segments = self.data.get('corridor_segments', [])
        robot_dims = self.data.get('robot_dimensions', {})

        # 时间效率
        processing_time = result.get('processing_time_ms', 0)
        num_segments = result.get('total_segments', 1)

        if num_segments > 0:
            time_per_segment = processing_time / num_segments
            print(f"每段处理时间: {time_per_segment:.2f} ms/段")

        # 走廊宽度一致性
        if corridor_segments:
            widths = []
            for segment in corridor_segments:
                vertices = segment.get('polygon_vertices', [])
                if len(vertices) >= 4:
                    vertices_array = np.array(vertices)
                    min_x, max_x = np.min(vertices_array[:, 0]), np.max(vertices_array[:, 0])
                    min_y, max_y = np.min(vertices_array[:, 1]), np.max(vertices_array[:, 1])
                    width = max(max_x - min_x, max_y - min_y)
                    widths.append(width)

            if widths:
                width_std = np.std(widths)
                width_mean = np.mean(widths)
                width_cv = width_std / width_mean if width_mean > 0 else 0
                print(f"宽度一致性 (变异系数): {width_cv:.4f}")

                # 与机器人尺寸的关系
                robot_total_width = robot_dims.get('width', 0) + 2 * robot_dims.get('safety_margin', 0)
                if robot_total_width > 0:
                    width_ratio = width_mean / robot_total_width
                    print(f"走廊/机器人宽度比: {width_ratio:.2f}")

        # 质量评分
        quality_score = 0

        # 成功性 (40分)
        if result.get('success', False):
            quality_score += 40

        # 效率性 (30分)
        if processing_time < 100:
            quality_score += 30
        elif processing_time < 500:
            quality_score += 20
        elif processing_time < 1000:
            quality_score += 10

        # 一致性 (30分)
        if corridor_segments and len(corridor_segments) > 1:
            if width_cv < 0.1:
                quality_score += 30
            elif width_cv < 0.2:
                quality_score += 20
            elif width_cv < 0.3:
                quality_score += 10

        print(f"\n走廊质量评分: {quality_score}/100")
        if quality_score >= 80:
            grade = "优秀 ⭐⭐⭐"
        elif quality_score >= 60:
            grade = "良好 ⭐⭐"
        elif quality_score >= 40:
            grade = "中等 ⭐"
        else:
            grade = "待改进"
        print(f"评级: {grade}")

    def export_corridor_data(self, output_file):
        """导出走廊数据为CSV格式"""
        corridor_segments = self.data.get('corridor_segments', [])

        if not corridor_segments:
            print("没有走廊数据可导出")
            return

        try:
            with open(output_file, 'w') as f:
                f.write("segment_id,vertex_id,x,y\n")
                for seg_idx, segment in enumerate(corridor_segments):
                    vertices = segment.get('polygon_vertices', [])
                    for vert_idx, vertex in enumerate(vertices):
                        f.write(f"{seg_idx},{vert_idx},{vertex[0]},{vertex[1]}\n")

            print(f"走廊数据已导出至: {output_file}")
        except Exception as e:
            print(f"导出失败: {e}")

def main():
    parser = argparse.ArgumentParser(description='走廊生成可视化工具')
    parser.add_argument('json_file', help='走廊生成导出的JSON文件路径')
    parser.add_argument('--save', '-s', help='保存图表的路径')
    parser.add_argument('--export-csv', help='导出走廊数据为CSV文件')
    parser.add_argument('--no-plot', action='store_true', help='只显示统计信息，不绘制图表')
    parser.add_argument('--no-show', action='store_true', help='保存图像但不显示')

    args = parser.parse_args()

    if not os.path.exists(args.json_file):
        print(f"错误：文件 {args.json_file} 不存在")
        return

    try:
        visualizer = CorridorVisualizer(args.json_file)

        # 显示统计信息
        visualizer.print_statistics()
        visualizer.analyze_corridor_quality()

        # 绘制走廊图表
        if not args.no_plot:
            save_path = args.save
            if not save_path:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                save_path = f"corridor_analysis_{timestamp}.png"

            visualizer.plot_corridor(save_path, args.no_show)

        # 导出CSV数据
        if args.export_csv:
            visualizer.export_corridor_data(args.export_csv)

    except Exception as e:
        print(f"错误：{e}")

if __name__ == "__main__":
    main()