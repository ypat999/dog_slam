#!/usr/bin/env python3
"""
后端优化可视化工具

用途：分析后端优化算法的优化前后对比
输入：后端优化导出的JSON文件
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
        'optimization_title': 'Trajectory Optimization Comparison',
        'before_optimization': 'Before Optimization',
        'after_optimization': 'After Optimization',
        'curvature_title': 'Curvature Comparison',
        'x_label': 'X (meters)',
        'y_label': 'Y (meters)',
        'curvature_label': 'Curvature (1/m)',
        'distance_label': 'Cumulative Distance (meters)',
        'start_label': 'Start',
        'end_label': 'Goal',
        'no_data': 'Warning: No data found',
        'saved_to': 'Optimization plot saved to',
    }

class BackendOptimizationVisualizer:
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

    def plot_optimization_comparison(self, save_path=None, no_show=False):
        """绘制优化前后对比"""
        input_traj = self.data.get('input_trajectory', [])
        optimized_path = self.data.get('optimized_path', [])

        if not input_traj or not optimized_path:
            print(LABELS.get('no_data', 'Warning: Missing trajectory data'))
            return

        # 提取输入轨迹坐标
        input_x = [point['x'] for point in input_traj]
        input_y = [point['y'] for point in input_traj]
        input_theta = [point['theta'] for point in input_traj]

        # 提取优化后路径坐标
        opt_x = [point['x'] for point in optimized_path]
        opt_y = [point['y'] for point in optimized_path]
        opt_yaw = [point['yaw'] for point in optimized_path]

        # 创建子图
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 12))

        # 子图1: 轨迹对比
        ax1.plot(input_x, input_y, 'r--', linewidth=2, label=LABELS.get('before_optimization', 'Before Optimization'), alpha=0.8)
        ax1.plot(opt_x, opt_y, 'b-', linewidth=2, label=LABELS.get('after_optimization', 'After Optimization'), alpha=0.8)

        # 标记起点和终点
        ax1.plot(input_x[0], input_y[0], 'go', markersize=10, label=LABELS.get('start_label', 'Start'))
        ax1.plot(input_x[-1], input_y[-1], 'ro', markersize=10, label=LABELS.get('end_label', 'Goal'))

        # 添加方向箭头 (每10个点)
        step = max(1, len(opt_x) // 10)
        arrow_added = False
        for i in range(0, len(opt_x), step):
            if i < len(opt_yaw):
                dx = 0.5 * np.cos(opt_yaw[i])
                dy = 0.5 * np.sin(opt_yaw[i])
                if not arrow_added:
                    ax1.arrow(opt_x[i], opt_y[i], dx, dy, head_width=0.2,
                             head_length=0.3, fc='blue', ec='blue', alpha=0.6,
                             label='Optimized Trajectory Direction')
                    arrow_added = True
                else:
                    ax1.arrow(opt_x[i], opt_y[i], dx, dy, head_width=0.2,
                             head_length=0.3, fc='blue', ec='blue', alpha=0.6)

        ax1.set_xlabel(LABELS.get('x_label', 'X (meters)'))
        ax1.set_ylabel(LABELS.get('y_label', 'Y (meters)'))
        ax1.set_title(LABELS.get('optimization_title', 'Trajectory Optimization Comparison'))
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')

        # 子图2: 曲率对比
        def calculate_curvature(x, y):
            """计算曲率"""
            if len(x) < 3:
                return [0] * len(x)

            curvatures = []
            for i in range(len(x)):
                if i == 0 or i == len(x) - 1:
                    curvatures.append(0)
                else:
                    # 使用三点法计算曲率
                    x1, y1 = x[i-1], y[i-1]
                    x2, y2 = x[i], y[i]
                    x3, y3 = x[i+1], y[i+1]

                    # 计算曲率
                    dx1, dy1 = x2 - x1, y2 - y1
                    dx2, dy2 = x3 - x2, y3 - y2
                    ds1 = np.sqrt(dx1**2 + dy1**2)
                    ds2 = np.sqrt(dx2**2 + dy2**2)

                    if ds1 > 0 and ds2 > 0:
                        cross_product = dx1 * dy2 - dy1 * dx2
                        curvature = cross_product / (ds1 * ds2 * (ds1 + ds2))
                        curvatures.append(abs(curvature))
                    else:
                        curvatures.append(0)

            return curvatures

        input_curvature = calculate_curvature(input_x, input_y)
        opt_curvature = calculate_curvature(opt_x, opt_y)

        # 计算距离用于x轴
        def calculate_distances(x, y):
            distances = [0]
            for i in range(1, len(x)):
                dx = x[i] - x[i-1]
                dy = y[i] - y[i-1]
                distances.append(distances[-1] + np.sqrt(dx**2 + dy**2))
            return distances

        input_dist = calculate_distances(input_x, input_y)
        opt_dist = calculate_distances(opt_x, opt_y)

        ax2.plot(input_dist, input_curvature, 'r--', linewidth=2, label=LABELS.get('before_optimization', 'Before Optimization'))
        ax2.plot(opt_dist, opt_curvature, 'b-', linewidth=2, label=LABELS.get('after_optimization', 'After Optimization'))
        ax2.set_xlabel(LABELS.get('distance_label', 'Cumulative Distance (meters)'))
        ax2.set_ylabel(LABELS.get('curvature_label', 'Curvature (1/m)'))
        ax2.set_title(LABELS.get('curvature_title', 'Curvature Comparison'))
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        # 子图3: 角度变化对比
        ax3.plot(input_dist, np.degrees(input_theta), 'r--', linewidth=2, label=LABELS.get('before_optimization', 'Before Optimization'))
        ax3.plot(opt_dist, np.degrees(opt_yaw), 'b-', linewidth=2, label=LABELS.get('after_optimization', 'After Optimization'))
        ax3.set_xlabel(LABELS.get('distance_label', 'Cumulative Distance (meters)'))
        ax3.set_ylabel(LABELS.get('angle_label', 'Angle (degrees)'))
        ax3.set_title(LABELS.get('angle_label', 'Angle Comparison'))
        ax3.legend()
        ax3.grid(True, alpha=0.3)

        # 子图4: 轨迹段统计
        segments = self.data.get('trajectory_segments', [])
        if segments:
            seg_indices = [seg['segment_index'] for seg in segments]
            seg_durations = [seg['duration_sec'] for seg in segments]
            seg_lengths = [seg['length_m'] for seg in segments]
            seg_velocities = [seg['avg_velocity_ms'] for seg in segments]

            ax4_twin = ax4.twinx()

            bars1 = ax4.bar([i - 0.2 for i in seg_indices], seg_durations,
                           width=0.4, label=LABELS.get('time_label', 'Duration (s)'), alpha=0.7, color='orange')
            line1 = ax4_twin.plot(seg_indices, seg_velocities, 'ro-',
                                 label=LABELS.get('velocity_label', 'Average Velocity (m/s)'), linewidth=2)

            ax4.set_xlabel('Segment Index')
            ax4.set_ylabel(LABELS.get('time_label', 'Duration (seconds)'), color='orange')
            ax4_twin.set_ylabel(LABELS.get('velocity_label', 'Average Velocity (m/s)'), color='red')
            ax4.set_title('Trajectory Segment Statistics')

            # 合并图例
            lines1, labels1 = ax4.get_legend_handles_labels()
            lines2, labels2 = ax4_twin.get_legend_handles_labels()
            ax4.legend(lines1 + lines2, labels1 + labels2, loc='upper right')

            ax4.grid(True, alpha=0.3)

        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"{LABELS.get('saved_to', 'Optimization plot saved to')}: {save_path}")

        if not no_show:
            plt.show()

    def print_statistics(self):
        """打印优化统计信息"""
        print(f"\n{'='*60}")
        print(f"后端优化分析报告")
        print(f"{'='*60}")

        # 基本信息
        print(f"会话ID: {self.data.get('session_id', 'N/A')}")
        print(f"时间戳: {self.data.get('timestamp', 'N/A')}")

        # 优化结果
        result = self.data.get('optimization_result', {})
        print(f"\n优化结果:")
        print(f"  成功: {'✅' if result.get('success') else '❌'}")
        print(f"  优化时间: {result.get('optimization_time_ms', 0):.2f} ms")
        print(f"  迭代次数: {result.get('iterations', 0)}")
        print(f"  成本降低: {result.get('cost_reduction', 0):.4f}")

        # 轨迹统计
        print(f"\n轨迹统计:")
        print(f"  总持续时间: {result.get('total_duration_sec', 0):.3f} 秒")
        print(f"  总长度: {result.get('total_length_m', 0):.3f} 米")
        print(f"  平均速度: {result.get('average_velocity_ms', 0):.3f} m/s")
        print(f"  轨迹段数: {result.get('num_segments', 0)}")
        print(f"  优化后点数: {result.get('num_optimized_points', 0)}")

        # 输入vs优化后对比
        input_traj = self.data.get('input_trajectory', [])
        optimized_path = self.data.get('optimized_path', [])

        if input_traj and optimized_path:
            print(f"\n轨迹对比:")
            print(f"  输入轨迹点数: {len(input_traj)}")
            print(f"  优化后点数: {len(optimized_path)}")

            # 计算路径长度
            def calc_path_length(trajectory, x_key, y_key):
                if len(trajectory) < 2:
                    return 0
                length = 0
                for i in range(1, len(trajectory)):
                    dx = trajectory[i][x_key] - trajectory[i-1][x_key]
                    dy = trajectory[i][y_key] - trajectory[i-1][y_key]
                    length += np.sqrt(dx**2 + dy**2)
                return length

            input_length = calc_path_length(input_traj, 'x', 'y')
            opt_length = calc_path_length(optimized_path, 'x', 'y')

            print(f"  输入路径长度: {input_length:.3f} 米")
            print(f"  优化后路径长度: {opt_length:.3f} 米")

            if input_length > 0:
                length_change = (opt_length - input_length) / input_length * 100
                print(f"  路径长度变化: {length_change:+.2f}%")

        # 轨迹段详情
        segments = self.data.get('trajectory_segments', [])
        if segments:
            print(f"\n轨迹段详情:")
            for i, seg in enumerate(segments[:5]):  # 显示前5段
                print(f"  段 {seg['segment_index']+1}: "
                      f"时间 {seg['duration_sec']:.2f}s, "
                      f"长度 {seg['length_m']:.2f}m, "
                      f"速度 {seg['avg_velocity_ms']:.2f}m/s")

            if len(segments) > 5:
                print(f"  ... 还有 {len(segments)-5} 个段")

    def analyze_optimization_quality(self):
        """优化质量分析"""
        print(f"\n{'='*60}")
        print(f"优化质量分析")
        print(f"{'='*60}")

        result = self.data.get('optimization_result', {})

        # 时间效率
        opt_time = result.get('optimization_time_ms', 0)
        iterations = result.get('iterations', 1)
        num_segments = result.get('num_segments', 1)

        if iterations > 0:
            time_per_iteration = opt_time / iterations
            print(f"每迭代优化时间: {time_per_iteration:.2f} ms/次")

        if num_segments > 0:
            time_per_segment = opt_time / num_segments
            print(f"每段优化时间: {time_per_segment:.2f} ms/段")

        # 收敛性分析
        if iterations > 0:
            convergence_rate = min(result.get('cost_reduction', 0) / iterations, 1.0)
            print(f"收敛速度: {convergence_rate:.6f} 成本降低/迭代")

        # 平滑度改进
        input_traj = self.data.get('input_trajectory', [])
        optimized_path = self.data.get('optimized_path', [])

        if input_traj and optimized_path and len(input_traj) > 2 and len(optimized_path) > 2:
            # 计算角度变化平滑度
            def calc_smoothness(trajectory, angle_key):
                if len(trajectory) < 3:
                    return 0

                angle_changes = []
                for i in range(1, len(trajectory) - 1):
                    prev_angle = trajectory[i-1][angle_key]
                    curr_angle = trajectory[i][angle_key]
                    next_angle = trajectory[i+1][angle_key]

                    # 计算角度变化
                    change1 = abs(curr_angle - prev_angle)
                    change2 = abs(next_angle - curr_angle)
                    angle_changes.append(change1 + change2)

                return np.mean(angle_changes) if angle_changes else 0

            input_smoothness = calc_smoothness(input_traj, 'theta')
            opt_smoothness = calc_smoothness(optimized_path, 'yaw')

            print(f"优化前平滑度: {input_smoothness:.4f}")
            print(f"优化后平滑度: {opt_smoothness:.4f}")

            if input_smoothness > 0:
                smoothness_improvement = (input_smoothness - opt_smoothness) / input_smoothness * 100
                print(f"平滑度改进: {smoothness_improvement:+.2f}%")

        # 质量评分
        quality_score = 0

        # 成功性 (25分)
        if result.get('success', False):
            quality_score += 25

        # 效率性 (25分)
        if opt_time < 50:
            quality_score += 25
        elif opt_time < 200:
            quality_score += 15
        elif opt_time < 500:
            quality_score += 10

        # 收敛性 (25分)
        if iterations < 50:
            quality_score += 25
        elif iterations < 100:
            quality_score += 15
        elif iterations < 200:
            quality_score += 10

        # 改进性 (25分)
        cost_reduction = result.get('cost_reduction', 0)
        if cost_reduction > 0.1:
            quality_score += 25
        elif cost_reduction > 0.05:
            quality_score += 15
        elif cost_reduction > 0.01:
            quality_score += 10

        print(f"\n优化质量评分: {quality_score}/100")
        if quality_score >= 80:
            grade = "优秀 ⭐⭐⭐"
        elif quality_score >= 60:
            grade = "良好 ⭐⭐"
        elif quality_score >= 40:
            grade = "中等 ⭐"
        else:
            grade = "待改进"
        print(f"评级: {grade}")

    def export_comparison_data(self, output_file):
        """导出对比数据为CSV格式"""
        input_traj = self.data.get('input_trajectory', [])
        optimized_path = self.data.get('optimized_path', [])

        try:
            with open(output_file, 'w') as f:
                f.write("type,index,x,y,angle,timestamp\n")

                # 写入输入轨迹
                for i, point in enumerate(input_traj):
                    f.write(f"input,{i},{point['x']},{point['y']},{point['theta']},0\n")

                # 写入优化后轨迹
                for i, point in enumerate(optimized_path):
                    timestamp = point.get('timestamp_sec', 0) + point.get('timestamp_nanosec', 0) * 1e-9
                    f.write(f"optimized,{i},{point['x']},{point['y']},{point['yaw']},{timestamp}\n")

            print(f"对比数据已导出至: {output_file}")
        except Exception as e:
            print(f"导出失败: {e}")

def main():
    parser = argparse.ArgumentParser(description='后端优化可视化工具')
    parser.add_argument('json_file', help='后端优化导出的JSON文件路径')
    parser.add_argument('--save', '-s', help='保存图表的路径')
    parser.add_argument('--export-csv', help='导出对比数据为CSV文件')
    parser.add_argument('--no-plot', action='store_true', help='只显示统计信息，不绘制图表')
    parser.add_argument('--no-show', action='store_true', help='保存图像但不显示')

    args = parser.parse_args()

    if not os.path.exists(args.json_file):
        print(f"错误：文件 {args.json_file} 不存在")
        return

    try:
        visualizer = BackendOptimizationVisualizer(args.json_file)

        # 显示统计信息
        visualizer.print_statistics()
        visualizer.analyze_optimization_quality()

        # 绘制对比图表
        if not args.no_plot:
            save_path = args.save
            if not save_path:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                save_path = f"backend_optimization_analysis_{timestamp}.png"

            visualizer.plot_optimization_comparison(save_path, args.no_show)

        # 导出CSV数据
        if args.export_csv:
            visualizer.export_comparison_data(args.export_csv)

    except Exception as e:
        print(f"错误：{e}")

if __name__ == "__main__":
    main()