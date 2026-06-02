#!/usr/bin/env python3
"""
字体配置模块

用于解决matplotlib中文字体显示问题
"""

import matplotlib
import matplotlib.pyplot as plt
import warnings
import os

def setup_matplotlib_font():
    """
    设置matplotlib字体配置，支持中文显示

    Returns:
        bool: 是否成功设置中文字体
    """
    try:
        # 常见中文字体列表（按优先级排序）
        chinese_fonts = [
            # Windows系统字体
            'Microsoft YaHei',   # 微软雅黑
            'SimHei',            # 黑体
            'SimSun',            # 宋体

            # Linux系统字体
            'DejaVu Sans',       # 通用字体（基础支持）
            'WenQuanYi Micro Hei', # 文泉驿微米黑
            'WenQuanYi Zen Hei', # 文泉驿正黑
            'Droid Sans Fallback', # Android字体
            'Noto Sans CJK SC',  # Google Noto字体
            'Source Han Sans CN', # 思源黑体

            # macOS系统字体
            'PingFang SC',       # 苹果苹方
            'Hiragino Sans GB',  # 冬青黑体
            'STHeiti',           # 华文黑体
        ]

        # 获取系统可用字体
        available_fonts = set()
        try:
            font_list = matplotlib.font_manager.fontManager.ttflist
            available_fonts = {f.name for f in font_list}
        except Exception:
            # 如果获取字体列表失败，尝试基础配置
            pass

        # 查找第一个可用的中文字体
        selected_font = None
        for font in chinese_fonts:
            if font in available_fonts:
                selected_font = font
                break

        if selected_font and selected_font != 'DejaVu Sans':
            # 设置字体 (但排除DejaVu Sans，因为它不支持中文)
            plt.rcParams['font.sans-serif'] = [selected_font] + plt.rcParams['font.sans-serif']
            plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题

            print(f"✅ 中文字体设置成功: {selected_font}")
            return True
        else:
            # 如果没有找到合适的中文字体，使用英文标签和禁用警告
            print("⚠️  未找到合适中文字体，使用英文标签")

            # 禁用所有matplotlib字体相关警告
            warnings.filterwarnings('ignore', category=UserWarning, module='matplotlib')
            warnings.filterwarnings('ignore', message='.*Glyph.*missing.*')
            warnings.filterwarnings('ignore', message='.*font.*')

            return False

    except Exception as e:
        print(f"⚠️  字体配置失败: {e}")
        # 禁用警告
        warnings.filterwarnings('ignore', category=UserWarning, module='matplotlib')
        return False

def get_labels(chinese_available=True):
    """
    获取双语标签

    Args:
        chinese_available (bool): 是否支持中文显示

    Returns:
        dict: 标签字典
    """
    if chinese_available:
        return {
            # 通用标签
            'x_label': 'X (米)',
            'y_label': 'Y (米)',
            'time_label': '时间 (秒)',
            'distance_label': '距离 (米)',
            'angle_label': '角度 (度)',
            'velocity_label': '速度 (m/s)',
            'start_label': '起点',
            'end_label': '终点',

            # 前端搜索
            'frontend_title': '前端搜索轨迹',
            'trajectory_label': '搜索轨迹',
            'angle_change_title': '角度变化',

            # 走廊生成
            'corridor_title': '安全驾驶走廊',
            'corridor_width_title': '走廊宽度变化',
            'input_trajectory_label': '输入轨迹',

            # 后端优化
            'optimization_title': '轨迹优化对比',
            'before_optimization': '优化前',
            'after_optimization': '优化后',
            'curvature_title': '曲率对比',
            'curvature_label': '曲率 (1/m)',

            # 状态信息
            'success': '成功',
            'failed': '失败',
            'processing': '处理中...',
            'saved_to': '已保存至',
            'warning_no_data': '警告：未找到数据',
        }
    else:
        return {
            # 通用标签
            'x_label': 'X (meters)',
            'y_label': 'Y (meters)',
            'time_label': 'Time (seconds)',
            'distance_label': 'Distance (meters)',
            'angle_label': 'Angle (degrees)',
            'velocity_label': 'Velocity (m/s)',
            'start_label': 'Start',
            'end_label': 'Goal',

            # 前端搜索
            'frontend_title': 'Frontend Search Trajectory',
            'trajectory_label': 'Search Trajectory',
            'angle_change_title': 'Angle Changes',

            # 走廊生成
            'corridor_title': 'Safe Driving Corridor',
            'corridor_width_title': 'Corridor Width Changes',
            'input_trajectory_label': 'Input Trajectory',

            # 后端优化
            'optimization_title': 'Trajectory Optimization Comparison',
            'before_optimization': 'Before Optimization',
            'after_optimization': 'After Optimization',
            'curvature_title': 'Curvature Comparison',
            'curvature_label': 'Curvature (1/m)',

            # 状态信息
            'success': 'Success',
            'failed': 'Failed',
            'processing': 'Processing...',
            'saved_to': 'Saved to',
            'warning_no_data': 'Warning: No data found',
        }

# 全局配置
CHINESE_FONT_AVAILABLE = setup_matplotlib_font()
LABELS = get_labels(CHINESE_FONT_AVAILABLE)

def print_font_info():
    """打印字体配置信息"""
    print("🔧 Matplotlib字体配置信息:")
    print(f"  中文支持: {'是' if CHINESE_FONT_AVAILABLE else '否'}")
    print(f"  当前字体: {plt.rcParams['font.sans-serif']}")
    print(f"  负号处理: {not plt.rcParams['axes.unicode_minus']}")

if __name__ == "__main__":
    print_font_info()

    # 测试字体显示
    import matplotlib.pyplot as plt
    import numpy as np

    fig, ax = plt.subplots(figsize=(8, 6))
    x = np.linspace(0, 10, 100)
    y = np.sin(x)

    ax.plot(x, y, label=LABELS['trajectory_label'])
    ax.set_xlabel(LABELS['x_label'])
    ax.set_ylabel(LABELS['y_label'])
    ax.set_title(LABELS['frontend_title'])
    ax.legend()
    ax.grid(True)

    plt.tight_layout()
    plt.savefig('/tmp/navigo_debug_data/font_test.png', dpi=150, bbox_inches='tight')
    print(f"测试图片已保存: /tmp/navigo_debug_data/font_test.png")
    plt.close()