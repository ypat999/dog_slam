#!/usr/bin/env python3
"""
字体测试脚本

用于测试matplotlib中文字体配置是否正常工作
"""

import matplotlib.pyplot as plt
import numpy as np
import warnings

def test_font_display():
    """测试字体显示"""
    print("🔧 测试matplotlib字体配置...")

    # 禁用字体警告
    warnings.filterwarnings('ignore', category=UserWarning, module='matplotlib')

    try:
        # 尝试导入字体配置
        from font_config import CHINESE_FONT_AVAILABLE, LABELS, print_font_info
        print_font_info()
        use_chinese = CHINESE_FONT_AVAILABLE
    except ImportError:
        print("⚠️  字体配置模块不可用，使用英文标签")
        use_chinese = False

    # 创建测试图表
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    # 生成测试数据
    x = np.linspace(0, 10, 100)
    y1 = np.sin(x)
    y2 = np.cos(x)

    # 子图1
    if use_chinese:
        ax1.plot(x, y1, label='搜索轨迹', color='blue')
        ax1.plot(x, y2, label='优化轨迹', color='red')
        ax1.set_xlabel('距离 (米)')
        ax1.set_ylabel('高度 (米)')
        ax1.set_title('轨迹对比')
    else:
        ax1.plot(x, y1, label='Search Trajectory', color='blue')
        ax1.plot(x, y2, label='Optimized Trajectory', color='red')
        ax1.set_xlabel('Distance (meters)')
        ax1.set_ylabel('Height (meters)')
        ax1.set_title('Trajectory Comparison')

    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # 子图2 - 角度测试
    angles = np.degrees(np.arctan2(np.diff(y1), np.diff(x)))
    if use_chinese:
        ax2.plot(x[1:], angles, label='角度变化', color='green')
        ax2.set_xlabel('距离 (米)')
        ax2.set_ylabel('角度 (度)')
        ax2.set_title('角度变化分析')
    else:
        ax2.plot(x[1:], angles, label='Angle Changes', color='green')
        ax2.set_xlabel('Distance (meters)')
        ax2.set_ylabel('Angle (degrees)')
        ax2.set_title('Angle Analysis')

    ax2.legend()
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()

    # 保存测试图片
    output_file = '/tmp/navigo_debug_data/font_display_test.png'
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"✅ 测试图片已保存: {output_file}")

    plt.close()

    # 打印测试结果
    print(f"\n📊 字体测试结果:")
    print(f"  中文支持: {'是' if use_chinese else '否'}")
    print(f"  图表生成: 成功")
    print(f"  文件保存: {output_file}")

    return use_chinese

def test_simple_plot():
    """简单绘图测试（纯英文，避免字体问题）"""
    print("\n🎯 执行简单绘图测试...")

    # 禁用所有matplotlib警告
    warnings.filterwarnings('ignore')

    fig, ax = plt.subplots(figsize=(8, 5))

    x = np.linspace(0, 5, 50)
    y = np.sin(2 * np.pi * x)

    ax.plot(x, y, 'b-', linewidth=2, label='Sine Wave')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Amplitude')
    ax.set_title('Simple Plot Test')
    ax.legend()
    ax.grid(True, alpha=0.3)

    output_file = '/tmp/navigo_debug_data/simple_plot_test.png'
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    plt.close()

    print(f"✅ 简单测试成功: {output_file}")

if __name__ == "__main__":
    print("🚀 开始字体配置测试\n")

    try:
        # 测试1: 字体显示
        font_ok = test_font_display()

        # 测试2: 简单绘图
        test_simple_plot()

        print(f"\n🎉 测试完成!")
        print(f"字体状态: {'支持中文' if font_ok else '仅英文'}")
        print(f"建议: {'可以正常使用所有功能' if font_ok else '使用--no-plot参数避免显示问题'}")

    except Exception as e:
        print(f"❌ 测试失败: {e}")
        print("建议: 检查matplotlib安装或使用--no-plot参数")