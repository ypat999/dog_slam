#!/usr/bin/env python3
"""
批量分析工具

用途：批量分析指定目录下的所有Navigo导出JSON文件
输出：综合分析报告和统计汇总
"""

import json
import os
import argparse
from datetime import datetime
import glob
import subprocess
import sys

class BatchAnalyzer:
    def __init__(self, data_dir):
        """初始化批量分析器"""
        self.data_dir = data_dir
        self.results = {
            'frontend': [],
            'corridor': [],
            'backend': []
        }

    def find_json_files(self):
        """查找所有JSON文件"""
        json_files = glob.glob(os.path.join(self.data_dir, "*.json"))

        frontend_files = [f for f in json_files if 'frontend' in os.path.basename(f)]
        corridor_files = [f for f in json_files if 'corridor' in os.path.basename(f)]
        backend_files = [f for f in json_files if 'backend' in os.path.basename(f)]

        print(f"发现文件：")
        print(f"  前端搜索: {len(frontend_files)} 个")
        print(f"  走廊生成: {len(corridor_files)} 个")
        print(f"  后端优化: {len(backend_files)} 个")

        return {
            'frontend': frontend_files,
            'corridor': corridor_files,
            'backend': backend_files
        }

    def analyze_single_file(self, json_file, module_type):
        """分析单个JSON文件"""
        try:
            with open(json_file, 'r', encoding='utf-8') as f:
                data = json.load(f)

            result = {
                'file_path': json_file,
                'session_id': data.get('session_id', 'unknown'),
                'timestamp': data.get('timestamp', 'unknown'),
                'success': False,
                'score': 0
            }

            if module_type == 'frontend':
                search_result = data.get('search_result', {})
                result['success'] = search_result.get('success', False)
                result['search_time_ms'] = search_result.get('search_time_ms', 0)
                result['nodes_expanded'] = search_result.get('nodes_expanded', 0)
                result['path_length_m'] = search_result.get('path_length_m', 0)
                result['straight_line_distance_m'] = search_result.get('straight_line_distance_m', 0)

                # 计算前端评分
                score = 0
                if result['success']:
                    score += 25
                if result['search_time_ms'] < 50:
                    score += 25
                elif result['search_time_ms'] < 100:
                    score += 15
                elif result['search_time_ms'] < 200:
                    score += 10

                if result['nodes_expanded'] < 1000:
                    score += 25
                elif result['nodes_expanded'] < 5000:
                    score += 15
                elif result['nodes_expanded'] < 10000:
                    score += 10

                if result['path_length_m'] > 0 and result['straight_line_distance_m'] > 0:
                    detour_ratio = result['path_length_m'] / result['straight_line_distance_m']
                    if detour_ratio < 1.2:
                        score += 25
                    elif detour_ratio < 1.5:
                        score += 15
                    elif detour_ratio < 2.0:
                        score += 10

                result['score'] = score

            elif module_type == 'corridor':
                gen_result = data.get('generation_result', {})
                result['success'] = gen_result.get('success', False)
                result['processing_time_ms'] = gen_result.get('processing_time_ms', 0)
                result['total_segments'] = gen_result.get('total_segments', 0)
                result['total_vertices'] = gen_result.get('total_vertices', 0)

                # 计算走廊评分
                score = 0
                if result['success']:
                    score += 40
                if result['processing_time_ms'] < 100:
                    score += 30
                elif result['processing_time_ms'] < 500:
                    score += 20
                elif result['processing_time_ms'] < 1000:
                    score += 10

                # 简化一致性评分
                if result['total_segments'] > 0:
                    score += 30

                result['score'] = score

            elif module_type == 'backend':
                opt_result = data.get('optimization_result', {})
                result['success'] = opt_result.get('success', False)
                result['optimization_time_ms'] = opt_result.get('optimization_time_ms', 0)
                result['iterations'] = opt_result.get('iterations', 0)
                result['cost_reduction'] = opt_result.get('cost_reduction', 0)
                result['num_optimized_points'] = opt_result.get('num_optimized_points', 0)

                # 计算后端评分
                score = 0
                if result['success']:
                    score += 25
                if result['optimization_time_ms'] < 50:
                    score += 25
                elif result['optimization_time_ms'] < 200:
                    score += 15
                elif result['optimization_time_ms'] < 500:
                    score += 10

                if result['iterations'] < 50:
                    score += 25
                elif result['iterations'] < 100:
                    score += 15
                elif result['iterations'] < 200:
                    score += 10

                if result['cost_reduction'] > 0.1:
                    score += 25
                elif result['cost_reduction'] > 0.05:
                    score += 15
                elif result['cost_reduction'] > 0.01:
                    score += 10

                result['score'] = score

            return result

        except Exception as e:
            print(f"分析文件 {json_file} 时出错: {e}")
            return None

    def run_analysis(self):
        """运行批量分析"""
        files_dict = self.find_json_files()

        print(f"\n{'='*60}")
        print(f"开始批量分析...")
        print(f"{'='*60}")

        for module_type, files in files_dict.items():
            print(f"\n分析 {module_type} 模块...")
            for json_file in files:
                result = self.analyze_single_file(json_file, module_type)
                if result:
                    self.results[module_type].append(result)
                    status = "✅" if result['success'] else "❌"
                    print(f"  {os.path.basename(json_file)}: {status} (评分: {result['score']}/100)")

    def print_summary_report(self):
        """打印汇总报告"""
        print(f"\n{'='*60}")
        print(f"批量分析汇总报告")
        print(f"{'='*60}")

        for module_type, results in self.results.items():
            if not results:
                continue

            print(f"\n📊 {module_type.upper()} 模块统计:")
            print(f"  总文件数: {len(results)}")

            success_count = sum(1 for r in results if r['success'])
            print(f"  成功率: {success_count}/{len(results)} ({success_count/len(results)*100:.1f}%)")

            scores = [r['score'] for r in results]
            avg_score = sum(scores) / len(scores) if scores else 0
            print(f"  平均评分: {avg_score:.1f}/100")

            if module_type == 'frontend':
                search_times = [r['search_time_ms'] for r in results if 'search_time_ms' in r]
                if search_times:
                    print(f"  平均搜索时间: {sum(search_times)/len(search_times):.2f} ms")

                path_lengths = [r['path_length_m'] for r in results if 'path_length_m' in r and r['path_length_m'] > 0]
                if path_lengths:
                    print(f"  平均路径长度: {sum(path_lengths)/len(path_lengths):.2f} m")

            elif module_type == 'corridor':
                proc_times = [r['processing_time_ms'] for r in results if 'processing_time_ms' in r]
                if proc_times:
                    print(f"  平均处理时间: {sum(proc_times)/len(proc_times):.2f} ms")

                segments = [r['total_segments'] for r in results if 'total_segments' in r]
                if segments:
                    print(f"  平均走廊段数: {sum(segments)/len(segments):.1f}")

            elif module_type == 'backend':
                opt_times = [r['optimization_time_ms'] for r in results if 'optimization_time_ms' in r]
                if opt_times:
                    print(f"  平均优化时间: {sum(opt_times)/len(opt_times):.2f} ms")

                iterations = [r['iterations'] for r in results if 'iterations' in r]
                if iterations:
                    print(f"  平均迭代次数: {sum(iterations)/len(iterations):.1f}")

    def save_detailed_report(self, output_file):
        """保存详细报告到文件"""
        try:
            with open(output_file, 'w', encoding='utf-8') as f:
                f.write("Navigo 批量分析详细报告\n")
                f.write("="*50 + "\n")
                f.write(f"分析时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"数据目录: {self.data_dir}\n\n")

                for module_type, results in self.results.items():
                    if not results:
                        continue

                    f.write(f"{module_type.upper()} 模块详情:\n")
                    f.write("-" * 30 + "\n")

                    for result in results:
                        f.write(f"文件: {os.path.basename(result['file_path'])}\n")
                        f.write(f"  会话ID: {result['session_id']}\n")
                        f.write(f"  时间戳: {result['timestamp']}\n")
                        f.write(f"  成功: {'是' if result['success'] else '否'}\n")
                        f.write(f"  评分: {result['score']}/100\n")

                        if module_type == 'frontend':
                            f.write(f"  搜索时间: {result.get('search_time_ms', 0):.2f} ms\n")
                            f.write(f"  扩展节点: {result.get('nodes_expanded', 0)}\n")
                            f.write(f"  路径长度: {result.get('path_length_m', 0):.3f} m\n")

                        elif module_type == 'corridor':
                            f.write(f"  处理时间: {result.get('processing_time_ms', 0):.2f} ms\n")
                            f.write(f"  走廊段数: {result.get('total_segments', 0)}\n")

                        elif module_type == 'backend':
                            f.write(f"  优化时间: {result.get('optimization_time_ms', 0):.2f} ms\n")
                            f.write(f"  迭代次数: {result.get('iterations', 0)}\n")
                            f.write(f"  成本降低: {result.get('cost_reduction', 0):.4f}\n")

                        f.write("\n")

            print(f"\n详细报告已保存至: {output_file}")

        except Exception as e:
            print(f"保存报告失败: {e}")

    def generate_individual_plots(self, output_dir):
        """生成各个文件的可视化图表"""
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

        script_dir = os.path.dirname(os.path.abspath(__file__))

        print(f"\n生成可视化图表...")

        for module_type, results in self.results.items():
            if not results:
                continue

            script_name = f"visualize_{module_type.replace('frontend', 'frontend_search')}.py"
            if module_type == 'frontend':
                script_name = "visualize_frontend_search.py"
            elif module_type == 'corridor':
                script_name = "visualize_corridor.py"
            elif module_type == 'backend':
                script_name = "visualize_backend_optimization.py"

            script_path = os.path.join(script_dir, script_name)

            for result in results[:5]:  # 只处理前5个文件避免过多输出
                json_file = result['file_path']
                session_id = result['session_id']
                output_file = os.path.join(output_dir, f"{session_id}_{module_type}.png")

                try:
                    cmd = [sys.executable, script_path, json_file, "--save", output_file, "--no-plot"]
                    subprocess.run(cmd, check=True, capture_output=True)
                    print(f"  生成图表: {os.path.basename(output_file)}")
                except subprocess.CalledProcessError:
                    print(f"  跳过图表生成: {os.path.basename(json_file)} (脚本执行失败)")
                except FileNotFoundError:
                    print(f"  跳过图表生成: {script_name} 不存在")

def main():
    parser = argparse.ArgumentParser(description='Navigo批量分析工具')
    parser.add_argument('data_dir', help='包含JSON文件的数据目录', default='/tmp/navigo_debug_data', nargs='?')
    parser.add_argument('--output-dir', '-o', help='输出目录', default='./batch_analysis_results')
    parser.add_argument('--no-plots', action='store_true', help='不生成可视化图表')
    parser.add_argument('--report-file', help='详细报告输出文件', default='navigo_batch_report.txt')

    args = parser.parse_args()

    if not os.path.exists(args.data_dir):
        print(f"错误：数据目录 {args.data_dir} 不存在")
        return

    # 创建输出目录
    if not os.path.exists(args.output_dir):
        os.makedirs(args.output_dir)

    try:
        analyzer = BatchAnalyzer(args.data_dir)

        # 运行分析
        analyzer.run_analysis()

        # 显示汇总报告
        analyzer.print_summary_report()

        # 保存详细报告
        report_path = os.path.join(args.output_dir, args.report_file)
        analyzer.save_detailed_report(report_path)

        # 生成可视化图表
        if not args.no_plots:
            analyzer.generate_individual_plots(args.output_dir)

        print(f"\n✅ 批量分析完成！")
        print(f"输出目录: {args.output_dir}")

    except Exception as e:
        print(f"批量分析失败: {e}")

if __name__ == "__main__":
    main()