#!/usr/bin/env python3

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import argparse
import os
import json
from datetime import datetime
from typing import Dict, List, Tuple

class ExperimentBAnalyzer:
    """
    实验B分析器 (精简版) - 复用A1结果作为R2V延迟
    """
    
    def __init__(self, r2v_timing_csv: str, v2r_csv: str, tracking_error_csv: str):
        # 明确数据源
        self.r2v_timing_csv = r2v_timing_csv      # 来自A1的R2V延迟数据
        self.v2r_csv = v2r_csv                    # V2R延迟数据
        self.tracking_error_csv = tracking_error_csv  # 跟踪误差数据
        
        self.analysis_timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.setup_directories()
        self.setup_plotting()
        
        # 加载和验证数据
        self.load_data()
        self.analysis_results = {}

    def load_data(self):
        """加载所有数据文件"""
        try:
            # 加载A1的R2V延迟数据
            self.r2v_df = pd.read_csv(self.r2v_timing_csv)
            print(f"加载R2V延迟数据 (来自A1): {len(self.r2v_df)} 条记录")
            
            # 加载V2R延迟数据
            self.v2r_df = pd.read_csv(self.v2r_csv)
            print(f"加载V2R延迟数据: {len(self.v2r_df)} 条记录")
            
            # 加载跟踪误差数据
            self.tracking_error_df = pd.read_csv(self.tracking_error_csv)
            print(f"加载跟踪误差数据: {len(self.tracking_error_df)} 条记录")
            
            # 清理数据
            self.clean_data()
            
        except Exception as e:
            print(f"数据加载错误: {e}")
            raise

    def clean_data(self):
        """清理和过滤数据"""
        # 过滤有效的R2V数据 (来自A1)
        initial_r2v = len(self.r2v_df)
        self.r2v_df = self.r2v_df[
            (self.r2v_df['sequence_valid'] == True) &
            (self.r2v_df['r2v_total_latency_ms'] >= 0) &
            (self.r2v_df['r2v_total_latency_ms'] <= 1000)
        ]
        print(f"R2V数据清理: {len(self.r2v_df)}/{initial_r2v} 条有效记录")
        
        # 过滤V2R数据
        initial_v2r = len(self.v2r_df)
        self.v2r_df = self.v2r_df[
            (self.v2r_df['v2r_latency_ms'] > 0) &
            (self.v2r_df['v2r_latency_ms'] < 250)
        ]
        print(f"V2R数据清理: {len(self.v2r_df)}/{initial_v2r} 条有效记录")
        
        # 过滤跟踪误差数据 - 只要测试激活期间的数据
        initial_error = len(self.tracking_error_df)
        if 'test_active' in self.tracking_error_df.columns:
            self.tracking_error_df = self.tracking_error_df[
                self.tracking_error_df['test_active'] == True
            ]
        print(f"跟踪误差数据清理: {len(self.tracking_error_df)}/{initial_error} 条有效记录")

    def analyze_latency_components(self):
        """分析延迟组件 - 基于A1复用的简化方法"""
        print("\n" + "="*80)
        print("实验B - 第一部分: 延迟特性分析 (基于A1复用)")
        print("="*80)
        
        # 分析R2V延迟 (来自A1实验)
        r2v_stats = self.calculate_stats(self.r2v_df['r2v_total_latency_ms'], 'R2V (来自A1)')
        
        # 分析V2R延迟
        v2r_stats = self.calculate_stats(self.v2r_df['v2r_latency_ms'], 'V2R')
        
        # 计算总延迟 (R2V + V2R, 暂不包含Planner)
        # 注意：这里我们简化了Planner延迟，或者可以设置为固定估算值
        planner_estimated_ms = 20.0  # 估算值，或者设为0
        total_mean_ms = r2v_stats['mean'] + v2r_stats['mean'] + planner_estimated_ms
        
        # 存储结果
        self.analysis_results['latency_statistics'] = {
            'r2v_delay_ms': r2v_stats,
            'v2r_delay_ms': v2r_stats,
            'planner_delay_ms': {
                'mean': planner_estimated_ms,
                'note': '估算值 - 实际值需要Autoware内部测量'
            },
            'total_delay_ms': {
                'mean': total_mean_ms,
                'components': {
                    'r2v_contribution_pct': (r2v_stats['mean'] / total_mean_ms) * 100,
                    'planner_contribution_pct': (planner_estimated_ms / total_mean_ms) * 100,
                    'v2r_contribution_pct': (v2r_stats['mean'] / total_mean_ms) * 100
                }
            }
        }
        
        # 打印结果
        self.print_latency_table()
        self.assess_latency_performance()
        
        return self.analysis_results['latency_statistics']

    def calculate_stats(self, data: pd.Series, component_name: str) -> Dict:
        """计算统计信息"""
        return {
            'count': len(data),
            'mean': data.mean(),
            'median': data.median(),
            'std': data.std(),
            'min': data.min(),
            'max': data.max(),
            'p95': data.quantile(0.95),
            'p99': data.quantile(0.99)
        }

    def print_latency_table(self):
        """打印延迟统计表"""
        stats = self.analysis_results['latency_statistics']
        
        print(f"\n延迟组件分析 (精简版 - 基于A1复用)")
        print("-" * 100)
        print(f"{'组件':<20} {'样本数':<8} {'均值':<8} {'中位数':<8} {'标准差':<8} {'P95':<8} {'P99':<8}")
        print("-" * 100)
        
        # R2V延迟 (来自A1)
        r2v = stats['r2v_delay_ms']
        print(f"{'R2V延迟 (A1复用)':<20} {r2v['count']:<8} {r2v['mean']:<8.2f} {r2v['median']:<8.2f} "
              f"{r2v['std']:<8.2f} {r2v['p95']:<8.2f} {r2v['p99']:<8.2f}")
        
        # V2R延迟
        v2r = stats['v2r_delay_ms']
        print(f"{'V2R延迟':<20} {v2r['count']:<8} {v2r['mean']:<8.2f} {v2r['median']:<8.2f} "
              f"{v2r['std']:<8.2f} {v2r['p95']:<8.2f} {v2r['p99']:<8.2f}")
        
        # Planner延迟 (估算)
        planner = stats['planner_delay_ms']
        print(f"{'Planner延迟 (估算)':<20} {'-':<8} {planner['mean']:<8.2f} {'-':<8} "
              f"{'-':<8} {'-':<8} {'-':<8}")
        
        print("-" * 100)
        print(f"{'总端到端延迟':<20} {'-':<8} {stats['total_delay_ms']['mean']:<8.2f}")
        print("-" * 100)

    def assess_latency_performance(self):
        """评估延迟性能"""
        stats = self.analysis_results['latency_statistics']
        total_latency = stats['total_delay_ms']['mean']
        
        print(f"\n延迟性能评估:")
        print("-" * 60)
        
        criteria = [
            ('总延迟 < 150ms', total_latency < 150, f"{total_latency:.1f}ms"),
            ('R2V延迟 < 100ms', stats['r2v_delay_ms']['mean'] < 100, 
             f"{stats['r2v_delay_ms']['mean']:.1f}ms"),
            ('V2R延迟 < 50ms', stats['v2r_delay_ms']['mean'] < 50, 
             f"{stats['v2r_delay_ms']['mean']:.1f}ms"),
        ]
        
        passed = 0
        for criterion, result, value in criteria:
            status = "通过" if result else "失败"
            print(f"  {criterion:<30}: {status:<4} ({value})")
            if result:
                passed += 1
        
        print(f"\n总体评估: {passed}/{len(criteria)} 条标准通过")

    def analyze_tracking_error_baseline(self):
        """分析跟踪误差基线"""
        print("\n" + "="*80)
        print("实验B - 第二部分: 跟踪误差基线分析")
        print("="*80)
        
        if len(self.tracking_error_df) == 0:
            print("错误: 没有跟踪误差数据")
            return None
        
        # 获取测试参数
        target_velocity = self.tracking_error_df['target_velocity_ms'].iloc[0]
        total_latency_s = self.analysis_results['latency_statistics']['total_delay_ms']['mean'] / 1000.0
        
        # 计算理论误差
        theoretical_error_m = target_velocity * total_latency_s
        
        # 计算实际误差统计
        actual_errors = self.tracking_error_df['longitudinal_error_m']
        actual_error_mean = actual_errors.mean()
        actual_error_std = actual_errors.std()
        
        # 计算一致性
        agreement_ratio = actual_error_mean / theoretical_error_m if theoretical_error_m != 0 else 0
        
        # 存储结果
        self.analysis_results['error_baseline'] = {
            'target_velocity_ms': target_velocity,
            'total_latency_ms': total_latency_s * 1000,
            'theoretical_error_m': theoretical_error_m,
            'actual_error_mean_m': actual_error_mean,
            'actual_error_std_m': actual_error_std,
            'agreement_ratio': agreement_ratio,
            'sample_count': len(actual_errors)
        }
        
        print(f"\n跟踪误差基线分析:")
        print("-" * 60)
        print(f"测试配置:")
        print(f"  目标速度: {target_velocity:.3f} m/s")
        print(f"  总延迟: {total_latency_s*1000:.2f} ms")
        print(f"\n误差分析:")
        print(f"  理论误差 (v×ΔT): {theoretical_error_m:.6f} m")
        print(f"  实际误差均值: {actual_error_mean:.6f} m")
        print(f"  实际误差标准差: {actual_error_std:.6f} m")
        print(f"  一致性比率: {agreement_ratio:.3f}")
        
        # 评估预测准确性
        prediction_accurate = 0.5 <= agreement_ratio <= 1.5
        accuracy_status = "准确" if prediction_accurate else "不准确"
        print(f"\n基线预测评估: {accuracy_status}")
        
        return self.analysis_results['error_baseline']

    def create_comprehensive_plots(self):
        """创建综合分析图表"""
        print(f"\n生成分析图表...")
        
        fig = plt.figure(figsize=(20, 16))
        
        # 图1: 延迟组件对比
        ax1 = plt.subplot(3, 3, 1)
        stats = self.analysis_results['latency_statistics']
        components = ['R2V (A1)', 'V2R', 'Planner (估算)']
        means = [stats['r2v_delay_ms']['mean'], 
                stats['v2r_delay_ms']['mean'], 
                stats['planner_delay_ms']['mean']]
        colors = ['skyblue', 'lightgreen', 'lightcoral']
        
        bars = ax1.bar(components, means, color=colors)
        ax1.set_ylabel('延迟 (ms)')
        ax1.set_title('延迟组件对比 (基于A1复用)')
        ax1.grid(True, alpha=0.3)
        
        # 添加数值标签
        for bar, value in zip(bars, means):
            height = bar.get_height()
            ax1.text(bar.get_x() + bar.get_width()/2., height + max(means)*0.01,
                    f'{value:.1f}ms', ha='center', va='bottom')
        
        # 图2: R2V延迟分布 (来自A1)
        ax2 = plt.subplot(3, 3, 2)
        r2v_data = self.r2v_df['r2v_total_latency_ms']
        ax2.hist(r2v_data, bins=30, alpha=0.7, color='skyblue', edgecolor='black')
        ax2.axvline(r2v_data.mean(), color='red', linestyle='--', linewidth=2,
                   label=f'均值: {r2v_data.mean():.1f}ms')
        ax2.set_xlabel('R2V延迟 (ms)')
        ax2.set_ylabel('频率')
        ax2.set_title('R2V延迟分布 (来自A1实验)')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # 图3: V2R延迟分布
        ax3 = plt.subplot(3, 3, 3)
        v2r_data = self.v2r_df['v2r_latency_ms']
        ax3.hist(v2r_data, bins=30, alpha=0.7, color='lightgreen', edgecolor='black')
        ax3.axvline(v2r_data.mean(), color='red', linestyle='--', linewidth=2,
                   label=f'均值: {v2r_data.mean():.1f}ms')
        ax3.set_xlabel('V2R延迟 (ms)')
        ax3.set_ylabel('频率')
        ax3.set_title('V2R延迟分布')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        # 图4: 延迟组成饼图
        ax4 = plt.subplot(3, 3, 4)
        ax4.pie(means, labels=components, autopct='%1.1f%%', colors=colors)
        ax4.set_title('延迟组成比例')
        
        # 图5: 跟踪误差时间序列
        ax5 = plt.subplot(3, 3, 5)
        if len(self.tracking_error_df) > 0:
            error_baseline = self.analysis_results['error_baseline']
            time_data = self.tracking_error_df['elapsed_time_s']
            error_data = self.tracking_error_df['longitudinal_error_m']
            
            ax5.plot(time_data, error_data, alpha=0.7, linewidth=1, 
                    label='实际误差', color='blue')
            ax5.axhline(y=error_baseline['theoretical_error_m'], color='red', 
                       linestyle='--', linewidth=2,
                       label=f'理论值: {error_baseline["theoretical_error_m"]:.6f}m')
            ax5.axhline(y=error_baseline['actual_error_mean_m'], color='green', 
                       linestyle=':', linewidth=2,
                       label=f'均值: {error_baseline["actual_error_mean_m"]:.6f}m')
            
            ax5.set_xlabel('时间 (s)')
            ax5.set_ylabel('纵向误差 (m)')
            ax5.set_title('跟踪误差 vs 时间')
            ax5.legend()
            ax5.grid(True, alpha=0.3)
        
        # 图6: 误差分布
        ax6 = plt.subplot(3, 3, 6)
        if len(self.tracking_error_df) > 0:
            error_data = self.tracking_error_df['longitudinal_error_m']
            error_baseline = self.analysis_results['error_baseline']
            
            ax6.hist(error_data, bins=30, alpha=0.7, color='skyblue', edgecolor='black')
            ax6.axvline(error_baseline['actual_error_mean_m'], color='green', 
                       linestyle=':', linewidth=2,
                       label=f'均值: {error_baseline["actual_error_mean_m"]:.6f}m')
            ax6.axvline(error_baseline['theoretical_error_m'], color='red', 
                       linestyle='--', linewidth=2,
                       label=f'理论: {error_baseline["theoretical_error_m"]:.6f}m')
            ax6.set_xlabel('纵向误差 (m)')
            ax6.set_ylabel('频率')
            ax6.set_title('误差分布')
            ax6.legend()
            ax6.grid(True, alpha=0.3)
        
        # 图7: 理论vs实际误差对比
        ax7 = plt.subplot(3, 3, 7)
        if 'error_baseline' in self.analysis_results:
            agreement_ratio = self.analysis_results['error_baseline']['agreement_ratio']
            theoretical = self.analysis_results['error_baseline']['theoretical_error_m']
            actual = self.analysis_results['error_baseline']['actual_error_mean_m']
            
            categories = ['理论误差', '实际均值']
            values = [theoretical, actual]
            colors_bar = ['red', 'green']
            
            bars = ax7.bar(categories, values, color=colors_bar, alpha=0.7)
            ax7.set_ylabel('误差 (m)')
            ax7.set_title(f'误差对比 (一致性: {agreement_ratio:.3f})')
            ax7.grid(True, alpha=0.3)
            
            # 添加数值标签
            for bar, value in zip(bars, values):
                height = bar.get_height()
                ax7.text(bar.get_x() + bar.get_width()/2., height + max(values)*0.01,
                        f'{value:.6f}m', ha='center', va='bottom')
        
        plt.tight_layout()
        
        # 保存图表
        plot_filename = os.path.join(self.analysis_dir, f'experiment_b_analysis_{self.analysis_timestamp}.png')
        plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
        print(f"分析图表已保存: {plot_filename}")
        
        return fig

    def generate_report(self):
        """生成综合分析报告"""
        print(f"\n生成综合报告...")
        
        latency_stats = self.analysis_results['latency_statistics']
        
        report_content = f"""# 实验B: 延迟特性及基线跟踪误差分析报告 (精简版)

## 分析摘要
- **分析时间戳**: {self.analysis_timestamp}
- **R2V延迟数据**: {os.path.basename(self.r2v_timing_csv)} (来自A1实验, {len(self.r2v_df)} 条记录)
- **V2R延迟数据**: {os.path.basename(self.v2r_csv)} ({len(self.v2r_df)} 条记录)  
- **跟踪误差数据**: {os.path.basename(self.tracking_error_csv)} ({len(self.tracking_error_df)} 条记录)

## 第一部分: 延迟特性分析 (基于A1复用)

### 关键性能指标

| 组件 | 均值 (ms) | 中位数 (ms) | 标准差 (ms) | P95 (ms) | P99 (ms) |
|------|-----------|-------------|-------------|----------|----------|
| R2V延迟 (A1复用) | {latency_stats['r2v_delay_ms']['mean']:.2f} | {latency_stats['r2v_delay_ms']['median']:.2f} | {latency_stats['r2v_delay_ms']['std']:.2f} | {latency_stats['r2v_delay_ms']['p95']:.2f} | {latency_stats['r2v_delay_ms']['p99']:.2f} |
| V2R延迟 | {latency_stats['v2r_delay_ms']['mean']:.2f} | {latency_stats['v2r_delay_ms']['median']:.2f} | {latency_stats['v2r_delay_ms']['std']:.2f} | {latency_stats['v2r_delay_ms']['p95']:.2f} | {latency_stats['v2r_delay_ms']['p99']:.2f} |
| Planner延迟 (估算) | {latency_stats['planner_delay_ms']['mean']:.2f} | - | - | - | - |
| **总端到端延迟** | **{latency_stats['total_delay_ms']['mean']:.2f}** | - | - | - | - |

### 延迟组成
- R2V贡献: {latency_stats['total_delay_ms']['components']['r2v_contribution_pct']:.1f}%
- Planner贡献: {latency_stats['total_delay_ms']['components']['planner_contribution_pct']:.1f}%  
- V2R贡献: {latency_stats['total_delay_ms']['components']['v2r_contribution_pct']:.1f}%

"""

        if 'error_baseline' in self.analysis_results:
            error_baseline = self.analysis_results['error_baseline']
            report_content += f"""
## 第二部分: 基线跟踪误差分析

### 测试配置
- **目标速度**: {error_baseline['target_velocity_ms']:.3f} m/s
- **总系统延迟**: {error_baseline['total_latency_ms']:.2f} ms
- **测试样本**: {error_baseline['sample_count']} 次测量

### 误差分析结果

| 指标 | 数值 |
|------|------|
| 理论误差 (v × ΔT) | {error_baseline['theoretical_error_m']:.6f} m |
| 实际误差均值 | {error_baseline['actual_error_mean_m']:.6f} m |
| 实际误差标准差 | {error_baseline['actual_error_std_m']:.6f} m |
| 一致性比率 | {error_baseline['agreement_ratio']:.3f} |

### 基线评估
理论模型**{'准确' if 0.5 <= error_baseline['agreement_ratio'] <= 1.5 else '不准确'}**地预测了实际跟踪误差。

"""

        report_content += f"""
## 结论

### 平台性能
总端到端延迟 {latency_stats['total_delay_ms']['mean']:.1f}ms 表明系统性能{'优秀' if latency_stats['total_delay_ms']['mean'] < 100 else '良好' if latency_stats['total_delay_ms']['mean'] < 150 else '可接受'}，适用于ADS测试。

### 误差基线建立  
成功建立了跟踪误差归因的定量基线，为后续测试场景提供参考。

## 方法学说明
- **R2V延迟**: 复用实验A1的结果，测量从真值到感知输出的完整信息管道延迟
- **简化Planner延迟**: 使用估算值，实际值需要Autoware内部测量
- **基线验证**: 通过恒速直线测试验证延迟-误差关系

## 生成文件
- 分析图表: `experiment_b_analysis_{self.analysis_timestamp}.png`
- 详细结果: `experiment_b_results_{self.analysis_timestamp}.json`
- 本报告: `experiment_b_report_{self.analysis_timestamp}.md`
"""
        
        # 保存报告
        report_file = os.path.join(self.analysis_dir, f"experiment_b_report_{self.analysis_timestamp}.md")
        with open(report_file, 'w', encoding='utf-8') as f:
            f.write(report_content)
        
        print(f"报告已保存: {report_file}")

    def setup_directories(self):
        """设置输出目录"""
        self.base_dir = os.path.dirname(os.path.abspath(__file__))
        self.results_dir = os.path.join(self.base_dir, 'results')
        self.analysis_dir = os.path.join(self.results_dir, f'experiment_b_{self.analysis_timestamp}')
        
        for directory in [self.results_dir, self.analysis_dir]:
            os.makedirs(directory, exist_ok=True)

    def setup_plotting(self):
        """设置matplotlib配置"""
        plt.style.use('seaborn-v0_8-whitegrid')
        plt.rcParams['figure.figsize'] = [16, 12]
        plt.rcParams['font.size'] = 12
        plt.rcParams['axes.grid'] = True
        plt.rcParams['grid.alpha'] = 0.3

    def save_results_json(self):
        """保存详细结果到JSON"""
        results_file = os.path.join(self.analysis_dir, f"experiment_b_results_{self.analysis_timestamp}.json")
        
        # 转换numpy类型以便JSON序列化
        def convert_for_json(obj):
            if hasattr(obj, 'item'):
                return obj.item()
            elif isinstance(obj, np.integer):
                return int(obj)
            elif isinstance(obj, np.floating):
                return float(obj)
            else:
                return obj
        
        json_results = {
            'metadata': {
                'analysis_timestamp': self.analysis_timestamp,
                'r2v_file': os.path.basename(self.r2v_timing_csv),
                'v2r_file': os.path.basename(self.v2r_csv),
                'tracking_error_file': os.path.basename(self.tracking_error_csv),
                'r2v_samples': len(self.r2v_df),
                'v2r_samples': len(self.v2r_df),
                'tracking_error_samples': len(self.tracking_error_df),
                'analysis_method': 'simplified_a1_reuse'
            },
            'analysis_results': {
                k: {kk: convert_for_json(vv) if not isinstance(vv, dict) else 
                    {kkk: convert_for_json(vvv) for kkk, vvv in vv.items()} 
                    for kk, vv in v.items()}
                for k, v in self.analysis_results.items()
            }
        }
        
        with open(results_file, 'w', encoding='utf-8') as f:
            json.dump(json_results, f, indent=2, ensure_ascii=False)
        
        print(f"结果JSON已保存: {results_file}")

    def run_complete_analysis(self):
        """运行完整分析流程"""
        print("开始实验B综合分析 (精简版 - 基于A1复用)...")
        
        try:
            # 第一部分: 延迟分析
            self.analyze_latency_components()
            
            # 第二部分: 误差基线分析
            self.analyze_tracking_error_baseline()
            
            # 生成输出
            self.create_comprehensive_plots()
            self.save_results_json()
            self.generate_report()
            
            print(f"\n实验B分析完成!")
            print(f"所有结果已保存到: {self.analysis_dir}")
            return True
            
        except Exception as e:
            print(f"分析过程中出错: {e}")
            return False


def main():
    parser = argparse.ArgumentParser(description='实验B综合分析 (精简版)')
    parser.add_argument('r2v_timing_csv', help='R2V延迟CSV文件路径 (来自A1)')
    parser.add_argument('v2r_csv', help='V2R延迟CSV文件路径')
    parser.add_argument('tracking_error_csv', help='跟踪误差CSV文件路径')
    parser.add_argument('--show-plots', action='store_true', help='显示图表')
    
    args = parser.parse_args()
    
    # 验证输入文件
    input_files = [args.r2v_timing_csv, args.v2r_csv, args.tracking_error_csv]
    for file_path in input_files:
        if not os.path.exists(file_path):
            print(f"错误: 文件不存在: {file_path}")
            return 1
    
    # 运行分析
    analyzer = ExperimentBAnalyzer(args.r2v_timing_csv, args.v2r_csv, args.tracking_error_csv)
    success = analyzer.run_complete_analysis()
    
    if success and args.show_plots:
        plt.show()
    
    return 0 if success else 1


if __name__ == '__main__':
    exit(main())