#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
理想曲线提取工具

功能：
1. 从标注数据中识别完整的运动周期
2. 提取每个阶段的理想速度曲线
3. 对多个周期取平均，生成理想曲线模板
4. 导出为C头文件供ESP32使用
5. 可视化理想曲线

使用方法：
# 同时提取双腿（m1和m2）理想曲线（默认）
python 4_理想曲线提取.py --input 平地数据_已修正.csv --scene 平地


# 也可使用已标注文件
python 4_理想曲线提取.py --input 平地数据_已标注.csv --scene 平地
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
import argparse
from pathlib import Path

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial Unicode MS']
plt.rcParams['axes.unicode_minus'] = False


class 理想曲线提取器:
    """
    从标注数据中提取理想曲线
    """

    IDLE = 0
    LIFTING = 1
    PRESSING = 2

    阶段名称 = {
        0: '静止',
        1: '抬腿',
        2: '压腿'
    }

    def __init__(self, df, 电机='m1'):
        """
        初始化提取器

        参数:
            df: 已标注的DataFrame
            电机: 'm1' 或 'm2'
        """
        self.df = df
        self.电机 = 电机
        self.速度列 = f'{电机}_vel'
        self.位置列 = f'{电机}_pos'
        self.标签列 = f'{电机}_label'

    def 寻找完整周期(self):
        """
        寻找所有完整的运动周期

        周期定义：IDLE → LIFTING → PRESSING → IDLE

        返回:
            周期列表，每个周期是一个包含起始和结束索引的dict
        """
        标签 = self.df[self.标签列].values
        周期列表 = []

        i = 0
        while i < len(标签):
            # 找到IDLE的起始
            while i < len(标签) and 标签[i] != self.IDLE:
                i += 1

            if i >= len(标签):
                break

            周期起始 = i

            # 期望序列: IDLE → LIFTING → PRESSING → IDLE
            阶段序列 = []
            当前阶段 = 标签[i]
            阶段起始 = i

            while i < len(标签):
                if 标签[i] != 当前阶段:
                    # 记录阶段
                    阶段序列.append({
                        'phase': 当前阶段,
                        'start': 阶段起始,
                        'end': i
                    })
                    当前阶段 = 标签[i]
                    阶段起始 = i

                    # 检查是否回到IDLE（周期结束）
                    if 当前阶段 == self.IDLE and len(阶段序列) >= 2:
                        # 找到完整周期
                        周期结束 = i
                        周期列表.append({
                            'start': 周期起始,
                            'end': 周期结束,
                            'phases': 阶段序列
                        })
                        break
                i += 1

        print(f"找到 {len(周期列表)} 个完整运动周期")
        return 周期列表

    def 提取阶段曲线(self, 周期列表, 目标长度=100):
        """
        从多个周期中提取每个阶段的平均曲线

        参数:
            周期列表: 完整周期列表
            目标长度: 插值后的曲线长度

        返回:
            dict，包含每个阶段的理想曲线
        """
        阶段曲线 = {
            self.LIFTING: [],
            self.PRESSING: []
        }

        # 从每个周期提取阶段曲线
        for 周期 in 周期列表:
            for 阶段 in 周期['phases']:
                阶段类型 = 阶段['phase']

                if 阶段类型 in [self.LIFTING, self.PRESSING]:
                    # 提取该阶段的速度数据
                    起始 = 阶段['start']
                    结束 = 阶段['end']
                    速度序列 = self.df[self.速度列].iloc[起始:结束].values

                    # 插值到目标长度
                    if len(速度序列) > 2:
                        原始x = np.linspace(0, 1, len(速度序列))
                        目标x = np.linspace(0, 1, 目标长度)
                        插值函数 = interp1d(原始x, 速度序列, kind='cubic', fill_value='extrapolate')
                        插值曲线 = 插值函数(目标x)

                        阶段曲线[阶段类型].append(插值曲线)

        # 计算平均曲线
        理想曲线 = {}
        for 阶段类型, 曲线列表 in 阶段曲线.items():
            if len(曲线列表) > 0:
                平均曲线 = np.mean(曲线列表, axis=0)
                标准差 = np.std(曲线列表, axis=0)
                理想曲线[阶段类型] = {
                    'mean': 平均曲线,
                    'std': 标准差,
                    'count': len(曲线列表)
                }
                print(f"  {self.阶段名称[阶段类型]}: 平均了 {len(曲线列表)} 个样本")
            else:
                print(f"  {self.阶段名称[阶段类型]}: 无数据")

        return 理想曲线

    def 可视化理想曲线(self, 理想曲线, 输出文件='理想曲线.png'):
        """
        可视化理想曲线

        参数:
            理想曲线: 理想曲线dict
            输出文件: 输出图片文件名
        """
        fig, axes = plt.subplots(2, 1, figsize=(12, 8))

        阶段顺序 = [self.LIFTING, self.PRESSING]
        颜色 = ['green', 'red']

        for idx, 阶段类型 in enumerate(阶段顺序):
            ax = axes[idx]
            if 阶段类型 in 理想曲线:
                数据 = 理想曲线[阶段类型]
                平均曲线 = 数据['mean']
                标准差 = 数据['std']
                x = np.arange(len(平均曲线))

                # 绘制平均曲线
                腿部标记 = '左腿' if self.电机 == 'm1' else '右腿'
                ax.plot(x, 平均曲线, color=颜色[idx], linewidth=2, label=f'平均曲线 ({腿部标记})')

                # 绘制标准差区域
                ax.fill_between(x,
                               平均曲线 - 标准差,
                               平均曲线 + 标准差,
                               color=颜色[idx], alpha=0.2, label='标准差范围')

                ax.set_title(f'{self.阶段名称[阶段类型]}阶段理想曲线 - {腿部标记} (n={数据["count"]})',
                           fontsize=14, fontweight='bold')
                ax.set_xlabel('归一化时间 (0-100)', fontsize=12)
                ax.set_ylabel('速度 (rad/s)', fontsize=12)
                ax.grid(True, alpha=0.3)
                ax.legend(loc='upper right')
                ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)

        plt.tight_layout()
        plt.savefig(输出文件, dpi=150, bbox_inches='tight')
        print(f"✓ 理想曲线可视化已保存: {输出文件}")
        # plt.show()

    @staticmethod
    def 可视化双腿理想曲线(理想曲线_m1, 理想曲线_m2, 输出文件='理想曲线_双腿对比.png'):
        """
        可视化双腿理想曲线对比

        参数:
            理想曲线_m1: m1电机理想曲线dict
            理想曲线_m2: m2电机理想曲线dict
            输出文件: 输出图片文件名
        """
        fig, axes = plt.subplots(2, 1, figsize=(14, 10))

        LIFTING = 1
        PRESSING = 2
        阶段名称 = {1: '抬腿', 2: '压腿'}
        阶段顺序 = [LIFTING, PRESSING]
        颜色_m1 = ['#2E7D32', '#C62828']  # 绿、红
        颜色_m2 = ['#66BB6A', '#EF5350']  # 浅绿、浅红

        for idx, 阶段类型 in enumerate(阶段顺序):
            ax = axes[idx]

            # 绘制m1曲线
            if 阶段类型 in 理想曲线_m1:
                数据 = 理想曲线_m1[阶段类型]
                平均曲线 = 数据['mean']
                标准差 = 数据['std']
                x = np.arange(len(平均曲线))

                ax.plot(x, 平均曲线, color=颜色_m1[idx], linewidth=2.5,
                       label=f'左腿 (m1, n={数据["count"]})', linestyle='-')
                ax.fill_between(x, 平均曲线 - 标准差, 平均曲线 + 标准差,
                               color=颜色_m1[idx], alpha=0.15)

            # 绘制m2曲线
            if 阶段类型 in 理想曲线_m2:
                数据 = 理想曲线_m2[阶段类型]
                平均曲线 = 数据['mean']
                标准差 = 数据['std']
                x = np.arange(len(平均曲线))

                ax.plot(x, 平均曲线, color=颜色_m2[idx], linewidth=2.5,
                       label=f'右腿 (m2, n={数据["count"]})', linestyle='--')
                ax.fill_between(x, 平均曲线 - 标准差, 平均曲线 + 标准差,
                               color=颜色_m2[idx], alpha=0.15)

            ax.set_title(f'{阶段名称[阶段类型]}阶段理想曲线 - 双腿对比',
                       fontsize=14, fontweight='bold')
            ax.set_xlabel('归一化时间 (0-100)', fontsize=12)
            ax.set_ylabel('速度 (rad/s)', fontsize=12)
            ax.grid(True, alpha=0.3)
            ax.legend(loc='upper right', fontsize=10)
            ax.axhline(y=0, color='k', linestyle='--', alpha=0.3)

        plt.tight_layout()
        plt.savefig(输出文件, dpi=150, bbox_inches='tight')
        print(f"✓ 双腿对比曲线已保存: {输出文件}")
        # plt.show()

    def 导出为C头文件(self, 理想曲线, 场景名称, 输出文件='ideal_curves.h'):
        """
        导出为C头文件

        参数:
            理想曲线: 理想曲线dict
            场景名称: 场景名称 (如 'flat_ground', 'climbing')
            输出文件: 输出C头文件名
        """
        with open(输出文件, 'w', encoding='utf-8') as f:
            f.write("// 自动生成的理想曲线数据\n")
            f.write(f"// 场景: {场景名称}\n")
            f.write(f"// 电机: {self.电机}\n")
            f.write("// 生成时间: " + pd.Timestamp.now().strftime('%Y-%m-%d %H:%M:%S') + "\n\n")
            f.write("#ifndef IDEAL_CURVES_H\n")
            f.write("#define IDEAL_CURVES_H\n\n")
            f.write("#include <stdint.h>\n\n")

            阶段映射 = {
                self.LIFTING: 'lifting',
                self.PRESSING: 'pressing'
            }

            for 阶段类型, 阶段英文 in 阶段映射.items():
                if 阶段类型 in 理想曲线:
                    数据 = 理想曲线[阶段类型]
                    曲线 = 数据['mean']

                    # 数组名称
                    数组名 = f'ideal_{场景名称}_{self.电机}_{阶段英文}_vel'

                    # 写注释
                    腿部标记 = '左腿' if self.电机 == 'm1' else '右腿'
                    f.write(f"// {self.阶段名称[阶段类型]}阶段理想速度曲线 ({腿部标记})\n")
                    f.write(f"// 样本数: {数据['count']}\n")
                    f.write(f"// 长度: {len(曲线)}\n")

                    # 写数组
                    f.write(f"static const float {数组名}[] = {{\n")

                    # 每行10个数据
                    for i in range(0, len(曲线), 10):
                        f.write("    ")
                        chunk = 曲线[i:i+10]
                        for j, val in enumerate(chunk):
                            f.write(f"{val:.6f}f")
                            if i + j < len(曲线) - 1:
                                f.write(", ")
                        f.write("\n")

                    f.write("};\n")
                    f.write(f"static const uint16_t {数组名}_len = {len(曲线)};\n\n")

                    # 统计信息（作为注释）
                    f.write(f"// 统计信息:\n")
                    f.write(f"//   峰值速度: {np.max(np.abs(曲线)):.3f} rad/s\n")
                    f.write(f"//   平均速度: {np.mean(曲线):.3f} rad/s\n")
                    f.write(f"//   标准差: {np.mean(数据['std']):.3f} rad/s\n\n")

            f.write("#endif // IDEAL_CURVES_H\n")

        print(f"✓ C头文件已导出: {输出文件}")

    @staticmethod
    def 导出双腿C头文件(理想曲线_m1, 理想曲线_m2, 场景名称, 输出文件='ideal_curves.h'):
        """
        导出包含双腿数据的C头文件

        参数:
            理想曲线_m1: m1电机理想曲线dict
            理想曲线_m2: m2电机理想曲线dict
            场景名称: 场景名称 (如 'flat_ground', 'climbing')
            输出文件: 输出C头文件名
        """
        with open(输出文件, 'w', encoding='utf-8') as f:
            f.write("// 自动生成的理想曲线数据 - 双腿\n")
            f.write(f"// 场景: {场景名称}\n")
            f.write("// 包含: m1 (左腿) 和 m2 (右腿)\n")
            f.write("// 生成时间: " + pd.Timestamp.now().strftime('%Y-%m-%d %H:%M:%S') + "\n\n")
            f.write("#ifndef IDEAL_CURVES_H\n")
            f.write("#define IDEAL_CURVES_H\n\n")
            f.write("#include <stdint.h>\n\n")

            LIFTING = 1
            PRESSING = 2
            阶段名称 = {1: '抬腿', 2: '压腿'}
            阶段映射 = {
                LIFTING: 'lifting',
                PRESSING: 'pressing'
            }

            # 导出所有曲线
            for 电机名称, 理想曲线, 腿部标记 in [('m1', 理想曲线_m1, '左腿'), ('m2', 理想曲线_m2, '右腿')]:
                f.write(f"// ==================== {电机名称.upper()} ({腿部标记}) ====================\n\n")

                for 阶段类型, 阶段英文 in 阶段映射.items():
                    if 阶段类型 in 理想曲线:
                        数据 = 理想曲线[阶段类型]
                        曲线 = 数据['mean']

                        # 数组名称
                        数组名 = f'ideal_{场景名称}_{电机名称}_{阶段英文}_vel'

                        # 写注释
                        f.write(f"// {阶段名称[阶段类型]}阶段理想速度曲线 ({腿部标记})\n")
                        f.write(f"// 样本数: {数据['count']}\n")
                        f.write(f"// 长度: {len(曲线)}\n")

                        # 写数组
                        f.write(f"static const float {数组名}[] = {{\n")

                        # 每行10个数据
                        for i in range(0, len(曲线), 10):
                            f.write("    ")
                            chunk = 曲线[i:i+10]
                            for j, val in enumerate(chunk):
                                f.write(f"{val:.6f}f")
                                if i + j < len(曲线) - 1:
                                    f.write(", ")
                            f.write("\n")

                        f.write("};\n")
                        f.write(f"static const uint16_t {数组名}_len = {len(曲线)};\n\n")

                        # 统计信息（作为注释）
                        f.write(f"// 统计信息:\n")
                        f.write(f"//   峰值速度: {np.max(np.abs(曲线)):.3f} rad/s\n")
                        f.write(f"//   平均速度: {np.mean(曲线):.3f} rad/s\n")
                        f.write(f"//   标准差: {np.mean(数据['std']):.3f} rad/s\n\n")

            f.write("#endif // IDEAL_CURVES_H\n")

        print(f"✓ 双腿C头文件已导出: {输出文件}")


def main():
    parser = argparse.ArgumentParser(description='理想曲线提取工具')
    parser.add_argument('--input', '-i', type=str, required=True,
                        help='输入已标注的CSV文件')
    parser.add_argument('--motor', '-m', type=str, default='both',
                        choices=['m1', 'm2', 'both'], help='选择电机 (m1/m2/both，默认: both)')
    parser.add_argument('--scene', '-s', type=str, required=True,
                        choices=['平地', '爬楼'], help='场景名称')
    parser.add_argument('--length', '-l', type=int, default=100,
                        help='曲线长度 (默认: 100)')
    parser.add_argument('--output-h', type=str, default=None,
                        help='输出C头文件名')
    parser.add_argument('--output-png', type=str, default=None,
                        help='输出可视化图片文件名')

    args = parser.parse_args()

    # 场景英文映射
    场景映射 = {
        '平地': 'flat_ground',
        '爬楼': 'climbing'
    }
    场景英文 = 场景映射[args.scene]

    # 加载数据
    print(f"加载数据: {args.input}")
    df = pd.read_csv(args.input)
    print(f"✓ 加载完成: {len(df)} 行")

    if args.motor == 'both':
        # 同时处理双腿
        print("\n========== 处理双腿数据 ==========")

        # 生成默认输出文件名
        if args.output_h is None:
            args.output_h = f'ideal_curves_{场景英文}_both.h'
        if args.output_png is None:
            args.output_png = f'理想曲线_{args.scene}_双腿对比.png'

        理想曲线_m1 = None
        理想曲线_m2 = None

        # 处理 m1
        print("\n---------- 处理 m1 (左腿) ----------")
        提取器_m1 = 理想曲线提取器(df, 电机='m1')
        print("寻找完整运动周期...")
        周期列表_m1 = 提取器_m1.寻找完整周期()

        if len(周期列表_m1) > 0:
            print(f"提取理想曲线 (目标长度: {args.length})...")
            理想曲线_m1 = 提取器_m1.提取阶段曲线(周期列表_m1, 目标长度=args.length)
            提取器_m1.可视化理想曲线(理想曲线_m1, 输出文件=f'理想曲线_{args.scene}_m1.png')
        else:
            print("✗ m1未找到完整周期")

        # 处理 m2
        print("\n---------- 处理 m2 (右腿) ----------")
        提取器_m2 = 理想曲线提取器(df, 电机='m2')
        print("寻找完整运动周期...")
        周期列表_m2 = 提取器_m2.寻找完整周期()

        if len(周期列表_m2) > 0:
            print(f"提取理想曲线 (目标长度: {args.length})...")
            理想曲线_m2 = 提取器_m2.提取阶段曲线(周期列表_m2, 目标长度=args.length)
            提取器_m2.可视化理想曲线(理想曲线_m2, 输出文件=f'理想曲线_{args.scene}_m2.png')
        else:
            print("✗ m2未找到完整周期")

        # 双腿对比可视化
        if 理想曲线_m1 is not None and 理想曲线_m2 is not None:
            print("\n生成双腿对比可视化...")
            理想曲线提取器.可视化双腿理想曲线(理想曲线_m1, 理想曲线_m2, 输出文件=args.output_png)

            print("\n导出双腿C头文件...")
            理想曲线提取器.导出双腿C头文件(理想曲线_m1, 理想曲线_m2, 场景英文, 输出文件=args.output_h)

            print("\n✓ 双腿理想曲线提取完成！")
            print(f"\n生成文件:")
            print(f"  - {args.output_h} (包含双腿数据)")
            print(f"  - {args.output_png} (双腿对比)")
            print(f"  - 理想曲线_{args.scene}_m1.png (m1单独)")
            print(f"  - 理想曲线_{args.scene}_m2.png (m2单独)")
        else:
            print("\n✗ 无法生成双腿对比图：至少一条腿缺少数据")
            if 理想曲线_m1 is not None:
                print("仅导出m1数据...")
                提取器_m1.导出为C头文件(理想曲线_m1, 场景英文, 输出文件=f'ideal_curves_{场景英文}_m1.h')
            if 理想曲线_m2 is not None:
                print("仅导出m2数据...")
                提取器_m2.导出为C头文件(理想曲线_m2, 场景英文, 输出文件=f'ideal_curves_{场景英文}_m2.h')

    else:
        # 单腿处理（原有逻辑）
        # 生成默认输出文件名
        if args.output_h is None:
            args.output_h = f'ideal_curves_{场景英文}_{args.motor}.h'
        if args.output_png is None:
            args.output_png = f'理想曲线_{args.scene}_{args.motor}.png'

        # 创建提取器
        提取器 = 理想曲线提取器(df, 电机=args.motor)

        # 寻找完整周期
        print("\n寻找完整运动周期...")
        周期列表 = 提取器.寻找完整周期()

        if len(周期列表) == 0:
            print("✗ 未找到完整周期，请检查数据标注是否正确")
            return

        # 提取理想曲线
        print(f"\n提取理想曲线 (目标长度: {args.length})...")
        理想曲线 = 提取器.提取阶段曲线(周期列表, 目标长度=args.length)

        # 可视化
        print("\n生成可视化...")
        提取器.可视化理想曲线(理想曲线, 输出文件=args.output_png)

        # 导出C头文件
        print("\n导出C头文件...")
        提取器.导出为C头文件(理想曲线, 场景英文, 输出文件=args.output_h)

        print("\n✓ 理想曲线提取完成！")
        print(f"\n生成文件:")
        print(f"  - {args.output_h}")
        print(f"  - {args.output_png}")


if __name__ == '__main__':
    main()
