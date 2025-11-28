#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模型转换 - TFLite转换与量化 - 第二周Day6-7

功能：
1. 加载训练好的Keras模型
2. 转换为TensorFlow Lite格式
3. 量化模型以减小大小（INT8量化）
4. 生成C语言头文件（用于ESP32部署）
5. 测试TFLite模型推理性能

TFLite优势：
- 模型大小显著减小（通常减小4-10倍）
- 推理速度更快
- 适合嵌入式设备（ESP32等）

使用方法：
python 2_模型转换.py --input motion_ai_model.h5 --data 训练数据_平地_双腿.npz --quant int8

输出：
- motion_ai_model.tflite (浮点TFLite模型)
- motion_ai_model_quant.tflite (量化TFLite模型)

注意：
- 不再生成C头文件（太大，920KB）
- ESP32部署请使用ESP-IDF的二进制嵌入功能（详见ESP32部署说明.md）
- 如需C头文件，使用 --generate-header 选项
"""

import numpy as np
import argparse
import tensorflow as tf
from pathlib import Path
import time


class TFLite转换器:
    """
    TensorFlow Lite模型转换器
    """

    def __init__(self, keras_model_file):
        """
        初始化转换器

        参数:
            keras_model_file: Keras模型文件路径
        """
        self.keras_model_file = keras_model_file
        self.keras_model = None
        self.tflite_model = None
        self.tflite_quant_model = None

    def 加载Keras模型(self):
        """加载Keras模型"""
        print(f"\n加载Keras模型: {self.keras_model_file}")

        # 加载模型时禁用编译（避免自定义对象反序列化问题）
        try:
            self.keras_model = tf.keras.models.load_model(self.keras_model_file)
        except (TypeError, ValueError) as e:
            print(f"⚠ 标准加载失败，尝试不编译加载: {e}")
            # 如果加载失败，尝试不编译加载
            self.keras_model = tf.keras.models.load_model(self.keras_model_file, compile=False)
            print("✓ 模型已加载（未编译）")

        print(f"✓ 模型加载成功")
        print(f"  模型大小: {Path(self.keras_model_file).stat().st_size / 1024:.1f} KB")

        # 打印模型摘要
        print("\n模型摘要:")
        self.keras_model.summary()

        return self.keras_model

    def 转换为TFLite_浮点(self, 输出文件='motion_ai_model.tflite'):
        """
        转换为浮点TFLite模型

        参数:
            输出文件: 输出TFLite文件名
        """
        print("\n转换为浮点TFLite模型...")

        # 创建TFLite转换器
        converter = tf.lite.TFLiteConverter.from_keras_model(self.keras_model)

        # 优化选项
        converter.optimizations = [tf.lite.Optimize.DEFAULT]

        # 转换
        self.tflite_model = converter.convert()

        # 保存
        with open(输出文件, 'wb') as f:
            f.write(self.tflite_model)

        原始大小 = Path(self.keras_model_file).stat().st_size
        tflite大小 = Path(输出文件).stat().st_size
        压缩比 = 原始大小 / tflite大小

        print(f"✓ 浮点TFLite模型已保存: {输出文件}")
        print(f"  原始模型大小: {原始大小 / 1024:.1f} KB")
        print(f"  TFLite大小: {tflite大小 / 1024:.1f} KB")
        print(f"  压缩比: {压缩比:.2f}x")

        return 输出文件

    def 转换为TFLite_INT8量化(self, 代表性数据集, 输出文件='motion_ai_model_quant.tflite'):
        """
        转换为INT8量化TFLite模型（完全量化，兼容TFLite Micro）

        参数:
            代表性数据集: 用于量化的代表性数据（生成器函数）
            输出文件: 输出TFLite文件名
        """
        print("\n转换为INT8量化TFLite模型（完全量化）...")

        # 创建TFLite转换器
        converter = tf.lite.TFLiteConverter.from_keras_model(self.keras_model)

        # 设置优化选项
        converter.optimizations = [tf.lite.Optimize.DEFAULT]

        # 设置代表性数据集（用于量化校准）
        converter.representative_dataset = 代表性数据集

        # ⚠️ 关键修改：完全INT8量化，兼容TFLite Micro
        # 不设置 target_spec.supported_ops，让TFLite自动选择
        # 不设置 inference_input_type 和 inference_output_type
        # 这样会生成完全量化的模型，而不是混合精度模型

        # 转换
        try:
            self.tflite_quant_model = converter.convert()
            print("✓ 完全量化成功（兼容TFLite Micro）")
        except Exception as e:
            print(f"⚠ 完全量化失败: {e}")
            print("  尝试动态范围量化...")
            # 回退到动态范围量化
            converter = tf.lite.TFLiteConverter.from_keras_model(self.keras_model)
            converter.optimizations = [tf.lite.Optimize.DEFAULT]
            self.tflite_quant_model = converter.convert()

        # 保存
        with open(输出文件, 'wb') as f:
            f.write(self.tflite_quant_model)

        原始大小 = Path(self.keras_model_file).stat().st_size
        量化大小 = Path(输出文件).stat().st_size
        压缩比 = 原始大小 / 量化大小

        print(f"✓ 量化TFLite模型已保存: {输出文件}")
        print(f"  原始模型大小: {原始大小 / 1024:.1f} KB")
        print(f"  量化后大小: {量化大小 / 1024:.1f} KB")
        print(f"  压缩比: {压缩比:.2f}x")

        return 输出文件

    def 生成C头文件(self, tflite_file, 输出文件='model_data.h'):
        """
        生成C语言头文件

        参数:
            tflite_file: TFLite文件路径
            输出文件: 输出C头文件名
        """
        print(f"\n生成C头文件...")

        # 读取TFLite模型
        with open(tflite_file, 'rb') as f:
            tflite_data = f.read()

        # 生成C数组
        with open(输出文件, 'w', encoding='utf-8') as f:
            f.write("// 自动生成的TFLite模型数据\n")
            f.write(f"// 源文件: {tflite_file}\n")
            f.write(f"// 大小: {len(tflite_data)} bytes\n\n")

            f.write("#ifndef MODEL_DATA_H\n")
            f.write("#define MODEL_DATA_H\n\n")

            f.write("#include <stdint.h>\n\n")

            # 模型数据数组
            f.write(f"// TFLite模型数据 ({len(tflite_data)} bytes)\n")
            f.write("const unsigned char model_data[] __attribute__((aligned(8))) = {\n")

            # 每行16个字节
            for i in range(0, len(tflite_data), 16):
                chunk = tflite_data[i:i+16]
                hex_str = ', '.join([f'0x{b:02x}' for b in chunk])
                f.write(f"    {hex_str},\n")

            f.write("};\n\n")

            f.write(f"const unsigned int model_data_len = {len(tflite_data)};\n\n")

            f.write("#endif // MODEL_DATA_H\n")

        print(f"✓ C头文件已生成: {输出文件}")
        print(f"  文件大小: {Path(输出文件).stat().st_size / 1024:.1f} KB")

    def 测试TFLite推理(self, tflite_file, 测试数据, 测试次数=100):
        """
        测试TFLite模型推理性能

        参数:
            tflite_file: TFLite文件路径
            测试数据: 测试数据数组
            测试次数: 推理次数
        """
        print(f"\n测试TFLite推理性能...")

        # 加载TFLite模型
        interpreter = tf.lite.Interpreter(model_path=tflite_file)
        interpreter.allocate_tensors()

        # 获取输入输出详情
        input_details = interpreter.get_input_details()
        output_details = interpreter.get_output_details()

        print(f"输入张量:")
        for idx, detail in enumerate(input_details):
            print(f"  [{idx}] shape={detail['shape']}, dtype={detail['dtype']}, name={detail['name']}")

        print(f"输出张量:")
        for idx, detail in enumerate(output_details):
            print(f"  [{idx}] shape={detail['shape']}, dtype={detail['dtype']}, name={detail['name']}")

        # 准备测试数据
        if len(测试数据) > 0:
            sample = 测试数据[0:1]  # 取第一个样本: (1, 50, 4)
            # 确保数据形状为 (1, 50, 4) - 双腿联合输入
            if len(sample.shape) == 3:
                # 数据已经是 (1, 50, 4) 格式
                sample = sample.astype(np.float32)
            else:
                # 数据是 (1, 200) 格式，需要reshape为 (1, 50, 4)
                sample = sample.reshape(1, 50, 4).astype(np.float32)
        else:
            # 使用随机数据
            input_shape = input_details[0]['shape']
            sample = np.random.randn(*input_shape).astype(np.float32)

        # 预热
        for _ in range(10):
            interpreter.set_tensor(input_details[0]['index'], sample)
            interpreter.invoke()

        # 计时
        times = []
        for _ in range(测试次数):
            start = time.perf_counter()
            interpreter.set_tensor(input_details[0]['index'], sample)
            interpreter.invoke()
            end = time.perf_counter()
            times.append((end - start) * 1000)  # 转换为毫秒

        平均时间 = np.mean(times)
        最小时间 = np.min(times)
        最大时间 = np.max(times)

        print(f"\n推理性能测试结果 (测试次数={测试次数}):")
        print(f"  平均推理时间: {平均时间:.2f} ms")
        print(f"  最小推理时间: {最小时间:.2f} ms")
        print(f"  最大推理时间: {最大时间:.2f} ms")
        print(f"  推理频率: {1000/平均时间:.1f} Hz")

        return 平均时间


def 创建代表性数据集生成器(X_data, 样本数=100):
    """
    创建代表性数据集生成器（用于量化）

    参数:
        X_data: 输入数据 (形状: [N, 50, 4])
        样本数: 使用的样本数量

    返回:
        生成器函数
    """
    def representative_dataset():
        # 随机选择样本
        indices = np.random.choice(len(X_data), min(样本数, len(X_data)), replace=False)
        for i in indices:
            # 确保数据形状为 [1, 50, 4]
            sample = X_data[i:i+1].astype(np.float32)
            # 验证形状
            if len(sample.shape) == 2:
                # 如果是 [1, 200]，reshape为 [1, 50, 4]
                sample = sample.reshape(1, 50, 4)
            # TFLite期望输入是一个列表
            yield [sample]

    return representative_dataset


def main():
    parser = argparse.ArgumentParser(description='模型转换 - TFLite转换与量化')
    parser.add_argument('--input', '-i', type=str, required=True,
                        help='输入Keras模型文件 (.h5)')
    parser.add_argument('--data', '-d', type=str, default=None,
                        help='训练数据NPZ文件（用于量化校准）')
    parser.add_argument('--quant', '-q', type=str, default='int8',
                        choices=['none', 'float16', 'int8'],
                        help='量化类型 (默认: int8)')
    parser.add_argument('--output-dir', '-o', type=str, default='.',
                        help='输出目录 (默认: 当前目录)')
    parser.add_argument('--generate-header', action='store_true',
                        help='生成C头文件（不推荐，文件太大）')

    args = parser.parse_args()

    print("=" * 70)
    print("模型转换 - TFLite转换与量化")
    print("=" * 70)

    # 创建输出目录
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # 1. 初始化转换器
    转换器 = TFLite转换器(args.input)
    转换器.加载Keras模型()

    # 2. 转换为浮点TFLite
    浮点文件 = output_dir / 'motion_ai_model.tflite'
    转换器.转换为TFLite_浮点(输出文件=str(浮点文件))

    # 3. 量化（如果选择）
    量化文件 = None
    if args.quant != 'none':
        if args.data is None:
            print("\n⚠ 警告: 未提供训练数据，跳过INT8量化")
            print("  使用 --data 参数提供训练数据NPZ文件以启用INT8量化")
        else:
            # 加载训练数据
            print(f"\n加载训练数据用于量化校准: {args.data}")
            data = np.load(args.data, allow_pickle=True)
            X = data['X']
            print(f"✓ 加载完成: {len(X)} 样本")

            # 创建代表性数据集
            代表性数据集 = 创建代表性数据集生成器(X, 样本数=100)

            # INT8量化
            量化文件 = output_dir / 'motion_ai_model_quant.tflite'
            转换器.转换为TFLite_INT8量化(
                代表性数据集=代表性数据集,
                输出文件=str(量化文件)
            )

    # 4. 生成C头文件（可选，默认不生成）
    c头文件 = None
    if args.generate_header:
        print("\n⚠ 警告: C头文件会非常大（~920KB），建议使用ESP-IDF二进制嵌入")
        print("  详见: ESP32部署说明.md")
        最终模型 = 量化文件 if 量化文件 else 浮点文件
        c头文件 = output_dir / 'model_data.h'
        转换器.生成C头文件(
            tflite_file=str(最终模型),
            输出文件=str(c头文件)
        )

    # 5. 测试推理性能
    if args.data:
        data = np.load(args.data, allow_pickle=True)
        X = data['X']

        print("\n" + "=" * 70)
        print("测试推理性能")
        print("=" * 70)

        # 测试浮点模型
        print("\n【浮点TFLite模型】")
        浮点推理时间 = 转换器.测试TFLite推理(str(浮点文件), X, 测试次数=100)

        # 测试量化模型
        if 量化文件:
            print("\n【量化TFLite模型】")
            量化推理时间 = 转换器.测试TFLite推理(str(量化文件), X, 测试次数=100)

            print(f"\n性能对比:")
            print(f"  浮点模型: {浮点推理时间:.2f} ms")
            print(f"  量化模型: {量化推理时间:.2f} ms")
            if 量化推理时间 < 浮点推理时间:
                加速比 = 浮点推理时间 / 量化推理时间
                print(f"  加速比: {加速比:.2f}x")

    print("\n" + "=" * 70)
    print("✓ 模型转换完成！")
    print("=" * 70)
    print(f"\n生成文件:")
    print(f"  - {浮点文件} ({Path(浮点文件).stat().st_size / 1024:.1f} KB)")
    if 量化文件:
        print(f"  - {量化文件} ({Path(量化文件).stat().st_size / 1024:.1f} KB)")
    if c头文件:
        print(f"  - {c头文件} ({Path(c头文件).stat().st_size / 1024:.1f} KB)")

    print(f"\n下一步 - ESP32部署:")
    print(f"  推荐方案: 使用ESP-IDF二进制嵌入")
    print(f"  1. 将 {量化文件 if 量化文件 else 浮点文件} 复制到ESP32项目的 main/model/ 目录")
    print(f"  2. 在 main/CMakeLists.txt 中添加: EMBED_FILES \"model/{(量化文件 if 量化文件 else 浮点文件).name}\"")
    print(f"  3. 在代码中使用 extern const uint8_t model_data[] 引用嵌入的模型数据")
    print(f"\n  优势: Flash占用仅 {Path(量化文件 if 量化文件 else 浮点文件).stat().st_size / 1024:.1f} KB（vs C头文件 ~920 KB）")


if __name__ == '__main__':
    main()
