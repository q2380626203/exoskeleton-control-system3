#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
模型输出调试工具

功能:
1. 检查H5模型和TFLite模型的输出顺序
2. 对比两个模型的推理结果
3. 验证输出张量的名称、形状和顺序

使用方法:
python debug_model_outputs.py
"""

import numpy as np
import pandas as pd
from tensorflow import keras
import tensorflow as tf

def test_h5_model():
    """测试H5模型的输出"""
    print("=" * 70)
    print("1. 测试H5模型")
    print("=" * 70)

    # 加载模型
    model = keras.models.load_model('motion_ai_model.h5')

    # 打印模型输出信息
    print(f"\n模型输出层信息:")
    for i, output in enumerate(model.outputs):
        print(f"  输出{i}: 名称='{output.name}', 形状={output.shape}")

    # 创建测试数据
    test_input = np.random.randn(1, 50, 2).astype(np.float32)

    # 推理
    predictions = model.predict(test_input, verbose=0)

    print(f"\n输出数量: {len(predictions)}")
    for i, pred in enumerate(predictions):
        print(f"  输出{i}: 形状={pred.shape}, 样例值={pred[0][:3] if len(pred[0]) > 3 else pred[0]}")

    return predictions

def test_tflite_model():
    """测试TFLite模型的输出"""
    print("\n" + "=" * 70)
    print("2. 测试TFLite模型")
    print("=" * 70)

    # 加载TFLite模型
    interpreter = tf.lite.Interpreter(model_path='motion_ai_model_quant.tflite')
    interpreter.allocate_tensors()

    # 获取输入输出详情
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()

    print(f"\n输入张量信息:")
    for i, detail in enumerate(input_details):
        print(f"  输入{i}:")
        print(f"    名称: {detail['name']}")
        print(f"    形状: {detail['shape']}")
        print(f"    数据类型: {detail['dtype']}")
        print(f"    索引: {detail['index']}")

    print(f"\n输出张量信息:")
    for i, detail in enumerate(output_details):
        print(f"  输出{i}:")
        print(f"    名称: {detail['name']}")
        print(f"    形状: {detail['shape']}")
        print(f"    数据类型: {detail['dtype']}")
        print(f"    索引: {detail['index']}")
        print(f"    量化参数: scale={detail.get('quantization', (None,))[0]}, zero_point={detail.get('quantization_parameters', {}).get('zero_points', None)}")

    # 创建测试数据
    test_input = np.random.randn(1, 50, 2).astype(np.float32)

    # 设置输入
    interpreter.set_tensor(input_details[0]['index'], test_input)

    # 推理
    interpreter.invoke()

    # 获取输出
    print(f"\n输出结果:")
    outputs = []
    for i, detail in enumerate(output_details):
        output_data = interpreter.get_tensor(detail['index'])
        outputs.append(output_data)
        print(f"  输出{i} ({detail['name']}): 形状={output_data.shape}, 样例值={output_data[0][:3] if len(output_data[0]) > 3 else output_data[0]}")

    return outputs, output_details

def compare_real_data():
    """使用真实数据对比H5和TFLite模型"""
    print("\n" + "=" * 70)
    print("3. 使用真实数据对比")
    print("=" * 70)

    # 加载真实数据
    df = pd.read_csv('f:/yrh/esp32-espidf/yswgg/平地数据_已推理.csv')
    print(f"\n加载数据: {len(df)} 行")

    # 创建一个窗口
    window_idx = 100  # 测试第100个窗口
    window = np.zeros((1, 50, 2), dtype=np.float32)
    for i in range(50):
        idx = window_idx - 50 + i
        window[0, i, 0] = df['m1_vel'].values[idx]
        window[0, i, 1] = df['m2_vel'].values[idx]

    print(f"\n测试窗口 #{window_idx}:")
    print(f"  m1速度范围: [{window[0, :, 0].min():.3f}, {window[0, :, 0].max():.3f}]")
    print(f"  m2速度范围: [{window[0, :, 1].min():.3f}, {window[0, :, 1].max():.3f}]")

    # H5模型推理
    print("\nH5模型推理:")
    h5_model = keras.models.load_model('motion_ai_model.h5')
    h5_preds = h5_model.predict(window, verbose=0)

    阶段名称 = {0: '静止', 1: '抬腿', 2: '压腿'}

    print(f"  场景输出: {h5_preds[0][0]}")
    print(f"  m1阶段输出: {h5_preds[1][0]} -> 预测={阶段名称[np.argmax(h5_preds[1][0])]}")
    print(f"  m2阶段输出: {h5_preds[2][0]} -> 预测={阶段名称[np.argmax(h5_preds[2][0])]}")
    print(f"  参数输出: {h5_preds[3][0]}")

    # TFLite模型推理
    print("\nTFLite模型推理:")
    interpreter = tf.lite.Interpreter(model_path='motion_ai_model_quant.tflite')
    interpreter.allocate_tensors()

    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()

    interpreter.set_tensor(input_details[0]['index'], window)
    interpreter.invoke()

    for i, detail in enumerate(output_details):
        output_data = interpreter.get_tensor(detail['index'])
        print(f"  输出{i} ({detail['name']}): {output_data[0]}")

        # 如果是分类输出,显示预测类别
        if len(output_data[0]) == 3:  # 阶段分类
            pred_label = np.argmax(output_data[0])
            print(f"    -> 预测={阶段名称[pred_label]}")
        elif len(output_data[0]) == 2:  # 场景分类
            print(f"    -> 预测={'平地' if np.argmax(output_data[0]) == 0 else '爬楼'}")

    # 对比
    print("\n对比分析:")
    print(f"  H5输出顺序: scene, m1_phase, m2_phase, params")
    print(f"  TFLite输出顺序: 需要根据上面的名称确认")

    # 检查是否输出顺序一致
    print("\n⚠ 关键检查点:")
    print("  1. TFLite的输出1应该对应m1_phase")
    print("  2. TFLite的输出2应该对应m2_phase")
    print("  3. 如果输出名称或顺序不同,ESP32代码需要相应调整!")

def main():
    print("=" * 70)
    print("模型输出调试工具")
    print("=" * 70)

    # 测试H5模型
    h5_outputs = test_h5_model()

    # 测试TFLite模型
    tflite_outputs, output_details = test_tflite_model()

    # 对比真实数据
    compare_real_data()

    print("\n" + "=" * 70)
    print("✓ 调试完成")
    print("=" * 70)

if __name__ == '__main__':
    main()
