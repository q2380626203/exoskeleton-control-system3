/**
 * AI推理模块 - TensorFlow Lite Micro
 *
 * 功能：
 * 1. 加载嵌入的量化模型
 * 2. 运行推理识别运动阶段
 * 3. 输出参数调整建议
 */

#include "ai_inference.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "AI";

// 声明嵌入的模型数据
extern const uint8_t model_data_start[] asm("_binary_motion_ai_model_quant_tflite_start");
extern const uint8_t model_data_end[]   asm("_binary_motion_ai_model_quant_tflite_end");

// Tensor Arena 大小（根据模型需求调整）
constexpr int kTensorArenaSize = 60 * 1024; // 60KB
alignas(16) static uint8_t tensor_arena[kTensorArenaSize];

// 全局变量
namespace {
    const tflite::Model* model = nullptr;
    tflite::MicroInterpreter* interpreter = nullptr;
    TfLiteTensor* input_tensor = nullptr;
    bool model_initialized = false;
}

/**
 * 初始化AI模型
 */
bool ai_model_init(void) {
    if (model_initialized) {
        ESP_LOGW(TAG, "Model already initialized");
        return true;
    }

    ESP_LOGI(TAG, "Initializing AI model...");

    // 计算模型大小
    const uint32_t model_size = model_data_end - model_data_start;
    ESP_LOGI(TAG, "Embedded model size: %lu bytes", model_size);

    // 加载模型
    model = tflite::GetModel(model_data_start);
    if (model->version() != TFLITE_SCHEMA_VERSION) {
        ESP_LOGE(TAG, "Model version mismatch! Expected %d, got %lu",
                 TFLITE_SCHEMA_VERSION, model->version());
        return false;
    }
    ESP_LOGI(TAG, "Model schema version: %lu", model->version());

    // 配置操作解析器
    // 使用 MicroMutableOpResolver 并注册模型需要的所有算子
    // 模板参数设为50以容纳足够多的算子（实际需要约36-40个）
    static tflite::MicroMutableOpResolver<50> resolver;

    // 注册卷积相关算子
    resolver.AddConv2D();
    resolver.AddDepthwiseConv2D();
    resolver.AddMaxPool2D();
    resolver.AddAveragePool2D();

    // 注册全连接层
    resolver.AddFullyConnected();

    // 注册归一化算子（注意：没有BatchNormalization，使用L2Normalization）
    resolver.AddL2Normalization();

    // 注册形状操作算子
    resolver.AddReshape();
    resolver.AddExpandDims();
    resolver.AddSqueeze();
    resolver.AddPad();
    resolver.AddPadV2();
    resolver.AddTranspose();
    resolver.AddPack();
    resolver.AddUnpack();

    // 注册激活函数
    resolver.AddSoftmax();
    resolver.AddRelu();
    resolver.AddRelu6();
    resolver.AddTanh();
    resolver.AddLogistic();  // Sigmoid
    resolver.AddLeakyRelu();
    resolver.AddPrelu();

    // 注册量化算子
    resolver.AddQuantize();
    resolver.AddDequantize();

    // 注册算术运算算子
    resolver.AddAdd();
    resolver.AddMul();
    resolver.AddSub();
    resolver.AddDiv();
    resolver.AddMean();
    resolver.AddSum();
    resolver.AddMaximum();
    resolver.AddMinimum();

    // 注册其他常用算子
    resolver.AddConcatenation();
    resolver.AddSplit();
    resolver.AddSlice();
    resolver.AddStridedSlice();
    resolver.AddGather();

    // 如果后续出现"Didn't find op for builtin opcode"错误，
    // 参考managed_components/espressif__esp-tflite-micro/tensorflow/lite/micro/micro_mutable_op_resolver.h
    // 添加相应的算子


    // 创建解释器
    static tflite::MicroInterpreter static_interpreter(
        model, resolver, tensor_arena, kTensorArenaSize
    );
    interpreter = &static_interpreter;

    // 分配张量
    TfLiteStatus allocate_status = interpreter->AllocateTensors();
    if (allocate_status != kTfLiteOk) {
        ESP_LOGE(TAG, "AllocateTensors() failed");
        return false;
    }

    // 获取输入张量
    input_tensor = interpreter->input(0);
    if (!input_tensor) {
        ESP_LOGE(TAG, "Failed to get input tensor");
        return false;
    }

    // 打印模型信息
    ESP_LOGI(TAG, "✓ Model loaded successfully!");
    ESP_LOGI(TAG, "  Input shape: [%d, %d, %d]",
             input_tensor->dims->data[0],
             input_tensor->dims->data[1],
             input_tensor->dims->data[2]);
    ESP_LOGI(TAG, "  Arena used: %d / %d bytes",
             interpreter->arena_used_bytes(), kTensorArenaSize);
    ESP_LOGI(TAG, "  Inputs count: %d", interpreter->inputs_size());
    ESP_LOGI(TAG, "  Outputs count: %d", interpreter->outputs_size());

    model_initialized = true;
    return true;
}

/**
 * 运行AI推理
 */
bool ai_run_inference(const float velocity_window[50], ai_inference_result_t *result) {
    if (!model_initialized || !interpreter || !input_tensor) {
        ESP_LOGE(TAG, "Model not initialized! Call ai_model_init() first");
        return false;
    }

    if (!velocity_window || !result) {
        ESP_LOGE(TAG, "Invalid parameters");
        return false;
    }

    // 填充输入数据到张量
    // 输入形状应该是 [1, 50, 1]
    for (int i = 0; i < 50; i++) {
        input_tensor->data.f[i] = velocity_window[i];
    }

    // 执行推理
    TfLiteStatus invoke_status = interpreter->Invoke();
    if (invoke_status != kTfLiteOk) {
        ESP_LOGE(TAG, "Invoke() failed!");
        return false;
    }

    // 获取输出张量
    // 注意：实际模型输出顺序与训练时定义不同！
    // 输出0: 阶段分类 [1, 3] - (静止, 抬腿, 压腿)
    // 输出1: 场景分类 [1, 2] - (平地, 爬楼)
    // 输出2: 参数调整 [1, 3] - (delta_torque, delta_kd, delta_scale)

    TfLiteTensor* output_phase = interpreter->output(0);  // 修正：输出0是阶段
    TfLiteTensor* output_scene = interpreter->output(1);  // 修正：输出1是场景
    TfLiteTensor* output_params = interpreter->output(2);

    if (!output_scene || !output_phase || !output_params) {
        ESP_LOGE(TAG, "Failed to get output tensors");
        return false;
    }

    // 解析场景（目前只有平地数据，所以场景固定为0）
    // 将来有爬楼数据后可以解注释
    result->scene = 0; // 平地
    result->scene_confidence = 1.0f;

    // 如果将来有多场景数据，使用下面的代码：
    /*
    if (output_scene->data.f[0] > output_scene->data.f[1]) {
        result->scene = 0; // 平地
        result->scene_confidence = output_scene->data.f[0];
    } else {
        result->scene = 1; // 爬楼
        result->scene_confidence = output_scene->data.f[1];
    }
    */

    // 解析阶段 (0: 静止, 1: 抬腿, 2: 压腿)
    result->phase = 0;
    result->phase_confidence = output_phase->data.f[0];
    for (int i = 1; i < 3; i++) {
        if (output_phase->data.f[i] > result->phase_confidence) {
            result->phase = i;
            result->phase_confidence = output_phase->data.f[i];
        }
    }

    // 解析参数调整
    result->delta_torque = output_params->data.f[0];
    result->delta_kd = output_params->data.f[1];
    result->delta_scale = output_params->data.f[2];

    // 打印推理结果（调试用）- 已禁用以减少日志输出
    // 如需查看推理详情，取消下面的注释
    /*
    const char* scene_names[] = {"平地", "爬楼"};
    const char* phase_names[] = {"静止", "抬腿", "压腿"};

    ESP_LOGI(TAG, "推理结果:");
    ESP_LOGI(TAG, "  场景: %s (%.1f%%)",
             scene_names[result->scene], result->scene_confidence * 100);
    ESP_LOGI(TAG, "  阶段: %s (%.1f%%)",
             phase_names[result->phase], result->phase_confidence * 100);
    ESP_LOGI(TAG, "  参数调整: Δtorque=%.3f, Δkd=%.3f, Δscale=%.3f",
             result->delta_torque, result->delta_kd, result->delta_scale);
    */

    return true;
}

/**
 * 获取模型信息
 */
void ai_get_model_info(uint32_t *model_size, uint32_t *arena_used) {
    if (model_size) {
        *model_size = model_data_end - model_data_start;
    }
    if (arena_used && interpreter) {
        *arena_used = interpreter->arena_used_bytes();
    }
}
