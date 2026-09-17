#include "adla_gate_inference/adla_engine.hpp"

#include <iostream>
#include <cstring>

#ifdef __aarch64__
extern "C" {
#include "adla_gate_inference/nnsdk/nn_sdk.h"
#include "adla_gate_inference/nnsdk/nn_util.h"
}
#endif

AdlaEngine::AdlaEngine() = default;

AdlaEngine::~AdlaEngine() {
    release();
}

bool AdlaEngine::init(const std::string& model_path, int input_width, int input_height, int input_channels) {
    release();

    model_path_ = model_path;
    input_width_ = input_width;
    input_height_ = input_height;
    input_channels_ = input_channels;

#ifdef __aarch64__
    aml_config config{};
    std::memset(&config, 0, sizeof(aml_config));
    config.nbgType = NN_ADLA_FILE;
    config.path = model_path_.c_str();
    config.modelType = ADLA_LOADABLE;
    config.typeSize = sizeof(aml_config);

    context_ = aml_module_create(&config);
    if (!context_) {
        std::cerr << "[AdlaEngine] ERROR: aml_module_create failed for: " << model_path_ << std::endl;
        initialized_ = false;
        return false;
    }

    std::cout << "[AdlaEngine] Successfully loaded ADLA model on NPU: " << model_path_ << std::endl;
    initialized_ = true;
    return true;
#else
    std::cout << "[AdlaEngine] Host simulation mode active (non-aarch64). Model path: " << model_path_ << std::endl;
    mock_output_buffer_.assign(17 * 8400, 0.0f);
    initialized_ = true;
    return true;
#endif
}

const float* AdlaEngine::run_inference_rgb(const uint8_t* rgb_data, size_t data_size, size_t& out_num_floats) {
    if (!initialized_) {
        out_num_floats = 0;
        return nullptr;
    }

#ifdef __aarch64__
    if (!context_) {
        out_num_floats = 0;
        return nullptr;
    }

    nn_input inData{};
    std::memset(&inData, 0, sizeof(nn_input));
    inData.input_index = 0;
    inData.input_type = RGB24_RAW_DATA;
    inData.size = static_cast<unsigned int>(data_size);
    inData.input = const_cast<uint8_t*>(rgb_data);

    int ret = aml_module_input_set(context_, &inData);
    if (ret != 0) {
        std::cerr << "[AdlaEngine] ERROR: aml_module_input_set failed with code: " << ret << std::endl;
        out_num_floats = 0;
        return nullptr;
    }

    aml_output_config_t outconfig{};
    std::memset(&outconfig, 0, sizeof(aml_output_config_t));
    outconfig.typeSize = sizeof(aml_output_config_t);
    outconfig.format = AML_OUTDATA_FLOAT32;
    outconfig.perfMode = AML_NO_PERF;

    auto* qout = reinterpret_cast<nn_output*>(aml_module_output_get(context_, outconfig));
    if (!qout || qout->num < 1 || !qout->out[0].buf) {
        std::cerr << "[AdlaEngine] ERROR: aml_module_output_get returned invalid output buffer" << std::endl;
        out_num_floats = 0;
        return nullptr;
    }

    out_num_floats = qout->out[0].size / sizeof(float);
    return reinterpret_cast<const float*>(qout->out[0].buf);
#else
    (void)rgb_data;
    (void)data_size;
    out_num_floats = mock_output_buffer_.size();
    return mock_output_buffer_.data();
#endif
}

void AdlaEngine::release() {
#ifdef __aarch64__
    if (context_) {
        aml_module_destroy(context_);
        context_ = nullptr;
    }
#endif
    initialized_ = false;
}
