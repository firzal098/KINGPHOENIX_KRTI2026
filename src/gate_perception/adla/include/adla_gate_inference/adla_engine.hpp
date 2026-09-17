#ifndef ADLA_GATE_INFERENCE_ADLA_ENGINE_HPP_
#define ADLA_GATE_INFERENCE_ADLA_ENGINE_HPP_

#include <string>
#include <vector>
#include <memory>
#include <cstdint>
#include <iostream>

class AdlaEngine {
public:
    AdlaEngine();
    ~AdlaEngine();

    // Disable copy
    AdlaEngine(const AdlaEngine&) = delete;
    AdlaEngine& operator=(const AdlaEngine&) = delete;

    /**
     * @brief Loads the .adla model and initializes the NPU context.
     * @param model_path Path to the .adla compiled model file.
     * @param input_width Expected input width (default 640).
     * @param input_height Expected input height (default 640).
     * @param input_channels Expected input channels (default 3).
     * @return true if successfully initialized, false otherwise.
     */
    bool init(const std::string& model_path, int input_width = 640, int input_height = 640, int input_channels = 3);

    /**
     * @brief Executes inference on an interleaved RGB24 image buffer (640x640x3 bytes).
     * @param rgb_data Pointer to the raw RGB pixel data.
     * @param data_size Size of the buffer in bytes (must equal input_width * input_height * 3).
     * @param out_num_floats Output parameter receiving the count of returned float elements.
     * @return Pointer to raw float output tensor from NPU, or nullptr on failure.
     */
    const float* run_inference_rgb(const uint8_t* rgb_data, size_t data_size, size_t& out_num_floats);

    /**
     * @brief Releases the NPU context.
     */
    void release();

    bool is_initialized() const { return initialized_; }
    const std::string& get_model_path() const { return model_path_; }

    int get_input_width() const { return input_width_; }
    int get_input_height() const { return input_height_; }

private:
    void* context_{nullptr};
    bool initialized_{false};
    std::string model_path_{};
    int input_width_{640};
    int input_height_{640};
    int input_channels_{3};

#ifndef __aarch64__
    // Simulation buffer for non-ARM host compilation
    std::vector<float> mock_output_buffer_;
#endif
};

#endif // ADLA_GATE_INFERENCE_ADLA_ENGINE_HPP_
