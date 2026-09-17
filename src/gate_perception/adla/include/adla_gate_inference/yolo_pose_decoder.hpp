#ifndef ADLA_GATE_INFERENCE_YOLO_POSE_DECODER_HPP_
#define ADLA_GATE_INFERENCE_YOLO_POSE_DECODER_HPP_

#include <vector>
#include <array>
#include <opencv2/core.hpp>

struct Keypoint {
    float x{0.0f};
    float y{0.0f};
    float conf{0.0f};
};

struct GateDetection {
    float cx{0.0f};
    float cy{0.0f};
    float w{0.0f};
    float h{0.0f};
    float score{0.0f};

    // 4 physical corners in image space: 0=TL, 1=TR, 2=BR, 3=BL
    std::array<Keypoint, 4> corners;

    // All keypoints scaled to original image dimensions
    std::vector<Keypoint> all_keypoints;
};

class YoloPoseDecoder {
public:
    YoloPoseDecoder() = default;

    /**
     * @brief Decodes the raw output tensor from the YOLO pose NPU inference.
     * @param raw_tensor Pointer to the float output array.
     * @param num_floats Total count of float elements in the buffer.
     * @param conf_threshold Minimum confidence score for candidate gate detection.
     * @param corner_conf_thresh Minimum confidence for individual corners (TL, TR, BR, BL).
     * @param iou_threshold IoU threshold for Non-Maximum Suppression.
     * @param orig_width Original camera frame width.
     * @param orig_height Original camera frame height.
     * @param net_width Neural network input width (default 640).
     * @param net_height Neural network input height (default 640).
     * @return Vector of valid detected gates.
     */
    std::vector<GateDetection> decode(
        const float* raw_tensor,
        size_t num_floats,
        float conf_threshold,
        float corner_conf_thresh,
        float iou_threshold,
        float orig_width,
        float orig_height,
        float net_width = 640.0f,
        float net_height = 640.0f
    );
};

#endif // ADLA_GATE_INFERENCE_YOLO_POSE_DECODER_HPP_
