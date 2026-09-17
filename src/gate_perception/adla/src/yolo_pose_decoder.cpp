#include "adla_gate_inference/yolo_pose_decoder.hpp"
#include <opencv2/dnn.hpp>
#include <algorithm>
#include <iostream>

std::vector<GateDetection> YoloPoseDecoder::decode(
    const float* raw_tensor,
    size_t num_floats,
    float conf_threshold,
    float corner_conf_thresh,
    float iou_threshold,
    float orig_width,
    float orig_height,
    float net_width,
    float net_height
) {
    std::vector<GateDetection> results;
    if (!raw_tensor || num_floats == 0) {
        return results;
    }

    int anchors = 8400;
    int channels = 17;
    bool is_channels_first = true;

    if (num_floats == 17 * 8400) {
        channels = 17;
        anchors = 8400;
    } else if (num_floats == 38 * 8400) {
        channels = 38;
        anchors = 8400;
    } else if (num_floats % 8400 == 0) {
        channels = static_cast<int>(num_floats / 8400);
        anchors = 8400;
    } else {
        std::cerr << "[YoloPoseDecoder] Unexpected tensor size: " << num_floats << std::endl;
        return results;
    }

    // Determine layout: channels-first vs anchors-first
    float max_cf = 0.0f;
    for (int a = 0; a < std::min(anchors, 100); ++a) {
        float s = raw_tensor[4 * anchors + a];
        if (s > max_cf) max_cf = s;
    }
    if (max_cf > 1.0f) {
        float max_af = 0.0f;
        for (int a = 0; a < std::min(anchors, 100); ++a) {
            float s = raw_tensor[a * channels + 4];
            if (s > max_af) max_af = s;
        }
        if (max_af <= 1.0f && max_af > 0.01f) {
            is_channels_first = false;
        }
    }

    auto get_val = [&](int c, int a) -> float {
        if (is_channels_first) {
            return raw_tensor[c * anchors + a];
        } else {
            return raw_tensor[a * channels + c];
        }
    };

    std::vector<cv::Rect2d> boxes;
    std::vector<float> scores;
    std::vector<std::vector<Keypoint>> raw_kpts_list;

    int num_kpts = (channels - 5) / 3;

    for (int a = 0; a < anchors; ++a) {
        float score = get_val(4, a);
        if (score < conf_threshold) {
            continue;
        }

        float cx = get_val(0, a);
        float cy = get_val(1, a);
        float w  = get_val(2, a);
        float h  = get_val(3, a);

        float x1 = cx - w * 0.5f;
        float y1 = cy - h * 0.5f;

        std::vector<Keypoint> kpts;
        kpts.reserve(num_kpts);
        for (int k = 0; k < num_kpts; ++k) {
            float kx = get_val(5 + k * 3 + 0, a);
            float ky = get_val(5 + k * 3 + 1, a);
            float kc = get_val(5 + k * 3 + 2, a);
            kpts.push_back({kx, ky, kc});
        }

        boxes.emplace_back(x1, y1, w, h);
        scores.push_back(score);
        raw_kpts_list.push_back(std::move(kpts));
    }

    if (boxes.empty()) {
        return results;
    }

    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, scores, conf_threshold, iou_threshold, indices);

    float x_scale = orig_width / net_width;
    float y_scale = orig_height / net_height;

    for (int idx : indices) {
        const auto& kpts = raw_kpts_list[idx];

        std::array<Keypoint, 4> corners;
        if (num_kpts == 4) {
            // gate_yolo_pose: 4 corners directly (TL, TR, BR, BL)
            corners[0] = kpts[0];
            corners[1] = kpts[1];
            corners[2] = kpts[2];
            corners[3] = kpts[3];
        } else if (num_kpts >= 10) {
            // legacy model: corners at indices 6, 7, 8, 9
            corners[0] = kpts[6];
            corners[1] = kpts[7];
            corners[2] = kpts[8];
            corners[3] = kpts[9];
        } else {
            for (int i = 0; i < std::min(4, num_kpts); ++i) {
                corners[i] = kpts[i];
            }
        }

        // Strict requirement: all 4 corners must meet corner_conf_thresh
        int valid_corners = 0;
        for (int i = 0; i < 4; ++i) {
            if (corners[i].conf >= corner_conf_thresh) {
                valid_corners++;
            }
        }
        if (valid_corners < 4) {
            continue;
        }

        GateDetection det;
        det.score = scores[idx];
        const auto& b = boxes[idx];
        det.cx = static_cast<float>((b.x + b.width * 0.5) * x_scale);
        det.cy = static_cast<float>((b.y + b.height * 0.5) * y_scale);
        det.w  = static_cast<float>(b.width * x_scale);
        det.h  = static_cast<float>(b.height * y_scale);

        for (int i = 0; i < 4; ++i) {
            det.corners[i].x = corners[i].x * x_scale;
            det.corners[i].y = corners[i].y * y_scale;
            det.corners[i].conf = corners[i].conf;
        }

        det.all_keypoints.reserve(kpts.size());
        for (const auto& kp : kpts) {
            det.all_keypoints.push_back({kp.x * x_scale, kp.y * y_scale, kp.conf});
        }

        results.push_back(std::move(det));
    }

    return results;
}
