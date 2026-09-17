#!/usr/bin/env bash
# Quick runner script for ADLA offline gate inference test

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(dirname "$SCRIPT_DIR")"

echo "Running ADLA Offline Gate Inference Verification..."
ros2 run adla_gate_inference test_offline_inference \
    --ros-args \
    -p input_dir:="${PACKAGE_DIR}/tests/test_images" \
    -p output_dir:="${PACKAGE_DIR}/tests/output_images" \
    -p model_path:="${PACKAGE_DIR}/resource/gate_yolo_pose_int8.adla"

echo ""
echo "Done! Check annotated images in: ${PACKAGE_DIR}/tests/output_images"
