#!/bin/bash

set -e

# download Torch Vision
echo "Downloading TorchVision v${DT_TORCH_VISION_VERSION}..."
TORCH_VISION_WHEEL_NAME="torchvision-${DT_TORCH_VISION_VERSION}-cp38-cp38-linux_aarch64.whl"
WHEEL_URL="https://duckietown-public-storage.s3.amazonaws.com/assets/python/wheels/${TORCH_VISION_WHEEL_NAME}"
wget -q "${WHEEL_URL}" -O "/tmp/${TORCH_VISION_WHEEL_NAME}"
# install Torch Vision
echo "Installing TorchVision v${DT_TORCH_VISION_VERSION}..."
pip3 install "/tmp/${TORCH_VISION_WHEEL_NAME}"
rm "/tmp/${TORCH_VISION_WHEEL_NAME}"

# clean
pip3 uninstall -y dataclasses
