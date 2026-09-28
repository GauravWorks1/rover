#!/bin/bash
# Download MobileNet SSD Caffe model for person detection
# Run this on the Raspberry Pi: bash models/download_models.sh

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "Downloading MobileNet SSD deploy prototxt..."
wget -O "$SCRIPT_DIR/MobileNetSSD_deploy.prototxt" \
  https://raw.githubusercontent.com/chuanqi305/MobileNet-SSD/master/deploy.prototxt

echo "Downloading MobileNet SSD caffemodel weights..."
wget -O "$SCRIPT_DIR/MobileNetSSD_deploy.caffemodel" \
  https://drive.google.com/uc?export=download\&id=0B3gersZ2cHIxRm5PMWRoTkdHdHc

# Alternative mirror if Google Drive fails:
if [ ! -f "$SCRIPT_DIR/MobileNetSSD_deploy.caffemodel" ] || [ $(stat -c%s "$SCRIPT_DIR/MobileNetSSD_deploy.caffemodel") -lt 1000000 ]; then
  echo "Primary download failed, trying alternative..."
  wget -O "$SCRIPT_DIR/MobileNetSSD_deploy.caffemodel" \
    https://github.com/chuanqi305/MobileNet-SSD/raw/master/mobilenet_iter_73000.caffemodel
fi

echo ""
echo "Model files downloaded to: $SCRIPT_DIR/"
ls -lh "$SCRIPT_DIR"/*.prototxt "$SCRIPT_DIR"/*.caffemodel 2>/dev/null
echo ""
echo "Done!"
