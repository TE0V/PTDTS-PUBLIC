# YOLO Model Setup Guide

## Issue: No Detections or Overlays in Tracking Mode

If you're seeing no detections or overlays in the video feed, the most common cause is **missing YOLO model files**.

## Quick Diagnosis

Check if model files exist:
```bash
ls -la models/
```

You should see either:
- **NCNN format (recommended)**: `yolov11n-UAV-finetune_ncnn_model.param` and `.bin`
- **PyTorch format**: `yolov11n-UAV-finetune.pt`

If the `models/` directory is empty, you need to add the model files.

## Solution: Add YOLO Model Files

### Option 1: NCNN Format (Recommended - 2-3x faster on Pi 5)

**If you already have NCNN model files:**
```bash
# Copy both .param and .bin files
cp /path/to/your/yolov11n-UAV-finetune_ncnn_model.param models/
cp /path/to/your/yolov11n-UAV-finetune_ncnn_model.bin models/

# Optional: Copy class names file (if available)
cp /path/to/your/yolov11n-UAV-finetune_ncnn_model.txt models/
```

**Verify the files:**
```bash
ls -la models/
# Should show:
# -rw-r--r-- 1 user user  XXXXX ... yolov11n-UAV-finetune_ncnn_model.bin
# -rw-r--r-- 1 user user  XXXXX ... yolov11n-UAV-finetune_ncnn_model.param
```

### Option 2: PyTorch Format

**If you have a PyTorch .pt model:**
```bash
# Copy the .pt file
cp /path/to/your/yolov11n-UAV-finetune.pt models/

# Update config.yaml to use PyTorch model:
# Edit config/config.yaml and change:
#   model_path: models/yolov11n-UAV-finetune.pt
```

### Option 3: Convert PyTorch to NCNN

If you have a PyTorch model and want better performance, convert it to NCNN:

```bash
# Install ultralytics
pip3 install ultralytics

# Export to NCNN format
yolo export model=models/yolov11n-UAV-finetune.pt format=ncnn

# This creates a directory like:
# yolov11n-UAV-finetune_ncnn_model/
#   ├── model.ncnn.param
#   └── model.ncnn.bin

# Move the files to models/ with correct names
mv yolov11n-UAV-finetune_ncnn_model/model.ncnn.param \
   models/yolov11n-UAV-finetune_ncnn_model.param

mv yolov11n-UAV-finetune_ncnn_model/model.ncnn.bin \
   models/yolov11n-UAV-finetune_ncnn_model.bin

# Clean up
rmdir yolov11n-UAV-finetune_ncnn_model
```

## Performance Notes

**NCNN Format:**
- Faster: ~15-30 FPS on Raspberry Pi 5
- Optimized for ARM64 CPUs
- Recommended for production use

**PyTorch Format:**
- Slower: ~5-10 FPS on Raspberry Pi 5
- Uses more memory
- Good for development/testing

## Using a Pre-trained Model

If you don't have a custom trained model, you can use a standard YOLOv11n model:

```bash
# Download standard YOLOv11n
pip3 install ultralytics
python3 -c "from ultralytics import YOLO; YOLO('yolo11n.pt')"

# This downloads the model to ~/.ultralytics/
# Copy it to your models directory
cp ~/.ultralytics/yolo11n.pt models/

# Update config.yaml:
# model_path: models/yolo11n.pt
```

**Note:** Standard YOLO models are trained on COCO dataset (80 classes including person, car, bird, etc.) but NOT specifically on drones. For best drone detection, use a fine-tuned model trained on UAV imagery.

## Training a Custom Model

If you need to train a custom drone detection model:

1. **Collect drone images** - Gather hundreds of images of drones from various angles, distances, and lighting conditions
2. **Annotate** - Use tools like [Roboflow](https://roboflow.com/) or [LabelImg](https://github.com/heartexlabs/labelImg)
3. **Train** - Use Ultralytics:
   ```bash
   yolo train data=drone_dataset.yaml model=yolo11n.pt epochs=100
   ```
4. **Export to NCNN** - Follow Option 3 above
5. **Test** - Verify detection works with your PTDTS system

## Troubleshooting

### Issue: "Failed to load NCNN model"

**Possible causes:**
- NCNN library not installed
- Model files corrupted
- Incorrect file paths

**Solution:**
```bash
# Install NCNN
pip3 install ncnn-python

# Verify model files are complete
ls -lh models/yolov11n-UAV-finetune_ncnn_model.*

# Check logs for specific error
tail -50 logs/ptdts.log
```

### Issue: "No module named 'ultralytics'"

**Solution:**
```bash
pip3 install ultralytics
```

### Issue: Low FPS / Slow Detection

**Solutions:**
1. **Use NCNN format** instead of PyTorch (2-3x faster)
2. **Lower camera resolution** in config.yaml (e.g., 1280x720 instead of 1920x1080)
3. **Reduce confidence threshold** - Lower values = faster but more false positives
4. **Close other applications** - Free up CPU/memory

### Issue: Too Many False Positives

**Solutions:**
1. **Increase confidence threshold** in config.yaml:
   ```yaml
   yolo:
     detection_confidence: 0.6  # Increase from 0.4
     tracking_confidence: 0.5   # Increase from 0.3
   ```
2. **Use a fine-tuned model** trained specifically on drones
3. **Adjust IOU threshold** for better NMS:
   ```yaml
   yolo:
     iou_threshold: 0.5  # Increase from 0.45
   ```

### Issue: Detections Work But No Overlays in Web UI

**Possible causes:**
1. Video streaming issue
2. Browser not receiving frames
3. JavaScript error

**Debug:**
```bash
# Check browser console for errors (F12 in most browsers)
# Check web app logs
tail -f logs/ptdts.log | grep -i "video\|frame\|detect"

# Verify detector is loaded
grep "Visual detector initialized" logs/ptdts.log
```

## Verifying Setup

After adding model files, restart the system and verify:

```bash
# Restart PTDTS
python main.py

# Check logs for successful model loading
tail -50 logs/ptdts.log | grep -i "detector\|model"

# You should see:
# "Visual detector initialized"
# "Loading NCNN model: param=models/..."
# "NCNN model loaded successfully"
```

Open the web UI and check:
- Video feed shows camera image
- When an object is in view, you should see yellow bounding boxes
- Telemetry panel shows "Detector FPS" and "Inference Time"

## Configuration Reference

Relevant config.yaml settings:

```yaml
yolo:
  # Model path (no extension for NCNN, .pt for PyTorch)
  model_path: models/yolov11n-UAV-finetune_ncnn_model

  device: cpu  # or 'cuda' if using GPU (PyTorch only)

  # Detection thresholds
  detection_confidence: 0.4  # Higher = fewer but more accurate detections
  tracking_confidence: 0.3   # Lower threshold for tracking (already confirmed)
  iou_threshold: 0.45        # Non-maximum suppression threshold

  # Inference settings
  max_det: 10       # Maximum detections per frame
  verbose: false    # Disable verbose YOLO logging
```

## Next Steps

After successfully setting up the model:
1. **Test detection** - Point camera at objects and verify bounding boxes appear
2. **Calibrate thresholds** - Adjust confidence values based on your environment
3. **Test tracking** - Switch to TRACKING state and verify smooth following
4. **Optimize performance** - Tune resolution and FPS for your needs

For more help, see:
- `README.md` - Full system setup
- `docs/ARCHITECTURE.md` - System design
- `docs/TROUBLESHOOTING.md` - Common issues
