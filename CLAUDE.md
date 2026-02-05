# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS Noetic node for real-time image preprocessing to handle lighting variations in computer vision applications. Designed for YOLO object detection in RoboCup competitions. Applies gamma correction and CLAHE (Contrast Limited Adaptive Histogram Equalization) with optional auto-tuning based on brightness statistics.

## Build and Run Commands

```bash
# Build Docker image
docker compose build

# Run in Docker (starts ROS node)
docker compose up

# Local build (requires ROS Noetic)
cd /catkin_ws && catkin_make

# Launch node
roslaunch image_preprocess preprocess.launch

# View preprocessed output
rqt_image_view /camera/image_preprocessed

# View debug overlay (when debug_enable=true)
rqt_image_view /camera/image_preprocess_debug
```

## Architecture

```
/usb_cam/image_raw (input)
        ↓
[preprocess_node.py]
  ├─ Brightness statistics (mean, std, saturation ratio, dark ratio)
  ├─ Auto-tuning (EMA smoothing, rate-limited adjustments)
  ├─ Gamma correction (lookup table)
  └─ CLAHE on L-channel (LAB color space)
        ↓
/camera/image_preprocessed (output)
/camera/image_preprocess_debug (optional)
```

**Single-node architecture**: `src/image_preprocess/scripts/preprocess_node.py` is the only executable. All configuration via `launch/preprocess.launch`.

## Key Parameters

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `gamma` | 1.10 | Base gamma correction (< 1.0 darkens, > 1.0 brightens) |
| `clahe_clip` | 2.5 | CLAHE clip limit for local contrast |
| `clahe_grid` | 8 | CLAHE grid size |
| `auto_tune_enable` | true | Enable automatic parameter adjustment |

Auto-tuning bounds: gamma 0.70–1.60, clahe_clip 1.2–3.8

## Lighting Presets (from README)

- **Dark venue**: gamma 1.3–1.6, clahe_clip 3.0
- **Bright/washed out**: gamma 0.75–0.9, clahe_clip 1.5
- **Uneven lighting**: gamma auto, clahe_clip 2.5
- **Default/universal**: gamma 1.1, clahe_clip 2.5
