# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS Noetic node for real-time image preprocessing to handle lighting variations in computer vision applications. Designed for YOLO object detection in RoboCup competitions. Applies gamma correction and CLAHE (Contrast Limited Adaptive Histogram Equalization) with optional auto-tuning based on brightness statistics.

**Version 0.2.0 features:**
- Preset configurations (dark_venue, bright_venue, uniform, auto)
- Statistics topic publishing (PreprocessStats message)
- Individual processing toggles (gamma_enable, clahe_enable)
- Image quality assessment (blur detection, exposure anomaly)
- Histogram overlay in debug mode
- Parameter validation with warnings

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

# View statistics
rostopic echo /camera/image_preprocess_stats

# View debug overlay (when debug_enable=true)
rqt_image_view /camera/image_preprocess_debug
```

## Architecture

```
/usb_cam/image_raw (input)
        ↓
[preprocess_node.py]
  ├─ Brightness statistics (mean, std, saturation ratio, dark ratio)
  ├─ Image quality assessment (blur score, exposure check)
  ├─ EMA smoothing
  ├─ Auto-tuning (rate-limited adjustments)
  ├─ Gamma correction (lookup table) - toggleable
  └─ CLAHE on L-channel (LAB color space) - toggleable
        ↓
/camera/image_preprocessed (output)
/camera/image_preprocess_stats (statistics)
/camera/image_preprocess_debug (optional)
```

**Single-node architecture**: `src/image_preprocess/scripts/preprocess_node.py` is the only executable. All configuration via `launch/preprocess.launch`.

## Key Files

| File | Purpose |
|------|---------|
| `src/image_preprocess/scripts/preprocess_node.py` | Main node implementation |
| `src/image_preprocess/launch/preprocess.launch` | ROS parameters |
| `src/image_preprocess/msg/PreprocessStats.msg` | Statistics message definition |
| `src/image_preprocess/CMakeLists.txt` | Build configuration |
| `src/image_preprocess/package.xml` | Package dependencies |

## Presets

| Preset | gamma | clahe_clip | auto_tune | Use case |
|--------|-------|------------|-----------|----------|
| `dark_venue` | 1.4 | 3.0 | OFF | Dark venues |
| `bright_venue` | 0.85 | 1.5 | OFF | Bright venues |
| `uniform` | 1.0 | 2.0 | OFF | Uniform lighting |
| `auto` | 1.1 | 2.5 | ON | Auto-adjustment (default) |

## Key Parameters

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `preset` | auto | Preset configuration |
| `gamma` | (preset) | Base gamma correction (< 1.0 darkens, > 1.0 brightens) |
| `clahe_clip` | (preset) | CLAHE clip limit for local contrast |
| `gamma_enable` | true | Enable/disable gamma correction |
| `clahe_enable` | true | Enable/disable CLAHE |
| `stats_enable` | true | Enable statistics publishing |
| `auto_tune_enable` | (preset) | Enable automatic parameter adjustment |
| `debug_enable` | false | Enable debug overlay |
| `debug_histogram` | false | Enable histogram in debug |

Auto-tuning bounds: gamma 0.70–1.60, clahe_clip 1.2–3.8

## Code Patterns

- **Preset loading**: `_load_preset()` method loads defaults from `PRESETS` dict
- **Parameter validation**: `_validate_params()` checks ranges and logs warnings
- **Statistics computation**: `_compute_stats()` returns `BrightnessStats` dataclass
- **Quality assessment**: `_compute_blur_score()` uses Laplacian variance
- **Stats publishing**: `_publish_stats()` sends `PreprocessStats` message
