# image-preprocess

A preprocessing ROS node for adapting to lighting changes

## Features

- **Gamma Correction** (for dark/overexposed conditions)
  - Dark scenes → γ > 1.0 (brightens)
  - Too bright → γ < 1.0 (darkens)

- **CLAHE** (Local contrast normalization)
  - Safer than standard Histogram Equalization
  - Robust against uneven lighting in real environments

- **Preset Configurations** (v0.2.0)
  - Easily switch settings based on venue lighting

- **Statistics Publishing** (v0.2.0)
  - Publish brightness stats and processing time to a topic

- **Image Quality Assessment** (v0.2.0)
  - Blur detection, exposure anomaly detection

## Design Philosophy

- CPU-only, lightweight
- Handles both too dark and too bright lighting
- Parameters adjustable via ROS params
- Designed for YOLO (prevents overexposure and crushed blacks)
- Runs directly in Docker

## Prerequisites

- Ubuntu 20.04 / 22.04
- Docker / Docker Compose v2
- ROS Noetic
- USB camera or `/camera/image_raw` topic being published

## Quick Start

```bash
# Clone repository
git clone git@github.com:tidbots/image-preprocess.git
cd image-preprocess

# Build Docker image
docker compose build

# Run
docker compose up
```

## Architecture

```
/usb_cam/image_raw (input)
        ↓
[ preprocess_node.py ]
  ├─ Brightness statistics (mean / std / sat_ratio / dark_ratio)
  ├─ Image quality assessment (blur, exposure)
  ├─ EMA smoothing
  ├─ Auto parameter adjustment (optional)
  ├─ Gamma correction (LUT-based, toggleable)
  └─ CLAHE (on L channel in LAB, toggleable)
        ↓
/camera/image_preprocessed (output)
/camera/image_preprocess_stats (statistics)
/camera/image_preprocess_debug (debug, optional)
```

## ROS Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/usb_cam/image_raw` | sensor_msgs/Image | Input image (configurable) |
| `/camera/image_preprocessed` | sensor_msgs/Image | Preprocessed image |
| `/camera/image_preprocess_stats` | PreprocessStats | Statistics (when enabled) |
| `/camera/image_preprocess_debug` | sensor_msgs/Image | Debug overlay (when enabled) |

### PreprocessStats Message

```
std_msgs/Header header
float32 mean_luma          # Mean luminance
float32 std_luma           # Luminance std deviation
float32 sat_ratio          # Overexposure ratio
float32 dark_ratio         # Underexposure ratio
float32 current_gamma      # Current gamma value
float32 current_clahe_clip # Current CLAHE clip value
float32 frame_time_ms      # Processing time (ms)
float32 blur_score         # Blur score (higher = sharper)
bool is_overexposed        # Overexposure flag
bool is_underexposed       # Underexposure flag
```

## Preset Feature

Easily apply predefined settings based on venue lighting.

| Preset | gamma | clahe_clip | auto_tune | Use case |
|--------|-------|------------|-----------|----------|
| `dark_venue` | 1.4 | 3.0 | OFF | Dark venues |
| `bright_venue` | 0.85 | 1.5 | OFF | Too bright venues |
| `uniform` | 1.0 | 2.0 | OFF | Uniform lighting |
| `auto` | 1.1 | 2.5 | ON | Auto-adjustment (default) |

Usage:
```xml
<param name="preset" value="dark_venue"/>
```

Individual parameters override preset values.

## Parameter Reference

### Preset & Feature Toggles

| Parameter | Default | Description |
|-----------|---------|-------------|
| `preset` | `auto` | Preset name (dark_venue, bright_venue, uniform, auto, none) |
| `gamma_enable` | true | Enable gamma correction |
| `clahe_enable` | true | Enable CLAHE |
| `stats_enable` | true | Enable stats topic publishing |

### Basic Parameters

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `input_topic` | `/usb_cam/image_raw` | - | Input topic name |
| `output_topic` | `/camera/image_preprocessed` | - | Output topic name |
| `gamma` | (preset-dependent) | 0.70 - 1.60 | Gamma correction value |
| `clahe_clip` | (preset-dependent) | 1.2 - 3.8 | CLAHE clip limit |
| `clahe_grid` | 8 | - | CLAHE grid size |

### Debug Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `debug_enable` | false | Enable debug overlay |
| `debug_histogram` | false | Enable histogram display |
| `debug_topic` | `/camera/image_preprocess_debug` | Debug output topic |

### Auto-tuning Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `auto_tune_enable` | (preset-dependent) | Enable auto-tuning |
| `ema_alpha` | 0.15 | EMA smoothing factor |
| `auto_tune_update_every_n` | 8 | Update interval (frames) |
| `auto_tune_min_update_interval` | 0.25 | Minimum update interval (seconds) |

### Detection Thresholds

| Parameter | Default | Description |
|-----------|---------|-------------|
| `dark_mean_thr` | 90.0 | Mean luminance threshold for dark detection |
| `bright_mean_thr` | 170.0 | Mean luminance threshold for bright detection |
| `low_contrast_std_thr` | 35.0 | Std deviation threshold for low contrast |
| `sat_thr` | 245 | Pixel threshold for overexposure |
| `dark_thr` | 10 | Pixel threshold for crushed blacks |
| `sat_ratio_thr` | 0.12 | Overexposure ratio threshold |
| `dark_ratio_thr` | 0.12 | Crushed blacks ratio threshold |

### Adjustment Steps & Bounds

| Parameter | Default | Description |
|-----------|---------|-------------|
| `gamma_step` | 0.05 | Gamma adjustment step |
| `gamma_step_saturated` | 0.08 | Gamma step when saturated |
| `clahe_step` | 0.2 | CLAHE adjustment step |
| `gamma_min` / `gamma_max` | 0.70 / 1.60 | Gamma bounds |
| `clahe_min` / `clahe_max` | 1.2 / 3.8 | CLAHE bounds |

### Performance Monitoring & Quality Assessment

| Parameter | Default | Description |
|-----------|---------|-------------|
| `warn_frame_time_ms` | 30.0 | Processing time warning threshold (ms) |
| `blur_threshold` | 100.0 | Blur detection threshold (lower = blurry) |
| `overexpose_ratio` | 0.3 | Overexposure detection threshold |
| `underexpose_ratio` | 0.3 | Underexposure detection threshold |

## Usage

### Verification

```bash
# Run
docker compose up

# Visualize in another terminal
rqt_image_view /camera/image_preprocessed

# Check statistics
rostopic echo /camera/image_preprocess_stats
```

### Using Presets

```xml
<!-- For dark venues -->
<param name="preset" value="dark_venue"/>

<!-- For bright venues -->
<param name="preset" value="bright_venue"/>
```

### Individual Processing Toggle

```xml
<!-- Disable gamma only (for comparison) -->
<param name="gamma_enable" value="false"/>
```

### Debug Mode

```xml
<param name="debug_enable" value="true"/>
<param name="debug_histogram" value="true"/>
```

Debug image shows:
- Current parameter values
- Brightness statistics
- Processing time
- Quality warnings (OVEREXPOSED, UNDEREXPOSED, BLURRY)
- Histogram (when enabled)

### Docker Environment Variables

Configure ROS Master connection in `compose.yaml`:

```yaml
environment:
  - ROS_MASTER_URI=http://192.168.1.100:11311  # Remote ROS Master
  - ROS_IP=192.168.1.50                         # Your own IP
```

## Parameter Tuning Guidelines

### Basic Tuning Principles

1. First stabilize preprocessing (image appearance)
2. Then adjust YOLO conf / tile
3. Finally fine-tune Depth ROI

**Key: Don't touch YOLO settings first**

### Recommended Presets by Lighting

| Venue Condition | Preset | Notes |
|-----------------|--------|-------|
| Dark (evening, strong shadows) | `dark_venue` | Increasing gamma is priority |
| Too bright (washed out) | `bright_venue` | gamma < 1.0 suppresses |
| Uneven (spotlights) | `auto` | Let auto-tuning handle it |
| Uniform, sufficient | `uniform` | Minimal correction |

### Universal Preset (when in doubt)

```xml
<param name="preset" value="auto"/>
```

## Auto-tuning Mechanism

### Monitoring Metrics

Calculated for each frame (with EMA smoothing):

| Metric | Meaning |
|--------|---------|
| mean_luma | Overall brightness |
| std_luma | Brightness variance |
| sat_ratio | Overexposure rate (>245) |
| dark_ratio | Crushed blacks rate (<10) |

### Lighting State Classification

| State | Condition |
|-------|-----------|
| DARK | mean < 90 |
| BRIGHT | mean > 170 |
| SATURATED | sat_ratio > 0.12 |
| LOW_CONTRAST | std < 35 |
| NORMAL | None of the above |

### Auto-adjustment Rules

- Dark → gamma ↑
- Bright → gamma ↓
- Overexposed → gamma ↓ + clahe_clip ↓
- Low contrast → clahe_clip ↑

**Small changes per frame only (±0.05)**

## Project Structure

```
image-preprocess/
├── README.md             # Japanese version
├── README-en.md          # English version
├── LICENSE               # Apache 2.0
├── compose.yaml          # Docker Compose config
├── docker/
│   ├── Dockerfile        # ROS Noetic + OpenCV
│   └── entrypoint.sh     # ROS environment setup
└── src/
    └── image_preprocess/
        ├── package.xml
        ├── CMakeLists.txt
        ├── msg/
        │   └── PreprocessStats.msg  # Statistics message definition
        ├── scripts/
        │   └── preprocess_node.py   # Main node
        └── launch/
            └── preprocess.launch    # Launch configuration
```

## License

Apache 2.0
