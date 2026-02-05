# image_preprocess

A preprocessing ROS node for adapting to lighting changes

## Features

- **Gamma Correction** (for dark/overexposed conditions)
  - Dark scenes → γ > 1.0 (brightens)
  - Too bright → γ < 1.0 (darkens)

- **CLAHE** (Local contrast normalization)
  - Safer than standard Histogram Equalization
  - Robust against uneven lighting in real environments

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
git clone git@github.com:tidbots/image_preprocess.git
cd image_preprocess

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
  ├─ EMA smoothing
  ├─ Auto parameter adjustment (optional)
  ├─ Gamma correction (LUT-based)
  └─ CLAHE (on L channel in LAB color space)
        ↓
/camera/image_preprocessed (output)
/camera/image_preprocess_debug (debug, optional)
```

## ROS Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/usb_cam/image_raw` | sensor_msgs/Image | Input image (configurable) |
| `/camera/image_preprocessed` | sensor_msgs/Image | Preprocessed image |
| `/camera/image_preprocess_debug` | sensor_msgs/Image | Debug overlay (when enabled) |

## Parameter Reference

### Basic Parameters

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `input_topic` | `/usb_cam/image_raw` | - | Input topic name |
| `output_topic` | `/camera/image_preprocessed` | - | Output topic name |
| `gamma` | 1.10 | 0.70 - 1.60 | Gamma correction value |
| `clahe_clip` | 2.5 | 1.2 - 3.8 | CLAHE clip limit |
| `clahe_grid` | 8 | - | CLAHE grid size |

### Debug Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `debug_enable` | false | Enable debug overlay |
| `debug_topic` | `/camera/image_preprocess_debug` | Debug output topic |

### Auto-tuning Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `auto_tune_enable` | true | Enable auto-tuning |
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

### Adjustment Steps

| Parameter | Default | Description |
|-----------|---------|-------------|
| `gamma_step` | 0.05 | Gamma adjustment step |
| `gamma_step_saturated` | 0.08 | Gamma step when saturated |
| `clahe_step` | 0.2 | CLAHE adjustment step |

### Adjustment Bounds

| Parameter | Default | Description |
|-----------|---------|-------------|
| `gamma_min` | 0.70 | Minimum gamma value |
| `gamma_max` | 1.60 | Maximum gamma value |
| `clahe_min` | 1.2 | Minimum CLAHE value |
| `clahe_max` | 3.8 | Maximum CLAHE value |

## Usage

### Verification

```bash
# Run
docker compose up

# Visualize in another terminal
rqt_image_view /camera/image_preprocessed
```

### Debug Mode

Set `debug_enable` to `true` in `preprocess.launch`:

```xml
<param name="debug_enable" value="true"/>
```

The debug image shows current parameters and brightness statistics as an overlay.

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

### Recommended Settings by Lighting Pattern

#### A. Dark Venue (evening, insufficient light, strong shadows)

Symptoms: Overall dark, small objects blend into background, low confidence

```
gamma: 1.3 - 1.6
clahe_clip: 3.0
clahe_grid: 8
```

#### B. Too Bright Venue (overexposure, direct lighting)

Symptoms: White floors/tables washed out, object edges disappear

```
gamma: 0.75 - 0.9
clahe_clip: 1.5
clahe_grid: 8
```

#### C. Uneven Lighting (spotlights, shadows)

Symptoms: Brightness varies by location, inconsistent recognition

```
gamma: auto (auto_tune_enable=true)
clahe_clip: 2.5
clahe_grid: 8
```

#### D. Ideal Venue (uniform, sufficient lighting)

```
gamma: 1.0
clahe_clip: 2.0
clahe_grid: 8
```

### Universal Preset (when in doubt)

```
gamma: 1.1
clahe_clip: 2.5
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
image_preprocess/
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
        ├── scripts/
        │   └── preprocess_node.py  # Main node
        └── launch/
            └── preprocess.launch   # Launch configuration
```

## License

Apache 2.0
