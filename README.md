# image_preprocess

照明の変化に対応するための前処理ROSノード

## 機能

- **Gamma補正**（暗所/白飛び対策）
  - 暗い → γ > 1.0（明るくする）
  - 明るすぎ → γ < 1.0（暗くする）

- **CLAHE**（局所コントラスト正規化）
  - 通常のHistogram Equalizationより安全
  - 実機照明のムラに強い

## 設計方針

- CPUのみ・軽量
- 照明が暗すぎ / 明るすぎ の両方に対応
- パラメータはROS paramで調整可能
- YOLO向け（白飛び・黒つぶれを防止）
- Docker内でそのまま動作

## 前提条件

- Ubuntu 20.04 / 22.04
- Docker / Docker Compose v2
- ROS Noetic
- USBカメラ or `/camera/image_raw` トピックが発行されていること

## クイックスタート

```bash
# リポジトリをクローン
git clone git@github.com:tidbots/image_preprocess.git
cd image_preprocess

# Dockerイメージをビルド
docker compose build

# 起動
docker compose up
```

## アーキテクチャ

```
/usb_cam/image_raw (入力)
        ↓
[ preprocess_node.py ]
  ├─ 輝度統計計算 (mean / std / sat_ratio / dark_ratio)
  ├─ EMA平滑化
  ├─ 自動パラメータ調整（オプション）
  ├─ Gamma補正（LUT使用）
  └─ CLAHE（LAB色空間のLチャンネル）
        ↓
/camera/image_preprocessed (出力)
/camera/image_preprocess_debug (デバッグ用、オプション)
```

## ROSトピック

| トピック | 型 | 説明 |
|---------|------|------|
| `/usb_cam/image_raw` | sensor_msgs/Image | 入力画像（設定で変更可） |
| `/camera/image_preprocessed` | sensor_msgs/Image | 前処理済み画像 |
| `/camera/image_preprocess_debug` | sensor_msgs/Image | デバッグオーバーレイ（有効時のみ） |

## パラメータ一覧

### 基本パラメータ

| パラメータ | デフォルト | 範囲 | 説明 |
|-----------|-----------|------|------|
| `input_topic` | `/usb_cam/image_raw` | - | 入力トピック名 |
| `output_topic` | `/camera/image_preprocessed` | - | 出力トピック名 |
| `gamma` | 1.10 | 0.70 - 1.60 | Gamma補正値 |
| `clahe_clip` | 2.5 | 1.2 - 3.8 | CLAHEクリップリミット |
| `clahe_grid` | 8 | - | CLAHEグリッドサイズ |

### デバッグパラメータ

| パラメータ | デフォルト | 説明 |
|-----------|-----------|------|
| `debug_enable` | false | デバッグオーバーレイの有効化 |
| `debug_topic` | `/camera/image_preprocess_debug` | デバッグ出力トピック |

### 自動チューニングパラメータ

| パラメータ | デフォルト | 説明 |
|-----------|-----------|------|
| `auto_tune_enable` | true | 自動チューニングの有効化 |
| `ema_alpha` | 0.15 | EMA平滑化係数 |
| `auto_tune_update_every_n` | 8 | 更新間隔（フレーム数） |
| `auto_tune_min_update_interval` | 0.25 | 最小更新間隔（秒） |

### 検出閾値

| パラメータ | デフォルト | 説明 |
|-----------|-----------|------|
| `dark_mean_thr` | 90.0 | 暗い判定の平均輝度閾値 |
| `bright_mean_thr` | 170.0 | 明るい判定の平均輝度閾値 |
| `low_contrast_std_thr` | 35.0 | 低コントラスト判定の標準偏差閾値 |
| `sat_thr` | 245 | 白飛びピクセル閾値 |
| `dark_thr` | 10 | 黒つぶれピクセル閾値 |
| `sat_ratio_thr` | 0.12 | 白飛び率閾値 |
| `dark_ratio_thr` | 0.12 | 黒つぶれ率閾値 |

### 調整ステップ

| パラメータ | デフォルト | 説明 |
|-----------|-----------|------|
| `gamma_step` | 0.05 | Gamma調整ステップ |
| `gamma_step_saturated` | 0.08 | 白飛び時のGamma調整ステップ |
| `clahe_step` | 0.2 | CLAHE調整ステップ |

### 調整範囲

| パラメータ | デフォルト | 説明 |
|-----------|-----------|------|
| `gamma_min` | 0.70 | Gamma最小値 |
| `gamma_max` | 1.60 | Gamma最大値 |
| `clahe_min` | 1.2 | CLAHE最小値 |
| `clahe_max` | 3.8 | CLAHE最大値 |

## 使い方

### 動作確認

```bash
# 起動
docker compose up

# 別ターミナルで可視化
rqt_image_view /camera/image_preprocessed
```

### デバッグモード

`preprocess.launch` の `debug_enable` を `true` に変更：

```xml
<param name="debug_enable" value="true"/>
```

デバッグ画像には現在のパラメータと輝度統計がオーバーレイ表示されます。

### Docker環境変数

`compose.yaml` で ROS Master の接続先を変更できます：

```yaml
environment:
  - ROS_MASTER_URI=http://192.168.1.100:11311  # 別マシンのROS Master
  - ROS_IP=192.168.1.50                         # 自身のIP
```

## パラメータチューニング指針

### チューニングの基本方針

1. まず前処理（画像の見え）を安定させる
2. 次に YOLO の conf / tile を調整
3. 最後に Depth ROI を詰める

**いきなり YOLO 側を触らないのがコツ**

### 照明パターン別・推奨設定

#### A. 暗い会場（夕方・照度不足・影が強い）

症状：全体が暗い、小物が背景に溶ける、confidenceが低い

```
gamma: 1.3 〜 1.6
clahe_clip: 3.0
clahe_grid: 8
```

#### B. 明るすぎる会場（白飛び・直射照明）

症状：白い床・テーブルが飛ぶ、物体輪郭が消える

```
gamma: 0.75 〜 0.9
clahe_clip: 1.5
clahe_grid: 8
```

#### C. ムラのある照明（スポットライト・影あり）

症状：場所によって明るさが違う、認識が不安定

```
gamma: auto（auto_tune_enable=true）
clahe_clip: 2.5
clahe_grid: 8
```

#### D. 理想的な会場（均一・十分な照度）

```
gamma: 1.0
clahe_clip: 2.0
clahe_grid: 8
```

### 鉄板プリセット（迷ったらこれ）

```
gamma: 1.1
clahe_clip: 2.5
```

## 自動チューニングの仕組み

### 監視指標

各フレームで以下を計算（EMA平滑化）：

| 指標 | 意味 |
|------|------|
| mean_luma | 全体の明るさ |
| std_luma | 明るさのばらつき |
| sat_ratio | 白飛び率（>245） |
| dark_ratio | 黒つぶれ率（<10） |

### 照明状態の分類

| 状態 | 条件 |
|------|------|
| DARK | mean < 90 |
| BRIGHT | mean > 170 |
| SATURATED | sat_ratio > 0.12 |
| LOW_CONTRAST | std < 35 |
| NORMAL | 上記以外 |

### 自動調整ルール

- 暗い → gamma ↑
- 明るい → gamma ↓
- 白飛び → gamma ↓ + clahe_clip ↓
- コントラスト低 → clahe_clip ↑

**1フレームで大きく変えない（±0.05）**

## プロジェクト構成

```
image_preprocess/
├── README.md
├── README-en.md          # English version
├── LICENSE               # Apache 2.0
├── compose.yaml          # Docker Compose設定
├── docker/
│   ├── Dockerfile        # ROS Noetic + OpenCV
│   └── entrypoint.sh     # ROS環境セットアップ
└── src/
    └── image_preprocess/
        ├── package.xml
        ├── CMakeLists.txt
        ├── scripts/
        │   └── preprocess_node.py  # メインノード
        └── launch/
            └── preprocess.launch   # 起動設定
```

## ライセンス

Apache 2.0
