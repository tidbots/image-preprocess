# image-preprocess

照明の変化に対応するための前処理ROSノード

## 機能

- **Gamma補正**（暗所/白飛び対策）
  - 暗い → γ > 1.0（明るくする）
  - 明るすぎ → γ < 1.0（暗くする）

- **CLAHE**（局所コントラスト正規化）
  - 通常のHistogram Equalizationより安全
  - 実機照明のムラに強い

- **プリセット機能** (v0.2.0)
  - 会場照明に応じた設定を簡単に切り替え

- **統計情報出力** (v0.2.0)
  - 輝度統計・処理時間をトピックで出力

- **画質評価** (v0.2.0)
  - ブラー検出、露出異常検出

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
git clone git@github.com:tidbots/image-preprocess.git
cd image-preprocess

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
  ├─ 画質評価 (ブラー検出、露出チェック)
  ├─ EMA平滑化
  ├─ 自動パラメータ調整（オプション）
  ├─ Gamma補正（LUT使用、ON/OFF可）
  └─ CLAHE（LAB色空間のLチャンネル、ON/OFF可）
        ↓
/camera/image_preprocessed (出力)
/camera/image_preprocess_stats (統計情報)
/camera/image_preprocess_debug (デバッグ用、オプション)
```

## ROSトピック

| トピック | 型 | 説明 |
|---------|------|------|
| `/usb_cam/image_raw` | sensor_msgs/Image | 入力画像（設定で変更可） |
| `/camera/image_preprocessed` | sensor_msgs/Image | 前処理済み画像 |
| `/camera/image_preprocess_stats` | PreprocessStats | 統計情報（有効時） |
| `/camera/image_preprocess_debug` | sensor_msgs/Image | デバッグオーバーレイ（有効時のみ） |

### PreprocessStats メッセージ

```
std_msgs/Header header
float32 mean_luma          # 平均輝度
float32 std_luma           # 輝度標準偏差
float32 sat_ratio          # 白飛び率
float32 dark_ratio         # 黒つぶれ率
float32 current_gamma      # 現在のgamma値
float32 current_clahe_clip # 現在のCLAHE clip値
float32 frame_time_ms      # 処理時間(ms)
float32 blur_score         # ブラースコア（高い=シャープ）
bool is_overexposed        # 露出オーバー
bool is_underexposed       # 露出アンダー
```

## プリセット機能

会場の照明に応じて、事前定義された設定を簡単に適用できます。

| プリセット | gamma | clahe_clip | auto_tune | 用途 |
|-----------|-------|------------|-----------|------|
| `dark_venue` | 1.4 | 3.0 | OFF | 暗い会場 |
| `bright_venue` | 0.85 | 1.5 | OFF | 明るすぎる会場 |
| `uniform` | 1.0 | 2.0 | OFF | 均一な照明 |
| `auto` | 1.1 | 2.5 | ON | 自動調整（デフォルト） |

使い方:
```xml
<param name="preset" value="dark_venue"/>
```

個別パラメータを指定すると、プリセットの値を上書きします。

## パラメータ一覧

### プリセット・機能ON/OFF

| パラメータ | デフォルト | 説明 |
|-----------|-----------|------|
| `preset` | `auto` | プリセット名 (dark_venue, bright_venue, uniform, auto, none) |
| `gamma_enable` | true | Gamma補正の有効化 |
| `clahe_enable` | true | CLAHEの有効化 |
| `stats_enable` | true | 統計トピック出力の有効化 |

### 基本パラメータ

| パラメータ | デフォルト | 範囲 | 説明 |
|-----------|-----------|------|------|
| `input_topic` | `/usb_cam/image_raw` | - | 入力トピック名 |
| `output_topic` | `/camera/image_preprocessed` | - | 出力トピック名 |
| `gamma` | (プリセット依存) | 0.70 - 1.60 | Gamma補正値 |
| `clahe_clip` | (プリセット依存) | 1.2 - 3.8 | CLAHEクリップリミット |
| `clahe_grid` | 8 | - | CLAHEグリッドサイズ |

### デバッグパラメータ

| パラメータ | デフォルト | 説明 |
|-----------|-----------|------|
| `debug_enable` | false | デバッグオーバーレイの有効化 |
| `debug_histogram` | false | ヒストグラム表示の有効化 |
| `debug_topic` | `/camera/image_preprocess_debug` | デバッグ出力トピック |

### 自動チューニングパラメータ

| パラメータ | デフォルト | 説明 |
|-----------|-----------|------|
| `auto_tune_enable` | (プリセット依存) | 自動チューニングの有効化 |
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

### 調整ステップ・範囲

| パラメータ | デフォルト | 説明 |
|-----------|-----------|------|
| `gamma_step` | 0.05 | Gamma調整ステップ |
| `gamma_step_saturated` | 0.08 | 白飛び時のGamma調整ステップ |
| `clahe_step` | 0.2 | CLAHE調整ステップ |
| `gamma_min` / `gamma_max` | 0.70 / 1.60 | Gamma範囲 |
| `clahe_min` / `clahe_max` | 1.2 / 3.8 | CLAHE範囲 |

### 性能監視・画質評価

| パラメータ | デフォルト | 説明 |
|-----------|-----------|------|
| `warn_frame_time_ms` | 30.0 | 処理時間警告閾値(ms) |
| `blur_threshold` | 100.0 | ブラー検出閾値（低い=ぼやけ） |
| `overexpose_ratio` | 0.3 | 露出オーバー判定閾値 |
| `underexpose_ratio` | 0.3 | 露出アンダー判定閾値 |

## 使い方

### 動作確認

```bash
# 起動
docker compose up

# 別ターミナルで可視化
rqt_image_view /camera/image_preprocessed

# 統計情報の確認
rostopic echo /camera/image_preprocess_stats
```

### プリセットの使用

```xml
<!-- 暗い会場用 -->
<param name="preset" value="dark_venue"/>

<!-- 明るい会場用 -->
<param name="preset" value="bright_venue"/>
```

### 処理の個別ON/OFF

```xml
<!-- Gamma補正のみ無効化（効果比較用） -->
<param name="gamma_enable" value="false"/>
```

### デバッグモード

```xml
<param name="debug_enable" value="true"/>
<param name="debug_histogram" value="true"/>
```

デバッグ画像には以下が表示されます：
- 現在のパラメータ値
- 輝度統計
- 処理時間
- 画質警告（OVEREXPOSED, UNDEREXPOSED, BLURRY）
- ヒストグラム（有効時）

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

### 照明パターン別・推奨プリセット

| 会場の状態 | プリセット | 備考 |
|-----------|-----------|------|
| 暗い（夕方・影が強い） | `dark_venue` | gamma ↑ が最優先 |
| 明るすぎ（白飛び） | `bright_venue` | gamma < 1.0 で抑制 |
| ムラあり（スポット照明） | `auto` | 自動調整に任せる |
| 均一・十分な照度 | `uniform` | 最小限の補正 |

### 鉄板設定（迷ったらこれ）

```xml
<param name="preset" value="auto"/>
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
image-preprocess/
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
        ├── msg/
        │   └── PreprocessStats.msg  # 統計メッセージ定義
        ├── scripts/
        │   └── preprocess_node.py   # メインノード
        └── launch/
            └── preprocess.launch    # 起動設定
```

## ライセンス

Apache 2.0
