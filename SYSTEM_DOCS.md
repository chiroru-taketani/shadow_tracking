# Shadow Tracking System ドキュメント

## 1. システム概要
本システムは、LiDARセンサーを用いて空間内の人の位置をリアルタイムに検出し、その位置に応じて仮想的な「影」を生成・投影するインタラクティブシステムです。また、計算された光源の位置情報はLEDパネルに送信され、物理的な光の表現として可視化されます。

### 主な機能
- **人流検知**: LiDARセンサーによる高精度な位置追跡
- **影生成**: OpenGLを用いたリアルタイムな影のレンダリング
- **光源連動**: 人の動きに連動した光源位置の制御
- **LED連携**: 光源位置をLEDマトリクスパネルに同期表示

## 2. システム構成
システムは主に以下の3つのソフトウェアコンポーネントで構成されています。

1.  **LiDAR Processing (`LIDAR1b`)**
    *   担当: センサー制御、データ取得、物体検出
    *   言語: C++ (OpenCV, GLUT, URG Library)
2.  **Shadow Projection (`Shadow`)**
    *   担当: 影の描画、光源計算、シリアル通信
    *   言語: C++ (OpenGL, OpenCV, GLUT)
3.  **LED Controller (`Arduino`)**
    *   担当: LEDパネルの描画制御
    *   言語: Arduino (C++)

## 3. コンポーネント詳細

### 3.1 LiDAR Processing (`LIDAR1b/main_cv.cpp`)
URGセンサーから距離データを取得し、画像処理技術を用いて人の位置（足元）を特定します。

*   **処理フロー**:
    1.  **データ取得**: URGドライバ経由で距離データを取得。
    2.  **座標変換**: 極座標系（距離・角度）を直交座標系（X, Y）に変換し、OpenGLで描画。
    3.  **画像化**: OpenGLの描画結果をOpenCVの画像データ(`cv::Mat`)として取得。
    4.  **前処理**:
        *   2値化
        *   モルフォロジー演算（膨張・収縮）によるノイズ除去と領域結合。
    5.  **物体検出**: `cv::findContours`で輪郭を抽出し、一定面積以上の領域を「人」として認識。
    6.  **重心計算**: 抽出された領域の重心を計算。
    7.  **データ出力**: 検出した重心座標を共有ファイル `footpoint.txt` に書き出し。

*   **設定ファイル**:
    *   `scanarea.txt`: スキャン領域のサイズと解像度を定義。

### 3.2 Shadow Projection (`Shadow/main2.cpp`)
検出された人の位置情報を読み取り、光源の位置を計算して影を描画します。また、光源位置をArduinoへ送信します。

*   **処理フロー**:
    1.  **データ入力**: `footpoint.txt` から人の位置座標を読み込み。
    2.  **座標平滑化**:
        *   **メディアンフィルタ**: スパイクノイズ（突発的な外れ値）を除去。
        *   **アダプティブスムージング**: 移動速度に応じて追従性を動的に調整（停止時は安定重視、移動時は応答性重視）。
    3.  **静止判定**: 一定時間（`STATIC_OBJECT_LIMIT`）座標の変化がない場合、光源制御モードをOFFにする安全機構。
    4.  **光源計算**: 人の位置とタッチ位置（または仮想的な基準点）との関係から、影を生成するための光源ベクトルを算出。
    5.  **レンダリング**: OpenGLを用いてオブジェクトと影を描画。
    6.  **シリアル通信**: 計算された光源座標(`lightPos0`)をLEDパネルの解像度に合わせて変換し、シリアルポート経由で送信。

*   **シリアル通信フォーマット**:
    *   形式: `x,y\n` (例: `32,16\n`)
    *   ボーレート: 115200 bps

### 3.3 LED Controller (`Arduino/Cirial_tusin/Cirial_tusin.ino`)
PCから受信した座標データを基に、LEDマトリクスパネル上で光の点を移動させます。

*   **ハードウェア**: ESP32, HUB75 LED Matrix Panel (64x32)
*   **処理フロー**:
    1.  **データ受信**: シリアルポートから `x,y` 文字列を受信・パース。
    2.  **イージング処理**: 現在位置から目標位置へ、滑らかに移動するように座標を補間更新。
    3.  **色制御**: Y座標（高さ）に応じて、LEDの色（緑〜青緑成分）を動的に変化させるグラデーション効果。
    4.  **描画**: `ESP32-HUB75-MatrixPanel-I2S-DMA` ライブラリを使用して円を描画。

## 4. データフロー図

```mermaid
graph TD
    Sensor[URG LiDAR Sensor] -->|Ethernet/USB| PC_Lidar[LIDAR1b App]
    PC_Lidar -->|Write| File[footpoint.txt]
    File -->|Read| PC_Shadow[Shadow App]
    User[User Interaction] -->|Mouse/Touch| PC_Shadow
    PC_Shadow -->|OpenGL| Display[Projector/Monitor]
    PC_Shadow -->|Serial (x,y)| ESP32[Arduino/ESP32]
    ESP32 -->|HUB75| LED[LED Matrix Panel]
```

## 5. ビルドと実行方法

### 必要なライブラリ
*   **PC側**: OpenGL, GLUT, OpenCV 4.x
*   **Arduino側**: ESP32-HUB75-MatrixPanel-I2S-DMA

### コンパイルコマンド例 (macOS)

**LIDAR1b**:
```bash
cd LIDAR1b
g++ -O3 main_cv.cpp -std=c++11 `pkg-config --cflags --libs opencv4` -framework OpenGL -framework GLUT Connection_information.cpp liburg_cpp.a -Wno-deprecated
```

**Shadow**:
```bash
cd Shadow
g++ -O3 -std=c++11 main2.cpp -framework OpenGL -framework GLUT `pkg-config --cflags --libs opencv4` -Wno-deprecated
```

### 実行手順
1.  LiDARセンサーとESP32をPCに接続する。
2.  `LIDAR1b` アプリケーションを実行し、センシングを開始する。
3.  `Shadow` アプリケーションを実行し、映像投影を開始する。
