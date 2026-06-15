# Shadow Tracking — 影インタラクションシステム

LiDAR センサーとLEDパネルを連携させ、**物体の影をユーザーの手の動きに追従させる**リアルタイムインタラクティブシステムです。  
OpenGL/GLUT によるCGレンダリング、OpenCV による画像処理、Arduino (ESP32) によるLEDパネル制御を組み合わせたハードウェア連携プロジェクトです。

---

## 概要

テーブル上に置かれた物体に対して光源（LEDパネル）から光を当て、影を生成します。  
ユーザーが影に手を触れると、LiDAR がその位置を検出し、**手の動きに合わせて光源位置を再計算**することで、影がリアルタイムに移動します。

### システム構成図

```
┌────────────┐       シリアル通信       ┌─────────────────┐
│  LiDAR     │──── footpoint.txt ──────▶│                 │
│ (UBG-04LX) │                          │   Shadow (PC)   │
└────────────┘                          │  main01 / main2 │
                                        │  (OpenGL+OpenCV)│
┌────────────┐                          └────────┬────────┘
│  Kinect    │ ── scandata.txt ──▶               │ シリアル通信
│ (OpenNI)   │   (3Dスキャン用)                  ▼
└────────────┘                          ┌─────────────────┐
                                        │   Arduino/ESP32  │
       ┌────────────┐                   │ (LEDパネル制御)  │
       │  scan (PCL) │                  │   64×32 HUB75   │
       │ 点群→メッシュ│                  └─────────────────┘
       └────────────┘
```

---

## ディレクトリ構成

```
shadow_tracking/
├── Shadow/                        # CGレンダリング＆影制御（標準LEDパネル版）
│   ├── main01.cpp                 # 単一光源版（光源1つで影を操作）
│   ├── main2.cpp                  # 複数光源版（光源2つで赤・緑の影を個別操作）
│   ├── Obj_01〜05.png / Object_01.png  # テクスチャ画像（物体の表示用）
│   └── friends.png                # テクスチャ画像
│
├── Shadow_big_LED/                # CGレンダリング＆影制御（大型LEDパネル版）
│   ├── main01.cpp                 # 単一光源版（大型LEDパネル対応）
│   ├── main01_Scan.cpp            # PLY形式3Dモデル読み込み対応の拡張版
│   ├── main2.cpp                  # 複数光源版（大型LEDパネル対応）
│   ├── Obj_01〜05.png / Object_01.png  # テクスチャ画像
│   └── friends.png                # テクスチャ画像
│
├── LIDAR1b/                       # LiDARデータ取得プログラム
│   ├── main_cv.cpp                # LiDARスキャン＋物体検出の本体
│   ├── Lidar.h                    # LiDAR処理用ヘッダ
│   ├── Urg_driver.h               # URGセンサードライバヘッダ
│   ├── detect_os.h                # OS判別用ヘッダ
│   ├── Connection_information.cpp/h  # 接続情報管理
│   ├── liburg_cpp.a               # URGライブラリ（静的リンク）
│   ├── scanarea.txt               # スキャン領域設定ファイル (幅,高さ,解像度)
│   ├── footpoint.txt              # 検出した足元座標の出力ファイル
│   ├── mark.png                   # スキャン点描画用テクスチャ
│   └── README_LiDAR.md            # LiDARモジュールの詳細ドキュメント
│
├── Kinect/
│   └── kinectbase0/               # Kinect深度・骨格検出プログラム
│       ├── main.cpp / main00.cpp / main01.cpp  # Kinectアプリケーション
│       ├── SamplesConfig.xml      # OpenNI設定ファイル
│       ├── scandata.txt           # 3D点群データの出力ファイル
│       └── README.md              # Kinectモジュールの詳細ドキュメント
│
├── scan/                          # 点群→3Dメッシュ変換ツール (PCL)
│   ├── main01_Scan.cpp            # 点群データをPLY形式メッシュに変換
│   ├── CMakeLists.txt             # CMakeビルド設定
│   ├── scandata.txt〜scandata4.txt  # 入力用スキャンデータ
│   └── output_model.ply           # 出力された3Dメッシュモデル
│
└── Arduino/                       # ESP32 LEDパネル制御スケッチ
    ├── main01/
    │   └── main01.ino             # 単一光源版（光源点1つのイージング描画）
    └── main02/
        └── main02.ino             # 複数光源版（光源点2つの独立イージング描画）
```

---

## 各モジュール詳細

### 1. Shadow（標準LEDパネル版CGプログラム）

OpenGL/GLUTを使用して3つのウィンドウを管理します。

| ウィンドウ | 名前 | 役割 |
|:---:|:---:|:---|
| CG0 | 光源視点映像 | 光源位置から見た物体のレンダリング（影の計算元） |
| CG1 | 影エリア映像 | 上方からの俯瞰ビューに影をテクスチャマッピングで投影 |
| CG2 | LED表示映像 | LEDパネルに表示する光源点の位置を可視化 |

#### `main01.cpp`（単一光源版）
- **光源**: 1つ
- **影の操作**: マウスクリック＆ドラッグ、またはLiDARによる手の検出
- **動作原理**: 影の中をクリック → 光線と物体の衝突点を固定 → 手の移動に応じて光源位置を逆算

#### `main2.cpp`（複数光源版）
- **光源**: 2つ（赤色影・緑色影）
- **影の識別**: 影のピクセル色で赤影 / 緑影 / 重なり影を判別
- **追加機能**:
  - 移動平均フィルタ（10サンプル）によるLiDARデータの平滑化
  - 極端な移動の検出・無視（最大100mm/フレーム）
  - 微小移動のデッドゾーン（最小5mm）
  - LEDパネルへのシリアル通信送信

#### キーボード操作

| キー | 機能 |
|:---:|:---|
| `d` | デバッグ表示モード切替（光源位置・衝突点・タッチ位置の可視化） |
| `r` | カメラ視点リセット（俯瞰に戻す） |
| `t` | 光源位置を初期値にリセット |
| `0` | 物体をテクスチャ板に変更 |
| `1` | 物体を立方体に変更 |
| `2` | 物体を立方体+球体に変更 |
| `f` | フルスクリーン |
| `ESC` | 終了 |

---

### 2. Shadow_big_LED（大型LEDパネル版CGプログラム）

Shadow の大型LEDパネル対応版です。基本的な構成は Shadow と同様ですが、以下の拡張が含まれます。

#### `main01_Scan.cpp`（PLYモデル対応版）
- **PLYモデル読み込み**: `scan/` モジュールで生成した3Dメッシュ（PLY形式）をシーンに配置可能
- **バウンディングボックス自動計算**: モデルの座標範囲を解析し、床面への配置オフセットを自動算出
- **回転・変換処理**: Kinectのスキャン座標系からOpenGLの描画座標系への変換を実装
- **物体モード**: キー `3` でPLYモデル表示に切り替え

#### パラメータ設定
- LEDパネルのアスペクト比（9:16）に基づくウィンドウサイズ自動計算
- 机の高さ（`deskHeight = 730.0mm`）を考慮した描画座標の調整

---

### 3. LIDAR1b（LiDARスキャンプログラム）

- **使用センサー**: HOKUYO UBG-04LX-F01（2D LiDAR）
- **スキャン範囲**: 正面±90°（計180°）
- **処理フロー**:
  1. LiDARから距離データを取得
  2. 極座標 → 直交座標に変換しOpenGLで描画
  3. フレームバッファをOpenCVで読み取り
  4. グレースケール化 → モルフォロジー処理（膨張・収縮）
  5. 輪郭検出 → 重心計算
  6. スムージング処理（移動閾値フィルタ）
  7. `footpoint.txt` に座標を出力
- **出力フォーマット**: `footpoint.txt`
  ```
  検出数
  x座標,y座標
  ```

> **詳細**: [README_LiDAR.md](LIDAR1b/README_LiDAR.md) を参照

---

### 4. Kinect（深度・骨格検出プログラム）

Kinect（OpenNI 1.x）を使用して、物体の3Dスキャンデータを取得します。

- **機能**:
  - 深度画像・ユーザーラベルの取得
  - スケルトン（関節・骨格）検出とOpenGL 3D描画
  - 点群データの3D空間への投影・カラー表示
  - OpenCVによる深度画像のウィンドウ表示
  - 点群データの `scandata.txt` への書き出し（`s` キー）
- **依存ライブラリ**: OpenNI 1.x, OpenGL/GLUT, OpenCV 4
- **操作**:
  - マウス右ボタンドラッグ: 3D視点の回転
  - `s`: 点群データを `scandata.txt` にエクスポート
  - `f`: フルスクリーン切替
  - `ESC`: 終了

> **詳細**: [kinectbase0/README.md](Kinect/kinectbase0/README.md) を参照

---

### 5. scan（点群→3Dメッシュ変換ツール）

Kinect でスキャンした点群データ（`scandata.txt`）から、PCL（Point Cloud Library）を用いて3Dメッシュモデルを生成します。

- **処理フロー**:
  1. テキストファイルから点群データ（X, Y, Z）を読み込み
  2. 奥行きフィルタ（`z_max_limit = 600.0mm`）で物体領域を抽出
  3. ユークリッドクラスタリングで最大クラスタを選出
  4. 境界点検出 → 床面までの壁面点群を自動生成
  5. 法線推定 → Greedy Projection Triangulation で三角メッシュを構築
  6. `output_model.ply` として保存
- **ビルドシステム**: CMake
- **出力**: PLY形式の3Dメッシュ（Shadow_big_LED の `main01_Scan.cpp` で読み込み可能）

---

### 6. Arduino / ESP32（LEDパネル制御）

#### `main01.ino`（単一光源版）
- **ハードウェア**: ESP32 + HUB75 64×32 RGBマトリクスLEDパネル
- **ライブラリ**: `ESP32-HUB75-MatrixPanel-I2S-DMA`
- **通信**: シリアル通信（115200bps）
- **動作**: PCから受信した座標 (`x,y\n`) にイージング処理を適用し、光源点を滑らかに移動描画
- **イージング**: 目標Y座標に応じて速度を動的変更（MIN_EASING=0.1 〜 MAX_EASING=0.9）

#### `main02.ino`（複数光源版）
- **光源数**: 2つ（構造体 `Circle` の配列で管理）
- **受信フォーマット**: `lightID,x,y\n`
- **描画**: 各光源点を独立してイージング処理し、同時描画

---

## 影操作の原理

1. **光源→影→物体の幾何学的関係を利用**
2. ユーザーが影に触れた瞬間、光線（光源→タッチ位置）と物体平面の交点（衝突点）を固定
3. 手の移動に応じて、衝突点を固定したまま新しい光源位置を逆算:
   ```
   方向ベクトル = normalize(衝突点 - 新しい手の位置)
   t = (光源パネルのZ位置 - 手のZ位置) / 方向ベクトルのZ成分
   新しい光源X = 手のX + 方向ベクトルX × t
   新しい光源Y = 手のY + 方向ベクトルY × t
   ```
4. 光源位置の更新により、影が手の動きに追従する

---

## ビルド方法

### Shadow / Shadow_big_LED（macOS）

```bash
# main01（単一光源版）
g++ -O3 -std=c++11 main01.cpp -framework OpenGL -framework GLUT \
    $(pkg-config --cflags --libs opencv4) -Wno-deprecated

# main01_Scan（PLYモデル対応版 — Shadow_big_LED のみ）
g++ -O3 -std=c++11 main01_Scan.cpp -framework OpenGL -framework GLUT \
    $(pkg-config --cflags --libs opencv4) -Wno-deprecated

# main2（複数光源版）
g++ -O3 -std=c++11 main2.cpp -framework OpenGL -framework GLUT \
    $(pkg-config --cflags --libs opencv4) -Wno-deprecated
```

### LIDAR1b（macOS）

```bash
g++ -O3 main_cv.cpp -std=c++11 \
    $(pkg-config --cflags --libs opencv4) \
    -framework OpenGL -framework GLUT \
    Connection_information.cpp liburg_cpp.a -Wno-deprecated
```

### Kinect（macOS）

```bash
g++ -O3 -std=c++11 main.cpp $(pkg-config --cflags --libs opencv4) \
    -framework OpenGL -framework GLUT \
    -I/usr/include/ni /usr/lib/libOpenNI.dylib -Wno-deprecated
```

### scan（CMake）

```bash
cd scan/build
cmake ..
make
cd ../..
./scan/build/PointCloudToMesh
```

### Arduino

Arduino IDE または PlatformIO で `.ino` ファイルを ESP32 にアップロード。

**必要ライブラリ**: `ESP32-HUB75-MatrixPanel-I2S-DMA`

---

## 依存関係

| ライブラリ | 用途 |
|:---|:---|
| OpenGL / GLUT | 3DCGレンダリング |
| OpenCV 4 | 画像処理（グリーンバック透過、影領域判定、LiDAR点群処理） |
| URGライブラリ (`liburg_cpp.a`) | HOKUYO LiDAR通信 |
| ESP32-HUB75-MatrixPanel-I2S-DMA | LEDマトリクスパネル制御 |
| OpenNI 1.x | Kinectデバイスへのアクセス |
| PCL (Point Cloud Library) | 点群処理・3Dメッシュ生成 |

---

## 実行手順

### 基本フロー（LiDAR + Shadow + LED）

1. **LiDARプログラムを起動** — 手の位置検出を開始
   ```bash
   cd LIDAR1b && ./a.out
   ```
2. **Shadowプログラムを起動** — CG描画＆影制御
   ```bash
   cd Shadow && ./a.out
   ```
3. **ESP32を接続** — LEDパネルに光源が表示される
4. テーブル上の影に手を触れると、影が手に追従して動く

### 3Dスキャンモデルを使用する場合

1. **Kinectで物体をスキャン** — `scandata.txt` を生成
   ```bash
   cd Kinect/kinectbase0 && ./a.out
   # 's' キーを押して点群データを保存
   ```
2. **点群を3Dメッシュに変換** — `output_model.ply` を生成
   ```bash
   cd scan/build && cmake .. && make && cd ../..
   ./scan/build/PointCloudToMesh
   ```
3. **Shadow_big_LED で表示** — PLYモデルを使った影のインタラクション
   ```bash
   cd Shadow_big_LED && ./a.out
   # キー '3' でPLYモデル表示に切り替え
   ```

---

## シリアル通信設定

- **ポート**: `/dev/cu.usbserial-10`
- **ボーレート**: 115200 bps
- **データ形式**: 8N1（8ビット, パリティなし, ストップビット1）
