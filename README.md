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
                                        └────────┬────────┘
                                                 │ シリアル通信
                                                 ▼
                                        ┌─────────────────┐
                                        │   Arduino/ESP32  │
                                        │ (LEDパネル制御)  │
                                        │   64×32 HUB75   │
                                        └─────────────────┘
```

---

## ディレクトリ構成

```
shadow_tracking/
├── Shadow/                    # メインのCGレンダリング＆影制御プログラム
│   ├── main01.cpp             # 単一光源版（光源1つで影を操作）
│   ├── main2.cpp              # 複数光源版（光源2つで赤・緑の影を個別操作）
│   ├── Obj_01〜04.png         # テクスチャ画像（物体の表示用）
│   └── friends.png            # テクスチャ画像
│
├── LIDAR1b/                   # LiDARデータ取得プログラム
│   ├── main_cv.cpp            # LiDARスキャン＋物体検出の本体
│   ├── Lidar.h                # LiDAR処理用ヘッダ
│   ├── Urg_driver.h           # URGセンサードライバヘッダ
│   ├── Connection_information.cpp/h  # 接続情報管理
│   ├── liburg_cpp.a           # URGライブラリ（静的リンク）
│   ├── scanarea.txt           # スキャン領域設定ファイル (幅,高さ,解像度)
│   ├── footpoint.txt          # 検出した足元座標の出力ファイル
│   └── mark.png               # スキャン点描画用テクスチャ
│
└── Arduino/
    └── Cirial_tusin/
        └── Cirial_tusin.ino   # ESP32 + HUB75 LEDパネル制御スケッチ
```

---

## 各モジュール詳細

### 1. Shadow（メインCGプログラム）

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

### 2. LIDAR1b（LiDARスキャンプログラム）

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

### 3. Arduino / ESP32（LEDパネル制御）

- **ハードウェア**: ESP32 + HUB75 64×32 RGBマトリクスLEDパネル
- **ライブラリ**: `ESP32-HUB75-MatrixPanel-I2S-DMA`
- **通信**: シリアル通信（115200bps）
- **動作**: PCから受信した座標 (`x,y`) にイージング処理を適用し、光源点を滑らかに移動描画
- **受信フォーマット**: `x,y\n`（main2の場合は `lightID,x,y\n`）

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

### Shadow（macOS）

```bash
# main01（単一光源版）
g++ -O3 -std=c++11 main01.cpp -framework OpenGL -framework GLUT \
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

### Arduino

Arduino IDE または PlatformIO で `Cirial_tusin.ino` を ESP32 にアップロード。

**必要ライブラリ**: `ESP32-HUB75-MatrixPanel-I2S-DMA`

---

## 依存関係

| ライブラリ | 用途 |
|:---|:---|
| OpenGL / GLUT | 3DCGレンダリング |
| OpenCV 4 | 画像処理（グリーンバック透過、影領域判定、LiDAR点群処理） |
| URGライブラリ (`liburg_cpp.a`) | HOKUYO LiDAR通信 |
| ESP32-HUB75-MatrixPanel-I2S-DMA | LEDマトリクスパネル制御 |

---

## 実行手順

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

---

## シリアル通信設定

- **ポート**: `/dev/cu.usbserial-10`
- **ボーレート**: 115200 bps
- **データ形式**: 8N1（8ビット, パリティなし, ストップビット1）
