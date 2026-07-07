# Shadow Tracking: myCobot Edition

## 概要
LiDARセンサーと**ロボットアーム（myCobot 280）**を連携させ、物体の影をユーザーの手の動きに追従させるリアルタイムインタラクティブシステムです。
従来のLEDパネルを用いた2次元的な光源制御から発展し、myCobotの先端に取り付けた物理的な光源を3次元空間で動かすことで、よりダイナミックな影の生成を実現しています。

## システム構成
```
┌────────────┐                                ┌─────────────────┐
│  LiDAR     │──── footpoint.txt ────────────▶│                 │
│ (UBG-04LX) │                                │  Shadow (C++)   │
└────────────┘                                │  (main01.cpp)   │
                                              │                 │
┌────────────┐                                │                 │
│  Kinect    │ ── scandata.txt (3Dスキャン) ─▶│                 │
└────────────┘                                └────────┬────────┘
                                                       │ UDP通信 (127.0.0.1:5005)
                                                       │ (X, Y, Z, Rx, Ry, Rz)
                                                       ▼
                                              ┌─────────────────┐
                                              │ Python Server   │
                                              │ (udp_server.py) │
                                              └────────┬────────┘
                                                       │ USBシリアル通信
                                                       ▼
                                              ┌─────────────────┐
                                              │ myCobot 280     │
                                              │ (光源アタッチ)  │
                                              └─────────────────┘
```

## 従来版（LEDパネル）からの主な変更点
- **通信プロトコルの変更**: C++からESP32へのシリアル通信を廃止し、Python制御スクリプトへの**UDP通信**に変更。これにより高速かつ安定したデータ転送を実現しています。
- **3次元座標と姿勢制御**: 光源の位置を2D(X,Y)から3D(X,Y,Z)へ拡張。さらに、光源が常に物体を向くようC++側で方向ベクトルとオイラー角(Rx, Ry, Rz)を計算して送信しています。
- **安全装置（ソフトウェアリミット）**: Python側で可動域の制限（Clamp処理）を行い、アームの机への激突や関節の過負荷を防止しています。

## 環境構築と依存ライブラリ

### C++側 (Shadow_myCobot)
- macOS環境
- OpenGL / GLUT
- OpenCV 4
- **コンパイルコマンド**:
  ```bash
  g++ -O3 -std=c++11 main01.cpp -framework OpenGL -framework GLUT $(pkg-config --cflags --libs opencv4) -Wno-deprecated
  ```

### Python側 (myCobot制御)
- Python 3.x
- `pymycobot` ライブラリ
  ```bash
  pip3 install pymycobot
  ```
  ※または、プロジェクト内の仮想環境（`.venv`）をアクティベートして使用。

## 実行手順
アームの初期化や安全確保のため、**必ずPythonサーバーを先に起動**してください。

1. **myCobotの準備**
   - myCobotの電源を入れ、MacとUSBで接続します。
   - `python_control/udp_server.py` 内の `SERIAL_PORT` を、ご自身の環境のポート名（例: `/dev/tty.usbserial-XXXX`）に合わせて変更してください。

2. **Pythonサーバーの起動**
   ```bash
   cd python_control
   python3 udp_server.py
   ```
   ※アームが自動的に安全な初期姿勢に移動し、UDP受信待機状態になります。

3. **LiDARプログラムの起動** (※別ターミナル)
   ```bash
   cd ../LIDAR1b
   ./a.out
   ```
   ※手のトラッキングを行い、`footpoint.txt` に座標を出力します。

4. **C++ (Shadow) プログラムの起動** (※別ターミナル)
   ```bash
   cd ../Shadow_myCobot
   ./a.out
   ```
   ※ウィンドウが立ち上がり、影の計算が始まります。手の動き（またはマウスドラッグ）に合わせて、myCobotが滑らかに追従・回転します。

## ディレクトリ構成
- `main01.cpp`: 影の計算、UDP送信を行うメインプログラム
- `python_control/`: myCobot制御用Python環境
  - `udp_server.py`: UDPを受信し、myCobotを制御・安全装置を管理するスクリプト
- `README.md`: 本ドキュメント
