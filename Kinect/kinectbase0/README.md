# Kinect Base 0

Kinect（OpenNI）を用いて深度画像やユーザーの骨格情報（スケルトン）を取得し、OpenGLおよびOpenCVを用いて3D空間上にレンダリング・可視化を行うC++ベースのアプリケーションです。

## 概要

本プログラムは以下の機能を持っています。
- Kinectからの深度情報、画像情報、ユーザーラベルの取得
- ユーザーのスケルトン（関節点や骨格）の検出とOpenGLを用いた3D描画
- スキャンされた点群データの3D空間への投影とカラー表示
- OpenCVを用いた深度画像のウィンドウ表示 (DEPTHウィンドウ)
- 点群データのテキストファイル (`scandata.txt`) への保存機能

## 動作環境・依存ライブラリ

本プログラムをビルド・実行するためには、以下のライブラリおよび環境が必要です。

- **OS**: macOS（`-framework OpenGL -framework GLUT` や `.dylib` 指定のため）
- **C++**: C++11対応コンパイラ
- **OpenNI 1.x**: Kinectデバイスへのアクセスに使用
- **OpenGL / GLUT**: 3Dレンダリング、ウィンドウシステムに使用
- **OpenCV 4**: 深度画像の処理および表示に使用

## ディレクトリ構成

- `main.cpp`: メインのソースコード
- `SamplesConfig.xml`: OpenNIの初期化に必要な設定ファイル
- `scandata.txt`: プログラム実行中に点群データをエクスポートした際の出力ファイル（キー操作によって生成されます）

## コンパイル方法

`main.cpp` の冒頭に記載されている通り、以下のコマンドでコンパイルできます。
お使いの環境に合わせてパスなどを調整してください。

```bash
g++ -O3 -std=c++11 main.cpp `pkg-config --cflags --libs opencv4` -framework OpenGL -framework GLUT -I/usr/include/ni /usr/lib/libOpenNI.dylib -Wno-deprecated
```

## 実行方法

コンパイルによって生成された実行可能ファイル（例：`a.out`）を実行します。
実行時、カレントディレクトリに `SamplesConfig.xml` が配置されている必要があります。

```bash
./a.out
```

## 操作方法

アプリケーション実行中のウィンドウ内で、以下の操作が可能です。

### マウス操作
- **右ボタンドラッグ**: 3D空間の視点（カメラ）の回転（水平・垂直方向）

### キーボード操作
- `f` キー: ウィンドウをフルスクリーン表示に切り替えます。
- `s` キー: 現在の点群データ（X, Y, Z座標）を `scandata.txt` に書き出します。
- `Esc` キー: プログラムを終了し、リソースを解放します。

## 注意事項
- OpenNIのバージョンは1.x系を前提としています。
- 実行時にKinectが正しく接続されていることを確認してください。Kinectが見つからない場合は、プログラム内のMockDepthGeneratorによるモックデータでの動作を試みます。
- `UserCalibration.bin` や `SamplesConfig.xml` のパスが正しく読み込める状態である必要があります。
