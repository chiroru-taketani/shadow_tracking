# Archive

過去の実験コードや検証用プロトタイプを保管しているディレクトリです。

## ディレクトリ一覧

### `shadowtest0621/`
- **作成時期**: 2026年6月〜7月
- **内容**: Kinect (OpenNI) による点群取得、OpenGL/GLUTによる2光源（赤影・緑影）の重なり判定、Mac Retinaディスプレイ表示補正（`rDisp`）、LiDAR連携の実験プロトタイプ。
- **現行モジュールへの移行状況**:
  - **影追従・myCobot連携**: [`Shadow_myCobot/`](../Shadow_myCobot/)（`main01_myCobot.cpp`, `main0y.cpp`）へ昇華・移行済み。
  - **Kinect点群スキャン**: [`Kinect/`](../Kinect/) および [`scan/`](../scan/) へ分離・整理済み。
  - **LiDAR制御**: [`LIDAR1b/`](../LIDAR1b/) に整理済み。

### `Shadow_myCobot_legacy/`
- **退避時期**: 2026年10月
- **内容**:
  - `main01.cpp`: myCobot初期プロトタイプ（3D位置のみ送信版）
  - `main2.cpp`: LEDパネル用（シリアル通信・2光源版）の重複ファイル
  - `main0y.cpp`: Kinect (OpenNI) 連携を直接埋め込んでいた過去プロトタイプ
- **現行モジュール**: `Shadow_myCobot/main01_myCobot.cpp` に一本化済み。
