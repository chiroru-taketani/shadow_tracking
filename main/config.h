#ifndef CONFIG_H
#define CONFIG_H

#include <string>

// --- ユーザー設定項目 ---

// 接続するシリアルポート名
const char *PORT_NAME = "/dev/cu.usbserial-110";

// 座標データを読み込むファイル名
const std::string FILENAME = "../LIDAR1b/footpoint.txt";

// 読み取り処理の待ち時間 (ミリ秒)
const int WAIT_MILLISECONDS = 200;

// --- 座標変換の定義 ---

// 元の座標系の最大値（LiDARが認識する空間の大きさなど）
const double WORLD_MAX_X = 16; // 例: 25.6cm
const double WORLD_MAX_Y = 40; // 例: 12.8cm

// 変換先の平面（LEDパネル）の解像度
const int PANEL_RES_X = 64;
const int PANEL_RES_Y = 32;

#endif // CONFIG_H