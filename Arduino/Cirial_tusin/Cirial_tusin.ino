#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>

// パネルの解像度と接続数
#define RES_X 64 // LEDパネルのピクセル幅
#define RES_Y 32 // LEDパネルのピクセル高さ
#define CHAIN 1  // 接続するLEDパネルの数

// PCから送られてくる座標データの最大範囲（スケール変換用）
const float PC_COORDS_MAX_X = 40;
const float PC_COORDS_MAX_Y = 16; 

MatrixPanel_I2S_DMA *dma_display = nullptr;


// 円の描画パラメータ
int16_t circle_radius = 3;                     // 円の半径
uint8_t r_val = 255, g_val = 255, b_val = 255; // 円の色

// 円の座標に関する変数
float current_x; // 円の現在のX座標
float current_y; // 円の現在のY座標
int16_t target_x;  // 目標のX座標
int16_t target_y;  // 目標のY座標

// 移動の滑らかさを調整する係数
float easing_factor; 

// easing_factorの最小値と最大値を設定
const float MIN_EASING = 0.1; // 最も遅い速度
const float MAX_EASING = 0.9;  // 最も速い速度


/**
 * @brief 現在の座標に円を描画します。
 * @param x 円の中心x座標
 * @param y 円の中心y座標
 */
 
void drawCircle(int16_t x, int16_t y) {
  // 1. 画面全体を黒で塗りつぶして、前の描画を消去します。
  dma_display->fillScreenRGB888(0, 0, 0);

  // 2. 指定された座標(x, y)に円を描画します。
  dma_display->fillCircle(x, y, circle_radius, dma_display->color565(r_val, g_val, b_val));
}

void setup() {
  HUB75_I2S_CFG mxconfig(RES_X, RES_Y, CHAIN);
  mxconfig.clkphase = false;

  dma_display = new MatrixPanel_I2S_DMA(mxconfig);
  dma_display->begin();
  dma_display->setBrightness8(255);       // 明るさを設定 (0-255)
  dma_display->fillScreenRGB888(0, 0, 0); // 起動時に画面を黒でクリア

  Serial.begin(115200); 
  Serial.println("ESP32 setup complete. Waiting for coordinate data (e.g., 'x,y')...");

  // 円の初期座標と目標座標をパネル中央に設定
  current_x = RES_X / 2.0;
  current_y = RES_Y / 2.0;
  target_x = RES_X / 2;
  target_y = RES_Y / 2;
  
  // 起動時に初期位置へ円を描画
  drawCircle(current_x, current_y);
}

void loop() {
  // --- ▼▼▼ データ受信処理 ▼▼▼ ---
  if (Serial.available() > 0) {
    String received_data = Serial.readStringUntil('\n');
    int comma_index = received_data.indexOf(',');

    if (comma_index > 0) {
      String x_string = received_data.substring(0, comma_index);
      String y_string = received_data.substring(comma_index + 1);
      
      target_x = x_string.toInt();
      target_y = y_string.toInt();
      
      // Y軸の反転（必要に応じて有効化してください）
      target_y = (RES_Y - 1) - target_y;
    }
  }

  // --- ▼▼▼ イージング係数の計算 ▼▼▼ ---
  float ratio = (float)target_y / (RES_Y - 1);
  easing_factor = MIN_EASING + (MAX_EASING - MIN_EASING) * ratio;

  // --- ▼▼▼ 座標の更新 ▼▼▼ ---
  current_x += (target_x - current_x) * easing_factor;
  current_y += (target_y - current_y) * easing_factor;

  // --- ▼▼▼ 色変更処理（ここを変更しました）▼▼▼ ---

  // current_y の位置 (0 ~ 31) を、0.0 ~ 1.0 の割合に変換します
  float color_ratio = current_y / (float)(RES_Y - 1);

  // if (color_ratio < 0.0) color_ratio = 0.0;
  // if (color_ratio > 1.0) color_ratio = 1.0;

  r_val =255;
  
  g_val = (uint8_t)(255 * (color_ratio));
  
  b_val = (uint8_t)(255 * (color_ratio)); 

  // --- ▲▲▲ 色変更処理 ▲▲▲ ---
  // r_val = 255;
  // g_val = 255;
  // b_val = 255;

  // 描画を行う
  drawCircle( (int16_t)current_x, (int16_t)current_y );
  
  // アニメーション用ウェイト
  delay(20);
}