#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>

// パネルの解像度と接続数
#define RES_X 64 // LEDパネルのピクセル幅
#define RES_Y 32 // LEDパネルのピクセル高さ
#define CHAIN 1  // 接続するLEDパネルの数

MatrixPanel_I2S_DMA *dma_display = nullptr;

// 円の数と描画パラメータ
const int NUM_CIRCLES = 2;       // 描画する円の数
int16_t circle_radius = 3;       // 円の半径

// 1つの円に関するデータをまとめる「構造体」
struct Circle {
  float current_x;   // 現在のX座標
  float current_y;   // 現在のY座標
  int16_t target_x;  // 目標のX座標
  int16_t target_y;  // 目標のY座標
  uint8_t r_val;     // 赤の明るさ
  uint8_t g_val;     // 緑の明るさ
  uint8_t b_val;     // 青の明るさ
};

// NUM_CIRCLES(2)個の円のデータを入れる配列を用意
Circle circles[NUM_CIRCLES];

// easing_factor(移動の滑らかさ)の最小値と最大値を設定
const float MIN_EASING = 0.1; // 最も遅い速度
const float MAX_EASING = 0.9; // 最も速い速度


void setup() {
  HUB75_I2S_CFG mxconfig(RES_X, RES_Y, CHAIN);
  mxconfig.clkphase = false;

  dma_display = new MatrixPanel_I2S_DMA(mxconfig);
  dma_display->begin();
  dma_display->setBrightness8(255);       // 明るさを設定 (0-255)
  dma_display->fillScreenRGB888(0, 0, 0); // 起動時に画面を黒でクリア

  Serial.begin(115200); 
  Serial.println("ESP32 setup complete. Waiting for coordinate data (e.g., '0,38,23')...");

  // すべての円の初期状態を設定
  for (int i = 0; i < NUM_CIRCLES; i++) {
    circles[i].current_x = RES_X / 2.0;
    circles[i].current_y = RES_Y / 2.0;
    circles[i].target_x = RES_X / 2;
    circles[i].target_y = RES_Y / 2;
    circles[i].r_val = 255;
    circles[i].g_val = 255;
    circles[i].b_val = 255;
  }
  
  // 起動時に初期位置へ円を描画
  drawCircles();
}

/**
 * @brief すべての円を画面に描画します。
 */
void drawCircles() {
  // 1. 画面全体を黒で塗りつぶして、前の描画を消去します。
  dma_display->fillScreenRGB888(0, 0, 0);

  // 2. ループを使って、すべての円を描画します。
  for (int i = 0; i < NUM_CIRCLES; i++) {
    dma_display->fillCircle(
      (int16_t)circles[i].current_x, 
      (int16_t)circles[i].current_y, 
      circle_radius, 
      dma_display->color565(circles[i].r_val, circles[i].g_val, circles[i].b_val)
    );
  }
}

void loop() {
  // --- ▼▼▼ データ受信処理 ▼▼▼ ---
  if (Serial.available() > 0) {
    
    // 受信データを\nまで文字列として読み込む
    String received_data = Serial.readStringUntil('\n');

    // 1つ目のカンマと2つ目のカンマの位置を探す
    int first_comma = received_data.indexOf(','); 
    int second_comma = received_data.indexOf(',', first_comma + 1);

    // カンマが正しく2つ見つかった場合のみ処理を実行
    if (first_comma > 0 && second_comma > first_comma) {
      // 文字列を分割する
      String id_string = received_data.substring(0, first_comma);
      String x_string = received_data.substring(first_comma + 1, second_comma);
      String y_string = received_data.substring(second_comma + 1);
      
      // 文字列を整数に変換
      int id = id_string.toInt();
      
      // IDが0または1の場合に、目標座標を更新
      if (id >= 0 && id < NUM_CIRCLES) {
        circles[id].target_x = x_string.toInt();
        circles[id].target_y = y_string.toInt();
        Serial.println("Received: ID=" + String(id) + ", X=" + String(circles[id].target_x) + ", Y=" + String(circles[id].target_y));
      }
    }
  }

  // --- ▼▼▼ イージングと座標の更新 ▼▼▼ ---
  // すべての円に対して位置を計算
  for (int i = 0; i < NUM_CIRCLES; i++) {
    float ratio = (float)circles[i].target_y / (RES_Y - 1);
    float easing_factor = MIN_EASING + (MAX_EASING - MIN_EASING) * ratio;

    circles[i].current_x += (circles[i].target_x - circles[i].current_x) * easing_factor;
    circles[i].current_y += (circles[i].target_y - circles[i].current_y) * easing_factor;
  }

  // 描画を行う
  drawCircles();
  
  // アニメーション用ウェイト
  delay(20);
}