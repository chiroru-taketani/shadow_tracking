#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>

// パネルの解像度と接続数
#define RES_X 64 // LEDパネルのピクセル幅
#define RES_Y 32 // LEDパネルのピクセル高さ
#define CHAIN 1  // 接続するLEDパネルの数

// PCから送られてくる座標データの最大範囲（スケール変換用）
// この値は、PC側のプログラムが扱う座標空間の大きさに合わせて調整してください。
// 例: PC側が幅25.6cmの範囲で座標を計算している場合
const float PC_COORDS_MAX_X = 40;
const float PC_COORDS_MAX_Y = 16; // 例: 幅の半分

MatrixPanel_I2S_DMA *dma_display = nullptr;


// 円の描画パラメータ
int16_t circle_radius = 3;                     // 円の半径
uint8_t r_val = 255, g_val = 255, b_val = 255; // 円の色 (初期値: 白)

// 円の座標に関する変数
float current_x; // 円の現在のX座標（滑らかな移動のために浮動小数点数で管理）
float current_y; // 円の現在のY座標（滑らかな移動のために浮動小数点数で管理）
int16_t target_x;  // 目標のX座標
int16_t target_y;  // 目標のY座標

// --- ▼▼▼ 変更・追加した変数 ▼▼▼ ---

// 移動の滑らかさを調整する係数（動的に変更するためconstを外す）
float easing_factor; 

// easing_factorの最小値と最大値を設定
// この値を調整することで、速度の変化具合をカスタマイズできます。
const float MIN_EASING = 0.05; // 最も遅い速度 (target_y = 0 の時)
const float MAX_EASING = 0.6;  // 最も速い速度 (target_y = 32 の時)

// --- ▲▲▲ 変更・追加した変数 ▲▲▲ ---


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

  Serial.begin(115200); // PCとのシリアル通信を開始
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
  // シリアルポートにデータが送信されているか確認します
  if (Serial.available() > 0) {
    // 改行文字 '\n' までを1つの文字列として読み取ります
    String received_data = Serial.readStringUntil('\n');

    // 受信した文字列からカンマ ',' の位置を探します
    int comma_index = received_data.indexOf(',');

    // カンマが見つかった場合のみ、座標を解析します
    if (comma_index > 0) {
      // カンマより前の部分をX座標の文字列として抽出
      String x_string = received_data.substring(0, comma_index);
      // カンマより後の部分をY座標の文字列として抽出
      String y_string = received_data.substring(comma_index + 1);
      
      // 文字列を整数（int）に変換して、目標座標として保存
      target_x = x_string.toInt();
      target_y = y_string.toInt();
      
      // Y軸の反転（もし必要ならここで行う）
      target_y = (RES_Y - 1) - target_y;
    }
  }


  // target_y の位置(0~32)を、0.0~1.0の割合に変換します
  float ratio = (float)target_y / (RES_Y - 1);

  //ratio = 1.0 - ratio;

  // 割合に応じて、MIN_EASING と MAX_EASING の間で easing_factor を計算します
  easing_factor = MIN_EASING + (MAX_EASING - MIN_EASING) * ratio;

  

  // 現在の座標を目標の座標に少しだけ近づける
  current_x += (target_x - current_x) * easing_factor;
  current_y += (target_y - current_y) * easing_factor;

  // --- ▼▼▼ 色変更処理▼▼▼ ---

  // current_y の位置 (0 ~ RES_Y-1) を、0.0 ~ 1.0 の割合に変換します
  float color_ratio  = current_y / (float)(RES_Y - 1);

  // 割合に応じて色を計算します (青 -> 赤 のグラデーション)
  // Yが上 (0) のとき: r=0, b=255
  // Yが下 (31)のとき: r=255, b=0
  r_val = 200.0;
  g_val = (uint8_t)(255.0 * color_ratio);
  b_val = (uint8_t)(255.0 * color_ratio);
  // --- ▲▲▲ 色変更処理 ▲▲▲ ---

  // 描画を行う
  
  drawCircle( (int16_t)current_x, (int16_t)current_y );
  
  // アニメーションがスムーズに見えるように、少しだけ待機します
  delay(50);
  // --- ▲▲▲ アニメーション処理 ▲▲▲ ---
}