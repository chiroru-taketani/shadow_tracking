//LiDARの測距結果を画像表示するプログラム
/*
 * g++ -O3 main_cv2.cpp -std=c++11 `pkg-config --cflags --libs opencv4` -framework OpenGL -framework GLUT Connection_information.cpp liburg_cpp.a -Wno-deprecated
 */

#include <iostream> 
#include <time.h>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <GLUT/glut.h>
#include "Urg_driver.h"
#include "Connection_information.h"

#define DATASIZE 513 //LiDARから取得するデータ点数の上限

//三次元ベクトルを格納するための構造体: Vec_3D
typedef struct _Vec_3D
{
    double x, y, z;
} Vec_3D;


//--関数プロトタイプ宣言--
void display();// 描画処理を行うコールバック関数
void reshape(int w, int h);//ウィンドウサイズ変更時に呼ばれるコールバック関数
void timer(int value);//一定時間ごとに呼ばれるコールバック関数
void keyboard(unsigned char key, int x, int y);//キーボード入力
void initGL();//初期化関数

//--グローバル変数--
double fr = 60;//フレームレート(fps)
GLdouble model[16], proj[16];
GLint view[4];
qrk::Urg_driver urg;
std::vector<long> distData;

//--スキャン設定--
int scanAngle = 180;
double scanAngleStep = 0.35;
double scanAreaW, scanAreaH;
double scanReso;
cv::Mat scanImage, binImage;
cv::Mat element;

//--ファイル関連--
FILE *fp;
char areaSizeFile[] = "scanarea.txt";
char footPointFile[] = "footpoint.txt";
double rDisp = 1.0;



int main(int argc, char *argv[])
{
    glutInit(&argc, argv);
    
    qrk::Connection_information information(argc, argv);
    if(!urg.open(information.device_or_ip_name(), information.baudrate_or_port_number(), information.connection_type()))
    {
        std::cout << "URGの接続に失敗しました!" << std::endl;
        return 1;
    }
    urg.set_scanning_parameter(urg.deg2step(-scanAngle / 2), urg.deg2step(scanAngle / 2), 0);
    urg.start_measurement(qrk::Urg_driver::Distance, qrk::Urg_driver::Infinity_times, 0);

    initGL();

    glutMainLoop();

    return 0;
    
}

