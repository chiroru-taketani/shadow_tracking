// LiDAR
/*
 * g++ -O3 main_cv_02.cpp -std=c++11 `pkg-config --cflags --libs opencv4` -framework OpenGL -framework GLUT Connection_information.cpp liburg_cpp.a -Wno-deprecated
 */
#include <GLUT/glut.h>
#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <time.h>

//URGセンサー用
#include "Connection_information.h"
#include "Urg_driver.h"

//URGセンサーの初期設定
#define DATASIZE 513

typedef struct _Vec_3D
{
    double x, y, z;
} Vec_3D;

//関数プロトタイプ宣言
void display();
void reshape(int w, int h);
void timer(int value);
void keyboard(unsigned char key, int x, int y);
void initGL();

//===グローバル変数===
double fr = 60;
GLdouble model[16], proj[16];
GLint view[4];

//画像定義
cv::Mat scanImage;
cv::Mat binImage;

//ファイル関係
FILE *fp;                             //ファイルポインタ
char areaSizeFile[] = "scanarea.txt"; //スキャン領域の設定ファイル名

//スキャン設定
double scanAreaW, scanAreaH; //処理する領域のサイズ（cm）（ファイルから読み込む）
double scanReso;             //1pxあたりのサイズ（cm）（ファイルから読み込む）
double scanAngle = 180.0;    // スキャンする角度の範囲 (正面を0度として±90度)
double scanAngleStep = 0.35; // URGセンサーの1ステップあたりの角度 (UBG-04LX-F01)

//LiDER関係
qrk::Urg_driver urg;
std::vector<long> distData; //LiDERから取得したデータを格納する動的配列

//メイン関数
int main(int argc, char *argv[])
{
    //OpenGLの初期化
    glutInit(&argc, argv);

    //URGセンサーの準備
    qrk::Connection_information information(argc, argv); //接続情報

    //センサーへ接続
    if (!urg.open(information.device_or_ip_name(),
                  information.baudrate_or_port_number(),
                  information.connection_type()))
    {
        printf("URGの接続に失敗しました");
        return 1;
    }

    //スキャンパラメータ（スキャン範囲を設定）
    urg.set_scanning_parameter(urg.deg2step(-scanAngle / 2), urg.deg2step(scanAngle / 2), 0);

    //距離データの計測を無限に繰り返す設定して開始
    urg.start_measurement(qrk::Urg_driver::Distance, qrk::Urg_driver::Infinity_times, 0);

    initGL();

    glutMainLoop();

    return 0;
}

//初期設定
void initGL()
{
    //===ファイルの読み込み===
    // scanarea.txtファイルを開き、スキャン領域の幅・高さ・解像度を読み込む
    fp = fopen(areaSizeFile, "r"); //ファイルを開いて場所をポインタに格納
    if (fp == NULL)
    {
        std::cout << "scanarea.txtが開けません" << std::endl;
    }
    else
    {
        //ファイルを書き込む
        fscanf(fp, "%lf, %lf, %lf", &scanAreaW, &scanAreaH, &scanReso);
        //ファイルを閉じる
        fclose(fp);
    }
    //======

    //===画像定義===
    scanImage = cv::Mat(cv::Size(scanAreaW / scanReso, scanAreaH / scanReso, CV_8UC3)); //カラー画像
    binImage = cv::Mat(cv::Size(scanAreaW / scanReso, scanAreaH / scanReso, CV_8UC1));  //二値画像
    //======

    //===GLUTウィンドウの生成===
    glutInitWindowSize(scanAreaW / scanReso, scanAreaH / scanReso); //ウィンドウサイズの指定
    glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);      //ディスプレイモード設定
    glutInitWindowPosition(0, 0);                                   //ウィンドウの表示位置
    glutCreateWindow("CG");                                         //ウィンドウ生成
    //======

    //===コールバック関数の登録===
    glutDisplayFunc(display);           //描画にdisplay関数を使用
    glutReshapeFunc(reshape);           //ウィンドウ変更時にreshape関数を使用
    glutKeyboardFunc(keyboard);         //キー入力にkeyboardを使用
    glutTimerFunc(1000 / fr, timer, 0); //1000/frms後からtimer関数を呼び出す
    //======

    //===OpenGLの描画設定===
    glClearColor(0.0, 0.0, 0.0, 1.0);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_ALPHA_TEST);       //アルファテストを有効化
    glAlphaFunc(GL_GREATER, 0.01); //透明度が0.01より大きいピクセルのみ描画
    //======

    //===テクスチャの読み込み===
    //スキャン点を描画するための画像(mark.png)を読み込む
    cv::Mat textureImage = cv::imread("mark.png", cv::IMREAD_UNCHANGED);
    if (!textureImage.empty())
    {
        glBindTexture(GL_TEXTURE_2D, 0); //テクスチャを0番に登録
        glTexParameteri(GL_TEXTURE_2D,
                        GL_TEXTURE_MAG_FILTER,
                        GL_NEAREST); //画像を拡大した時の処理（ぼかし処理を入れない）
        glTexParameteri(GL_TEXTURE_2D,
                        GL_TEXTURE_MIN_FILTER,
                        GL_NEAREST); //画像を縮小した時の処理（ぼかし処理を入れない）
        glTexImage2D(GL_TEXTURE_2D,
                     0,
                     GL_RGBA,
                     textureImage.cols,
                     textureImage.rows,
                     0,
                     GL_BGRA,
                     GL_UNSIGNED_BYTE,
                     textureImage.data);
        // 画像データをGPUメモリへ転送
    }
    else
    {
        std::cout << "mark.pngが見つかりません" << std::endl;
    }

    //モルフォジー処理で使用する構造体要素（5*5の楕円系）を作成
    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    //======
}

//ディスプレイ関数
void display()
{
    //===１、LiDARデータをOpenGLで描画===
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); //画面をクリア
    glLoadIdentity();                                   //座標変換行列を初期化

    //スキャン点を描画するための設定
    glEnable(GL_TEXTURE_2D);         //テクスチャマッピングを有効化
    glColor4d(1.0, 1.0, 1.0, 1.0);   //描画色を白に設定
    glBindTexture(GL_TEXTURE_2D, 0); //使用するテクスチャを選択

    //LiDARから取得したすべてのデータ点に対してループ
    for (int i = 0; i < distData.size(); i++)
    {
        //スキャン点情報を(x,y)に変換
        double len = distData[i] / 10.0; // 距離(mm)をcmに変換
        if (len < 5.0)
        { //センサーから距離が近すぎる点は描画しない
            continue;
        }

        double rad = urg.index2rad(i); //i番目のスキャン点のラジアン
        double x = -len * sin(rad);    //i番目のスキャン点のx座標（センサー座標系→OpenGL座標系）
        double y = len * cos(rad);     //i番目のスキャン点のy座標（センサー座標系→OpenGL座標系）
        double z = 0;

        //距離が遠い点ほど円大きく描画(隙間が開くため)
        // 距離の平方根に比例させることで、大きさの変化を緩やかにしている
        double circleR = 80.0 * sqrt(len) * tan(scanAngleStep * M_PI / 180.0 / 2); //描画する円の半径を計算

        //スキャン点をテクスチャを貼った四角形として描画
        glPushMatrix();                               //現在の座標変換行列を保存
        glTranslated(x, y, z);                        //計算した座標に移動
        glRotated(rad * 180.0 / M_PI, 0.0, 0.0, 1.0); //描画する四角形を回転
        glScaled(circleR * 1.5, circleR * 1.0, 1);    //計算した半径に拡大縮小
        glBegin(GL_QUADS);                            //四角形の描画開始
        glTexCoord2d(0, 0);
        glVertex3d(-0.5, 0.5, 0.0);
        glTexCoord2d(0, 1);
        glVertex3d(-0.5, -0.5, 0.0);
        glTexCoord2d(1, 1);
        glVertex3d(0.5, -0.5, 0.0);
        glTexCoord2d(1, 0);
        glVertex3d(0.5, 0.5, 0.0);
        glEnd();       // 描画終了
        glPopMatrix(); //保存した座標変換行列を復元
    }
    glDisable(GL_TEXTURE_2D); //テクスチャマッピングを無効化

    // バックバッファに描画した内容をフロントバッファ(画面)に反映
    glutSwapBuffers();

    //===2.OpenGLの描画結果をOpenCVで画像処理

    glReadPixels(0,
                 0,
                 scanImage.cols,
                 scanImage.rows,
                 GL_BGR,
                 GL_UNSIGNED_BYTE,
                 scanImage.data);     //OpenGLのフレームバッファからピクセルデータを読み取り、scanImageに格納
    cv::flip(scanImage, scanImage, 0);//垂直方向に反転
    cv::cvtColor(scanImage, binImage, cv::COLOR_BGR2GRAY); //グレースケールに変換
    cv::threshold(binImage, binImage, 128, 255, cv::THRESH_BINARY);

    // モルフォロジー処理でノイズ除去と領域の結合を行う
    cv::dilate(binImage, binImage, element, cv::Point(-1, -1), 2); // 2回膨張させて、スキャン点間の隙間を埋める
    cv::erode(binImage,
              binImage,
              element,
              cv::Point(-1, -1),
              4); // 2回収縮させて、膨張した領域を元に戻しつつ、ノイズを除去

    cv::dilate(binImage, binImage, element, cv::Point(-1, -1), 2); // 2回膨張させて、スキャン点間の隙間を埋める
    cv::erode(binImage,
              binImage,
              element,
              cv::Point(-1, -1),
              2); // 2回収縮させて、膨張した領域を元に戻しつつ、ノイズを除去
    cv::dilate(binImage, binImage, element, cv::Point(-1, -1), 5); // 2回膨張させて、スキャン点間の隙間を埋める

    cv::imshow("binImage", binImage);//デバック用表示

//===３、輪郭検出===
std::vector<std::vector<cv::Point>> contours;//検出された輪郭の情報を格納するベクター
cv::findContours(binImage,
                 contours,
                 cv::RETE_EXTERAL,
                 cv::CHAIN_APPROX_SIMPLE); // 画像から輪郭を検出する (RETR_EXTERNAL: 最も外側の輪郭のみを検出)

int closestTargetIndex = -1;
double minDistance = 999.0; //距離を比較するための初期値
Vec_3D closestPointF;       //最も近い物体の座標を一時保存する変数

//1,すべての輪郭をチェックして一番近い物体を探す
for(int i = 0; i < contours.size(); i++){
    double area = cont
}
}
