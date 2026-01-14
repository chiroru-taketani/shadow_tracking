//g++ -O3 -std=c++11 main2.cpp -framework OpenGL -framework GLUT `pkg-config --cflags --libs opencv4` -Wno-deprecated
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <GLUT/glut.h>  //OpenGL/GLUTの使用
#include <opencv2/opencv.hpp>  //OpenCV関連ヘッダ

//定数宣言
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <vector>
#include <algorithm>


//定数宣言
#define TEX_SIZE 512  //テクスチャサイズ
#define POINTMAX 20

//三次元ベクトル構造体
typedef struct _Vec_3D
{
    double x, y, z;
    double s, t, u;
    int id;
} Vec_3D;

//関数名の宣言
void initGL();
int initSerial(const char *portName); // シリアル初期化関数宣言
void display0();
void display1();
void display2();
void reshape0(int w, int h);
void reshape1(int w, int h);
void reshape2(int w, int h);
void mouse0(int button, int state, int x, int y);
void mouse1(int button, int state, int x, int y);
void motion0(int x, int y);
void motion1(int x, int y);
void keyboard(unsigned char key, int x, int y);
void timer0(int value);
Vec_3D normcrossprod(Vec_3D v1, Vec_3D v2);
Vec_3D vectorNormalize(Vec_3D v0);
void glMyTexPlane(int texID, int w, int h);
void scene();

//マッピング関数
double mapRange(double value, double oldMin, double oldMax, double newMin, double newMax) {
    return (value - oldMin) * (newMax - newMin) / (oldMax - oldMin) + newMin;
}
// 使い方
//double result = mapRange(元の数字, 元の数字の最小値, 元の数字の最大値, 新しい数字の最小値, 新しい数字の最大値);


//===============グローバル変数 初期設定用構造体==============

// --- アプリケーション全体の管理状態 ---
struct AppConfig {
    int dispMode = 0;       // 0:通常表示, 1:デバッグ用ガイド表示（d）
    int objectFlg = 1;      // 描画する物体の種類 (0:テクスチャ板, 1:立方体, 2:立方体と球体)（数字キー）
    double frameRate = 30.0; //フレームレート
    double renderScale = 1.0; //解像度調整用のスケール係数
} g_appConfig;

// --- 空間・エリアの設定 ---
struct AreaConfig {
    double scanW = 400.0; // スキャンエリアの横幅 (mm)
    double scanH = 200.0; // スキャンエリアの縦幅 (mm)
    double aspectRate = 1.0 / 2.0; // LEDパネルのアスペクト比 (縦/横)
    double lightW = 256.0;  // LEDパネルの横幅
    double lightH = 128.0; // LEDパネルの縦幅 (lightW * aspectRate)
    double resolution = 2.0; // 1mmあたりのピクセル数．CG1のサイズが変化（画像処理の精度に影響）

    double LEDW = 64.0; // LEDパネルの横幅
    double LEDH = 32.0; // LEDパネルの縦幅
} g_areaConfig;


//---シリアル通信の情報---
struct SerialConfig {
    int fd = -1;  // ファイルディスクリプタ（-1は未接続）
    const char* port = "/dev/cu.usbserial-10";  // 通信ポートのパス
    const int baud = 115200;    // 通信速度 (bps)
} g_Serial;

int chaseFlg = 0;       // LIDARによる追従状態 (0:未検知/待機, 1:追従中)
unsigned char shadowVal = 255;  //影かどうか（0は影）

struct InteractionConfig{
    double dist = 30;  //影と手の認識距離（mm）
} g_interaction;

struct ObjectConfig{
      double scaleX = 30.0;
      double scaleY = 30.0;
      double scaleZ = 30.0;
} g_object;

// =================================================

//変数
// --- ウィンドウとマウスの状態
struct WindowInfo {
    int id;     // GLUTのウィンドウID
    int W;      // ウィンドウの横幅 (px)
    int H;      // ウィンドウの縦幅 (px)
    int mX, mY;  // マウスの位置 (px)
    int mButton, mState;  // 最後に操作されたボタンの種類と状態 (UP/DOWN)
} g_winInfo[3];

// --- 視点（カメラ）の情報 ---
struct Camera {
    double dist;  // 注視点からの距離
    double degX;  // 垂直方向の回転角度 (度)
    double degY;  // 水平方向の回転角度 (度)
}g_Cam[2];

Vec_3D objPos;  //影物体
Vec_3D lightPos0;  //LED光源座標
Vec_3D touchPos, touchPos0;  //手を触れた場所

Vec_3D lightVec;  //光線ベクトル
Vec_3D lightCollPos;  //光線が物体と当たる場所
Vec_3D footPoint[POINTMAX];
int footNum;

cv::Mat frameImage0, frameImage, frameImage1, shadowAreaImage, shadowAreaGrayImage;


//テクスチャ生成関数のパラメータ（）
static double genfunc[][4] = {
    {1.0, 0.0, 0.0, 0.0},
    {0.0, 1.0, 0.0, 0.0},
    {0.0, 0.0, 1.0, 0.0},
    {0.0, 0.0, 0.0, 1.0},
};

//メイン関数
int main(int argc, char *argv[])
{
    glutInit(&argc, argv);  //OpenGL/GLUTの初期化
    initGL();  //初期設定

     // シリアルポート初期化
    g_Serial.fd = initSerial(g_Serial.port);
    if (g_Serial.fd == -1) {
        printf("ポートが開けません: %s\n", g_Serial.port);
    } else {
        printf("ポート開きます！: %s\n", g_Serial.port);
    }

    glutMainLoop();  //イベント待ち無限ループ

    return 0;
}

// シリアル初期化関数
int initSerial(const char *portName) {
    int fd = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        return -1;
    }

    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    tcsetattr(fd, TCSANOW, &options);

    return fd;
}


//初期化関数
void initGL()
{
    //描画ウィンドウ生成0（ユーザに知覚させたい映像の生成）
    glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);  //ディスプレイモードの指定
    glutInitWindowSize(TEX_SIZE, TEX_SIZE);  //ウィンドウサイズの指定
    glutInitWindowPosition(0, 0);  //ウィンドウ位置の指定
    g_winInfo[0].id = glutCreateWindow("CG0");  //ウィンドウの生成
    //コールバック関数の指定
    glutDisplayFunc(display0);  //ディスプレイコールバック関数の指定
    glutReshapeFunc(reshape0);  //リシェイプコールバック関数の指定
    glutMouseFunc(mouse0);  //マウスクリックコールバック関数の指定
    glutMotionFunc(motion0);  //マウスドラッグコールバック関数の指定
    glutTimerFunc(1000/g_appConfig.frameRate, timer0, 0);  //タイマー
    glutKeyboardFunc(keyboard);  //キーボードコールバック関数の指定
    //各種設定
    glClearColor(0.0, 1.0, 0.0, 1.0);  //ウィンドウクリア色の指定（RGBA）
    glEnable(GL_DEPTH_TEST);  //デプスバッファの有効化
    glEnable(GL_NORMALIZE);  //ベクトル正規化有効化
    glEnable(GL_BLEND);  //アルファチャンネル有効化
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);    //アルファ関数の設定
    glEnable(GL_ALPHA_TEST);  //アルファテスト有効化
    glAlphaFunc(GL_GREATER, 0.1);  //アルファ値比較関数の設定

    //描画ウィンドウ生成1（影エリア映像の生成）
    glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);  //ディスプレイモードの指定
    glutInitWindowSize(g_areaConfig.scanW * g_areaConfig.resolution,  g_areaConfig.scanH * g_areaConfig.resolution);  //ウィンドウサイズの指定
    glutInitWindowPosition(512, 0);  //ウィンドウ位置の指定
    g_winInfo[1].id = glutCreateWindow("CG1");  //ウィンドウの生成
    //コールバック関数の指定
    glutDisplayFunc(display1);  //ディスプレイコールバック関数の指定
    glutReshapeFunc(reshape1);  //リシェイプコールバック関数の指定
    glutMouseFunc(mouse1);  //マウスクリックコールバック関数の指定
    glutMotionFunc(motion1);  //マウスドラッグコールバック関数の指定
    glutKeyboardFunc(keyboard);  //キーボードコールバック関数の指定
    //各種設定
    glClearColor(0.0, 0.0, 0.0, 1.0);  //ウィンドウクリア色の指定（RGBA）
    glEnable(GL_DEPTH_TEST);  //デプスバッファの有効化
    glEnable(GL_NORMALIZE);  //ベクトル正規化有効化
    glEnable(GL_BLEND);  //アルファチャンネル有効化
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);    //アルファ関数の設定
    glEnable(GL_ALPHA_TEST);  //アルファテスト有効化
    glAlphaFunc(GL_GREATER, 0.1);  //アルファ値比較関数の設定
    //視点関係
    g_Cam[1].dist = 250.0; g_Cam[1].degX = 90.0; g_Cam[1].degY = 0.0;  //視点極座標

    //描画ウィンドウ生成2（照明用映像の生成）
    glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);  //ディスプレイモードの指定
    glutInitWindowSize(800, 800*g_areaConfig.aspectRate);  //ウィンドウサイズの指定
    glutInitWindowPosition(512, g_areaConfig.scanH*g_areaConfig.resolution+100);  //ウィンドウ位置の指定
    g_winInfo[2].id = glutCreateWindow("CG2");  //ウィンドウの生成
    //コールバック関数の指定
    glutDisplayFunc(display2);  //ディスプレイコールバック関数の指定
    glutReshapeFunc(reshape2);  //リシェイプコールバック関数の指定
    glutKeyboardFunc(keyboard);  //キーボードコールバック関数の指定
    //各種設定
    glClearColor(0.0, 0.0, 0.0, 1.0);  //ウィンドウクリア色の指定（RGBA）
    glEnable(GL_DEPTH_TEST);  //デプスバッファの有効化
    glEnable(GL_NORMALIZE);  //ベクトル正規化有効化
    glEnable(GL_BLEND);  //アルファチャンネル有効化
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);    //アルファ関数の設定
    glEnable(GL_ALPHA_TEST);  //アルファテスト有効化
    glAlphaFunc(GL_GREATER, 0.1);  //アルファ値比較関数の設定

    //光源やテクスチャの初期設定
    cv::Mat textureImage;
    for (int i=0; i<3; i++) {
        glutSetWindow(g_winInfo[i].id);  //設定対象ウィンドウ

        //テクスチャ
        //テクスチャオブジェクト生成(#100)
        textureImage = cv::imread("Obj_01.png", cv::IMREAD_UNCHANGED);
        objPos.s = textureImage.cols; objPos.t = textureImage.rows;
        objPos.id = 100;
        glBindTexture(GL_TEXTURE_2D, objPos.id);  //テクスチャオブジェクト生成
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);  //テクスチャ反復方法(s方向)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);  //テクスチャ反復方法(t方向)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);  //テクスチャ補間方法(拡大時)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);  //テクスチャ補間方法(縮小時)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, textureImage.cols, textureImage.rows, 0, GL_BGRA, GL_UNSIGNED_BYTE, textureImage.data);

        glBindTexture(GL_TEXTURE_2D, 0);  //テクスチャオブジェクト生成
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);  //テクスチャ反復方法(s方向)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);  //テクスチャ反復方法(t方向)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);  //テクスチャ補間方法(拡大時)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);  //テクスチャ補間方法(縮小時)
    }
    
    //テクスチャ座標生成モードの指定（視点座標モード）
    glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_EYE_LINEAR);
    glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, GL_EYE_LINEAR);
    glTexGeni(GL_R, GL_TEXTURE_GEN_MODE, GL_EYE_LINEAR);
    glTexGeni(GL_Q, GL_TEXTURE_GEN_MODE, GL_EYE_LINEAR);

    frameImage = cv::Mat(cv::Size(TEX_SIZE, TEX_SIZE), CV_8UC3);
    frameImage1 = cv::Mat(cv::Size(TEX_SIZE, TEX_SIZE), CV_8UC4);
    
    //表示パネルサイズ・位置
    objPos.u = 50.0;  //テクスチャオブジェクトの横幅
    objPos.x = 0.0; 
    objPos.y = objPos.u*objPos.t/objPos.s*0.5; 
    objPos.z = g_areaConfig.scanH/2.0 + g_object.scaleZ/2.0;  //位置mm
    
    //光源位置
    lightPos0.x = 0.0;
    lightPos0.y = g_areaConfig.lightH/2.0; //光源の高さmm
    lightPos0.z = g_areaConfig.scanH/2.0 + g_object.scaleZ + 100.0; //光源の位置mm
    lightCollPos.z = objPos.z; //光源と物体との衝突位置
}

//ユーザに知覚させたいオブジェクトの描画
void scene()
{
    if(g_appConfig.objectFlg == 0) {
        //オブジェクト（本体）
        glDisable(GL_LIGHTING);
        glColor4d(1.0, 1.0, 1.0, 1.0);
        glPushMatrix();
        glTranslated(objPos.x, objPos.y, objPos.z);
        glScaled(objPos.u, objPos.u*objPos.t/objPos.s, 1.0);
        glMyTexPlane(objPos.id, objPos.s/50, objPos.t/50);
        glPopMatrix();
    }
    
    if (g_appConfig.objectFlg == 1) {//立方体

        glDisable(GL_LIGHTING);
        glColor4d(1.0, 0.0, 0.0, 1.0);
        glPushMatrix();
        glTranslated(0.0, g_object.scaleY/2.0, objPos.z);
        glScaled(g_object.scaleX, g_object.scaleY, g_object.scaleZ);
        glutSolidCube(1.0);
        glPopMatrix();
       
    }
    
    if (g_appConfig.objectFlg == 2) {//立方体と球体
        glDisable(GL_LIGHTING);
        glColor4d(1.0, 0.0, 0.0, 1.0);
        glPushMatrix();
        glTranslated(40.0, g_object.scaleY/2.0, objPos.z);
        glScaled(g_object.scaleX, g_object.scaleY, g_object.scaleZ);
        glutSolidCube(1.0);
        glPopMatrix();

        glDisable(GL_LIGHTING);
        glColor4d(1.0, 0.0, 0.0, 1.0);
        glPushMatrix();
        glTranslated(-40.0, 15.0, objPos.z);
        glScaled(15.0, 15.0, 15.0);
        glutSolidSphere(1.0, 36, 18);
        glPopMatrix();
    }
    
    if (g_appConfig.dispMode) {


        //認識範囲
        // glDisable(GL_LIGHTING);
        // glColor4d(1.0, 0.0, 0.0, 1.0);
        // glLineWidth(1.0);
        // glPushMatrix();
        // glBegin(GL_LINE_LOOP);
        // glVertex3d(g_areaConfig.scanW/2.0, 0.3, g_areaConfig.scanH/2.0);
        // glVertex3d(g_areaConfig.scanW/2.0, 0.3, -g_areaConfig.scanH/2.0);
        // glVertex3d(-g_areaConfig.scanW/2.0, 0.3, -g_areaConfig.scanH/2.0);
        // glVertex3d(-g_areaConfig.scanW/2.0, 0.3, g_areaConfig.scanH/2.0);
        // glEnd();
        // glPopMatrix();
        
        //LED光源位置 //緑
        glDisable(GL_LIGHTING);
        glColor4d(0.0, 1.0, 0.0, 1.0);
        glPushMatrix();
        glTranslated(lightPos0.x, lightPos0.y, lightPos0.z);
        glScaled(2.5, 2.5, 2.5);
        glutSolidSphere(1.0, 36, 18);
        glPopMatrix();
        
        //LED光源衝突位置
        glDisable(GL_LIGHTING);
        glColor4d(1.0, 1.0, 0.0, 1.0);
        glPushMatrix();
        glTranslated(lightCollPos.x, lightCollPos.y, lightCollPos.z);
        glScaled(2.5, 2.5, 2.5);
        glutSolidSphere(1.0, 36, 18);
        glPopMatrix();
        
        //マウスのタッチ位置
        glColor4d(1.0, 0.0, 0.0, 1.0);
        glPushMatrix();
        glTranslated(touchPos.x, 0.0, touchPos.z);
        glScaled(2.5, 2.5, 2.5);
        glutSolidSphere(1.0, 36, 18);
        glPopMatrix();

        glColor4d(1.0, 0.0, 1.0, 1.0);
        glPushMatrix();
        glTranslated(touchPos0.x, 0.0, touchPos0.z);
        glScaled(2.5, 2.5, 2.5);
        glutSolidSphere(1.0, 36, 18);
        glPopMatrix();

        glColor4d(1.0, 0.0, 0.0, 1.0);
        for (int i=0; i<footNum; i++) {
            glPushMatrix();
            glTranslated(footPoint[i].x, 0.0, footPoint[i].z);
            glScaled(2.5, 2.5, 2.5);
            glutSolidSphere(1.0, 36, 18);
            glPopMatrix();
        }
    }
}

//ディスプレイコールバック関数
void display0()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);  //画面消去
    
    //投影変換の設定
    glMatrixMode(GL_PROJECTION);  //変換行列の指定（設定対象は投影変換行列）
    glLoadIdentity();  //行列初期化
    gluPerspective(120.0, (double)g_winInfo[0].W / (double)g_winInfo[0].H, 50.0, 1000.0);  //透視投影ビューボリューム設定
    
    //モデルビュー変換の設定
    glMatrixMode(GL_MODELVIEW);  //変換行列の指定（設定対象はモデルビュー変換行列）
    glLoadIdentity();  //行列初期化
    gluLookAt(lightPos0.x, lightPos0.y, lightPos0.z, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);  //視点視線設定（視野変換行列を乗算）
    
    scene();  //ユーザび知覚させたいオブジェクトの描画
    
    if (g_appConfig.dispMode) {
        glMatrixMode(GL_PROJECTION);  //変換行列の指定（設定対象は投影変換行列）
        glLoadIdentity();  //行列初期化
        glMatrixMode(GL_MODELVIEW);  //変換行列の指定（設定対象はモデルビュー変換行列）
        glLoadIdentity();  //行列初期化
        glDisable(GL_LIGHTING);
        glColor4d(1.0, 1.0, 1.0, 1.0);
        glLineWidth(2.0);
        glBegin(GL_LINE_LOOP);
        glVertex3d(-0.99, 0.99, 0.99);
        glVertex3d(-0.99, -0.99, 0.99);
        glVertex3d(0.99, -0.99, 0.99);
        glVertex3d(0.99, 0.99, 0.99);
        glEnd();
        glLineWidth(1.0);
    }

    //描画したシーンを画像としてframeImageに格納
    glReadPixels(0, 0, g_winInfo[0].W * g_appConfig.renderScale, g_winInfo[0].H * g_appConfig.renderScale, GL_BGR, GL_UNSIGNED_BYTE, frameImage0.data);
    cv::resize(frameImage0, frameImage, frameImage.size());
    cv::flip(frameImage, frameImage, 0);
    
    //アルファチャンネル付き
    for (int j=0; j<TEX_SIZE; j++) {
        for (int i=0; i<TEX_SIZE; i++) {
            cv::Vec3b s;
            cv::Vec4b s1;

            s = frameImage.at<cv::Vec3b>(j, i);
            s1[0] = s[0]; s1[1] = s[1]; s1[2] = s[2];
            if (s[0]<1 && s[1]>254 && s[2]<1) {
                s1[3] = 0;
            }
            else {
                s1[3] = 255;
            }
            frameImage1.at<cv::Vec4b>(j, i) = s1;
        }
    }
    
    //cv::imshow("frame", frameImage);
    
    glutSwapBuffers();  //描画実行
}

//ディスプレイコールバック関数
void display1()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);  //画面消去
    
    //視点座標の計算
    Vec_3D e;
    e.x = g_Cam[1].dist*cos(g_Cam[1].degX*M_PI/180.0)*sin(g_Cam[1].degY*M_PI/180.0);
    e.y = g_Cam[1].dist*sin(g_Cam[1].degX*M_PI/180.0);
    e.z = g_Cam[1].dist*cos(g_Cam[1].degX*M_PI/180.0)*cos(g_Cam[1].degY*M_PI/180.0);
    
    //モデルビュー変換の設定
    glMatrixMode(GL_MODELVIEW);  //変換行列の指定（設定対象はモデルビュー変換行列）
    glLoadIdentity();  //行列初期化
    if (g_Cam[1].degX>90.0) {
        gluLookAt(e.x, e.y, e.z, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0);  //視点視線設定（視野変換行列を乗算）
    }
    else {
        gluLookAt(e.x, e.y, e.z, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);  //視点視線設定（視野変換行列を乗算）
    }
    
    if (g_appConfig.dispMode) {
        scene();  //ユーザび知覚させたいオブジェクトの描画
    }
    
    //======================================================================================================================
    //テクスチャ自動生成を有効化
    glEnable(GL_TEXTURE_GEN_S);
    glEnable(GL_TEXTURE_GEN_T);
    glEnable(GL_TEXTURE_GEN_R);
    glEnable(GL_TEXTURE_GEN_Q);
    
    //視点座標に基づくテクスチャ座標を自動生成の設定
    glTexGendv(GL_S, GL_EYE_PLANE, genfunc[0]);
    glTexGendv(GL_T, GL_EYE_PLANE, genfunc[1]);
    glTexGendv(GL_R, GL_EYE_PLANE, genfunc[2]);
    glTexGendv(GL_Q, GL_EYE_PLANE, genfunc[3]);

    //display0の映像視点に基づくテクスチャ座標の自動生成
    glMatrixMode(GL_TEXTURE);  //変換行列の指定（テクスチャ行列）
    glLoadIdentity();  //行列初期化
    glTranslated(0.5, 0.5, 0.5);  //テクスチャ座標と同じ0~1になるように0.5だけ平行移動
    glScaled(0.5, 0.5, 0.5);  //視点座標系は-1~1なので，-0.5~0.5になるように縮小
    glRotated(180.0, 1.0, 0.0, 0.0);  //上下反転
    gluPerspective(120.0, (GLdouble)g_winInfo[0].W / (GLdouble)g_winInfo[0].H, 50.0, 1000.0);  //display0と同じ投影変換
    gluLookAt(lightPos0.x, lightPos0.y, lightPos0.z, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);  //display0と同じ視点
    
    //自動生成されたテクスチャ座標を用いて，床面に相当する平面にdisplay0で生成した映像をテクスチャマッピング
    glEnable(GL_TEXTURE_2D);  //テクスチャ有効化
    glBindTexture(GL_TEXTURE_2D, 0);  //テクスチャ0を指定
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, TEX_SIZE, TEX_SIZE, 0, GL_BGRA, GL_UNSIGNED_BYTE, frameImage1.data);  //display0で生成した映像をテクスチャ画像として登録
    glMatrixMode(GL_MODELVIEW);  //変換行列の指定（設定対象はモデルビュー変換行列）
    //床面相当の平面を描画
    glDisable(GL_LIGHTING);
    glColor4d(0.0, 0.0, 0.0, 1.0);
    glPushMatrix();
    glTranslated(0.0, 0.0, 0.0);
    glScaled(1000.0, 1.0, 1000.0);
    glBegin(GL_QUADS);
    glVertex3d(-0.5, 0.0, 0.5);
    glVertex3d(0.5, 0.0, 0.5);
    glVertex3d(0.5, 0.0, -0.5);
    glVertex3d(-0.5, 0.0, -0.5);
    glEnd();
    glPopMatrix();
    
    //単純な白い床を描画
    glDisable(GL_TEXTURE_2D);  //テクスチャ有効化
    glDisable(GL_LIGHTING);
    glColor4d(1.0, 1.0, 1.0, 1.0);
    glPushMatrix();
    glTranslated(0.0, -1.0, 0.0);
    glScaled(g_areaConfig.scanW*3.0, 1.0, g_areaConfig.scanH*3.0);
    glBegin(GL_QUADS);
    glVertex3d(-0.5, 0.0, 0.5);
    glVertex3d(0.5, 0.0, 0.5);
    glVertex3d(0.5, 0.0, -0.5);
    glVertex3d(-0.5, 0.0, -0.5);
    glEnd();
    glPopMatrix();

    //テクスチャ自動生成を無効化
    glDisable(GL_TEXTURE_GEN_S);
    glDisable(GL_TEXTURE_GEN_T);
    glDisable(GL_TEXTURE_GEN_R);
    glDisable(GL_TEXTURE_GEN_Q);

    //自動生成されたテクスチャ座標をリセット
    glMatrixMode(GL_TEXTURE);  //変換行列の指定（テクスチャ行列）
    glLoadIdentity();  //行列初期化
    

    //======================================================================================================================

    glutSwapBuffers();  //描画実行
    
    //描画したシーンを画像としてframeImageに格納
    glReadPixels(0, 0, g_winInfo[1].W * g_appConfig.renderScale, g_winInfo[1].H *g_appConfig.renderScale, GL_BGR, GL_UNSIGNED_BYTE, shadowAreaImage.data);
    cv::flip(shadowAreaImage, shadowAreaImage, 0);
    cv::cvtColor(shadowAreaImage, shadowAreaGrayImage, cv::COLOR_BGR2GRAY);
    //cv::imshow("shadowAreaImage", shadowAreaGrayImage);
}

//ディスプレイコールバック関数
void display2()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);  //画面消去
        
    //モデルビュー変換の設定
    glMatrixMode(GL_MODELVIEW);  //変換行列の指定（設定対象はモデルビュー変換行列）
    glLoadIdentity();  //行列初期化
    
    //照明
    glColor4d(1.0, 2*lightPos0.y/g_areaConfig.lightH, 2*lightPos0.y/g_areaConfig.lightH, 1.0);
    glPushMatrix();
    glTranslated(-lightPos0.x, lightPos0.y, 0.0);
    glScaled(11.0, 11.0, 1.0);
    glutSolidSphere(1.0, 36, 18);
    glPopMatrix();

    glutSwapBuffers();  //描画実行
}

//リシェイプコールバック関数
void reshape0(int w, int h)
{
    int wID = 0;
    
    g_winInfo[wID].W = w; g_winInfo[wID].H = h;  //ウィンドウサイズ格納
    glViewport(0, 0, g_winInfo[wID].W, g_winInfo[wID].H);  //ビューポート設定
    frameImage0 = cv::Mat(cv::Size(g_winInfo[wID].W, g_winInfo[wID].H), CV_8UC3);
}

//リシェイプコールバック関数
void reshape1(int w, int h)
{
    int wID = 1;
    
    g_winInfo[wID].W = w; g_winInfo[wID].H = h;  //ウィンドウサイズ格納
    
    glViewport(0, 0, g_winInfo[wID].W, g_winInfo[wID].H);  //ビューポート設定
    
    //投影変換の設定
    glMatrixMode(GL_PROJECTION);  //変換行列の指定（設定対象は投影変換行列）
    glLoadIdentity();  //行列初期化
    //gluPerspective(40.0, (double)winW[wID]/(double)winH[wID], 50.0, 1000.0);  //透視投影ビューボリューム設定
    glFrustum(-g_areaConfig.scanW*0.5*0.1, g_areaConfig.scanW*0.5*0.1, -g_areaConfig.scanH*0.5*0.1, g_areaConfig.scanH*0.5*0.1, g_Cam[wID].dist*0.1, 1000.0);

    g_winInfo[wID].W = w; g_winInfo[wID].H = h;  //ウィンドウサイズ格納
    shadowAreaImage = cv::Mat(cv::Size(g_winInfo[wID].W, g_winInfo[wID].H), CV_8UC3);
}

//リシェイプコールバック関数
void reshape2(int w, int h)
{
    int wID = 2;
    
    g_winInfo[wID].W = w; g_winInfo[wID].H = h;  //ウィンドウサイズ格納
    
    glViewport(0, 0, g_winInfo[wID].W, g_winInfo[wID].H);  //ビューポート設定
    
    //投影変換の設定
    glMatrixMode(GL_PROJECTION);  //変換行列の指定（設定対象は投影変換行列）
    glLoadIdentity();  //行列初期化
    gluOrtho2D(-g_areaConfig.lightW*0.5, g_areaConfig.lightW*0.5, 0.0, g_areaConfig.lightH);
    
    g_winInfo[wID].W = w; g_winInfo[wID].H = h;  //ウィンドウサイズ格納
}

//マウスクリックコールバック関数
void mouse0(int button, int state, int x, int y)
{
    int wID = 0;
    
    //マウスボタンが押された
    if (state==GLUT_DOWN) {
        g_winInfo[wID].mX = x; g_winInfo[wID].mY = y; g_winInfo[wID].mState = state; g_winInfo[wID].mButton = button;  //マウス情報保持
    }
}

//マウスクリックコールバック関数
void mouse1(int button, int state, int x, int y)
{
    int wID = 1;
    
    //マウスボタンが押された
    if (state==GLUT_DOWN) {
        g_winInfo[wID].mX = x; g_winInfo[wID].mY = y; g_winInfo[wID].mState = state; g_winInfo[wID].mButton = button;  //マウス情報保持
        
        if (button==GLUT_LEFT_BUTTON) {
            //タッチ位置
            Vec_3D touchPosX;
            touchPosX.x = (g_winInfo[wID].mX-g_areaConfig.scanW*g_areaConfig.resolution*0.5)/g_areaConfig.resolution; 
            touchPosX.y = 0.0; 
            touchPosX.z = (g_winInfo[wID].mY-g_areaConfig.scanH*g_areaConfig.resolution*0.5)/g_areaConfig.resolution;
            double len = sqrt(pow(touchPos.x-touchPosX.x,2)+pow(touchPos.z-touchPosX.z,2));
            touchPos = touchPosX;
            //printf("touchPos = (%f, %f, %f)\n", touchPos.x, touchPos.y, touchPos.z);
            
            if (shadowVal==0 && len<10.0) {  //影を動かす
                lightVec.x = lightCollPos.x-touchPos.x;
                lightVec.y = lightCollPos.y-touchPos.y;
                lightVec.z = lightCollPos.z-touchPos.z;
                lightVec = vectorNormalize(lightVec);
                
                double t = (lightPos0.z-touchPos.z)/lightVec.z;
                lightPos0.x = touchPos.x+lightVec.x*t;
                lightPos0.y = touchPos.y+lightVec.y*t;
            }
            else {
                //タッチ位置シャドウ画像画素値
                shadowVal = shadowAreaGrayImage.at<unsigned char>(g_winInfo[wID].mY, g_winInfo[wID].mX);
                //printf("shadowVal = %d\n", shadowVal);
                
                //
                lightVec.x = lightPos0.x-touchPos.x; lightVec.y = lightPos0.y-touchPos.y; lightVec.z = lightPos0.z-touchPos.z;
                lightVec = vectorNormalize(lightVec);
                //printf("lightVec = (%f, %f, %f)\n", lightVec.x, lightVec.y, lightVec.z);
                double t = (objPos.z-touchPos.z)/lightVec.z;
                lightCollPos.x = touchPos.x+lightVec.x*t;
                lightCollPos.y = touchPos.y+lightVec.y*t;
            }
        }
    }
}

//マウスドラッグコールバック関数
void motion0(int x, int y)
{
    int wID = 0;

    if (g_winInfo[wID].mButton==GLUT_RIGHT_BUTTON) {
        //マウスの移動量を角度変化量に変換
        lightPos0.x += (g_winInfo[wID].mX-x)*0.5;  //マウス横方向→水平角
        lightPos0.y += (y-g_winInfo[wID].mY)*0.5;  //マウス縦方向→垂直角
    }
    
    //マウス座標をグローバル変数に保存
    g_winInfo[wID].mX = x; g_winInfo[wID].mY = y;
}

//マウスドラッグコールバック関数
void motion1(int x, int y)
{
    int wID = 1;
    
    if (g_winInfo[wID].mButton==GLUT_RIGHT_BUTTON) {//回転
        //マウスの移動量を角度変化量に変換
        g_Cam[wID].degY = g_Cam[wID].degY+(g_winInfo[wID].mX-x)*0.5;  //マウス横方向→水平角
        g_Cam[wID].degX = g_Cam[wID].degX+(y-g_winInfo[wID].mY)*0.5;  //マウス縦方向→垂直角
    }
    else if (g_winInfo[wID].mButton==GLUT_LEFT_BUTTON && shadowVal==0) {
        //タッチ位置
        touchPos.x = (g_winInfo[wID].mX-g_areaConfig.scanW*g_areaConfig.resolution*0.5)/g_areaConfig.resolution; 
        touchPos.y = 0.0; 
        touchPos.z = (g_winInfo[wID].mY-g_areaConfig.scanH*g_areaConfig.resolution*0.5)/g_areaConfig.resolution;
        
        lightVec.x = lightCollPos.x-touchPos.x;
        lightVec.y = lightCollPos.y-touchPos.y;
        lightVec.z = lightCollPos.z-touchPos.z;
        lightVec = vectorNormalize(lightVec);

        double t = (lightPos0.z-touchPos.z)/lightVec.z;
        lightPos0.x = touchPos.x+lightVec.x*t;
        lightPos0.y = touchPos.y+lightVec.y*t;
        //printf("光源位置: x = %f, y = %f\n", lightPos0.x, lightPos0.y);
    }

    
    //マウス座標をグローバル変数に保存
    g_winInfo[wID].mX = x; g_winInfo[wID].mY = y;
}

//キーボードコールバック関数
void keyboard(unsigned char key, int x, int y)
{
    switch (key) {
        case 27:  //[ESC]
            exit(0);
            
        case 'd':  //dispMode切り替え
            g_appConfig.dispMode = 1-g_appConfig.dispMode;
            break;
            
        case 'r':
            g_Cam[1].degX = 90.0;
            g_Cam[1].degY = 0.0;
            break;

        case 't':

         lightPos0.x = 0.0; 
         lightPos0.y = g_areaConfig.lightH/2.0; 
         lightPos0.z =  g_areaConfig.scanH/2.0 + g_object.scaleZ + 100.0;
            break;

        case '1'://立方体
            g_appConfig.objectFlg = 1;
            break;

        case '2'://立方体と球体
            g_appConfig.objectFlg = 2;
            break;

        case '0'://テクスチャ
            g_appConfig.objectFlg = 0;
            break;

        case 'f':
            glutFullScreen();
            break;
            
        default:
            break;
    }
}

//timer0内でLiDARデータを読み込み，影を動かす計算を行い，新しい光源の位置を決める
void updateLidarInteraction(){
    FILE *fp = fopen("../LIDAR1b/footpoint.txt", "r");
    if(fp == NULL) return;

    int tmpNum;
    double tmpX,tmpZ;
    if(fscanf(fp,"%d",&tmpNum) == 1 && fscanf(fp, "%lf,%lf", &tmpX, &tmpZ) == 2){
        //座標変換(cm -> mm)
        touchPos0.x = -(tmpX * 10.0) ;
        touchPos0.y = 0.0;
        touchPos0.z  = (tmpZ * 10.0) - g_areaConfig.scanH/2.0;

        printf("touchPos0: (%f, %f, %f)\n", touchPos0.x, touchPos0.y, touchPos0.z);

        //画面上の座標に変換(mm → px)
        double imgX = (touchPos0.x + g_areaConfig.scanW/2.0)*g_areaConfig.resolution;
        double imgY = (touchPos0.z + g_areaConfig.scanH/2.0)*g_areaConfig.resolution;

        
        printf("img X: %f, img Y: %f\n",imgX,imgY);



        if(imgX >= 0 && imgX < shadowAreaGrayImage.cols && imgY >= 0 && imgY < shadowAreaGrayImage.rows){

            unsigned char shadowVal0 = shadowAreaGrayImage.at<unsigned char>(imgY, imgX);//色の確認，触った座標の色
            printf("shadowVal0: %d\n",shadowVal0);
            //imshow("shadowAreaGrayImage", shadowAreaGrayImage);
            if(shadowVal0 == 0){ //影の中にいる場合
                if(chaseFlg == 0){
                    //追跡開始
                    chaseFlg =1;
                    touchPos = touchPos0;
                } else {
                    double len = sqrt(pow(touchPos.x - touchPos0.x, 2) + pow(touchPos.z - touchPos0.z,2));//距離計算
                    if(len < g_interaction.dist){//距離が一定値未満の場合
                        touchPos = touchPos0;//座標更新
                    }
                }

                //光源位置の計算
                lightVec.x = lightCollPos.x - touchPos.x;
                lightVec.y = lightCollPos.y - touchPos.y;
                lightVec.z = lightCollPos.z - touchPos.z;
                lightVec = vectorNormalize(lightVec);

                double t = (lightPos0.z-touchPos.z)/lightVec.z;
                lightPos0.x = touchPos.x+lightVec.x*t;
                lightPos0.y = touchPos.y+lightVec.y*t;

                printf("光源位置: x = %f, y = %f\n", lightPos0.x, lightPos0.y);

            }else{
                chaseFlg = 0;//待機
            }
        }


    }
    fclose(fp);

}

// timer0内でシリアル通信の送信処理
void sendLightPosToSerial(){
    if (g_Serial.fd == -1)return;
    
    char sendBuf[64];

    //LEDパネルの座標系に変換64*32
    double serial_x = mapRange(lightPos0.x, -g_areaConfig.lightW/2.0, g_areaConfig.lightW/2.0, 0.0, g_areaConfig.LEDW);
    double serial_y = mapRange(lightPos0.y, 0.0, g_areaConfig.lightH, 0.0, g_areaConfig.LEDH);

    int len = snprintf(sendBuf, sizeof(sendBuf), "%d,%d\n", (int)serial_x, (int)serial_y);

    write(g_Serial.fd, sendBuf, len);
    //printf("送信:%s\n", sendBuf);
}

//タイマーコールバック関数
void timer0(int value)
{
    // 1. 再描画の予約（全ウィンドウ）
    for (int i = 0; i < 3; i++) {
        glutSetWindow(g_winInfo[i].id);
        glutPostRedisplay();
    }

    // 2. LIDARデータの処理と影の追従
    updateLidarInteraction();

    // 3. シリアル通信で光源位置を送信
    sendLightPosToSerial();

    // 4. 次回のタイマー設定
    glutTimerFunc(1000 / g_appConfig.frameRate, timer0, 0);
}

//ベクトルの外積計算
Vec_3D normcrossprod(Vec_3D v1, Vec_3D v2)
{
    Vec_3D out;
    
    out.x = v1.y*v2.z-v1.z*v2.y;
    out.y = v1.z*v2.x-v1.x*v2.z;
    out.z = v1.x*v2.y-v1.y*v2.x;
    
    return vectorNormalize(out);
}

//ベクトル正規化
Vec_3D vectorNormalize(Vec_3D vec)
{
    Vec_3D out;
    double length;
    
    //ベクトル長
    length = sqrt(vec.x*vec.x+vec.y*vec.y+vec.z*vec.z);
    //正規化
    out.x = vec.x/length;
    out.y = vec.y/length;
    out.z = vec.z/length;
    
    return out;
}

//テクスチャ付き格子構造平面描画（両面）
void glMyTexPlane(int texID, int w, int h)
{
    double dX = 1.0/w, dY = 1.0/h;  //格子間隔
    Vec_3D p0, p1, p2, p3;  //各格子頂点

    if (texID>=0) {
        glEnable(GL_TEXTURE_2D);  //テクスチャ有効化
        glBindTexture(GL_TEXTURE_2D, texID);  //テクスチャオブジェクト
    }
    
    glBegin(GL_QUADS);  //物体配置終了
    
    //表
    glNormal3d(0.0, 0.0, 1.0);  //法線ベクトル設定
    p0.z = p1.z = p2.z = p3.z = 0.05;
    for (int j=0; j<h; j++) {
        //位置y座標
        p0.y = -0.5+j*dY;
        p1.y = p0.y;
        p2.y = p3.y = p0.y+dY;
        //テクスチャt座標
        p0.t = 1.0-j*dY;
        p1.t = p0.t;
        p2.t = p3.t = p0.t-dY;

        for (int i=0; i<w; i++) {
            //位置x座標
            p0.x = -0.5+dX*i;
            p1.x = p2.x = p0.x+dX;
            p3.x = p0.x;
            //テクスチャs座標
            p0.s = dX*i;
            p1.s = p2.s = p0.s+dX;
            p3.s = p0.s;
            //テクスチャ座標＆頂点座標指定
            glTexCoord2d(p0.s, p0.t);
            glVertex3d(p0.x, p0.y, p0.z);
            glTexCoord2d(p1.s, p1.t);
            glVertex3d(p1.x, p1.y, p1.z);
            glTexCoord2d(p2.s, p2.t);
            glVertex3d(p2.x, p2.y, p2.z);
            glTexCoord2d(p3.s, p3.t);
            glVertex3d(p3.x, p3.y, p3.z);
        }
    }
    
    //裏
    glNormal3d(0.0, 0.0, -1.0);  //法線ベクトル設定
    p0.z = p1.z = p2.z = p3.z = -0.05;
    for (int j=0; j<h; j++) {
        //位置y座標
        p0.y = -0.5+j*dY;
        p1.y = p0.y;
        p2.y = p3.y = p0.y+dY;
        //テクスチャt座標
        p0.t = 1.0-j*dY;
        p1.t = p0.t;
        p2.t = p3.t = p0.t-dY;
        
        for (int i=0; i<w; i++) {
            //位置x座標
            p0.x = -0.5+dX*i;
            p1.x = p2.x = p0.x+dX;
            p3.x = p0.x;
            //テクスチャs座標
            p0.s = dX*i;
            p1.s = p2.s = p0.s+dX;
            p3.s = p0.s;
            //テクスチャ座標＆頂点座標指定
            glTexCoord2d(p0.s, p0.t);
            glVertex3d(p0.x, p0.y, p0.z);
            glTexCoord2d(p1.s, p1.t);
            glVertex3d(p1.x, p1.y, p1.z);
            glTexCoord2d(p2.s, p2.t);
            glVertex3d(p2.x, p2.y, p2.z);
            glTexCoord2d(p3.s, p3.t);
            glVertex3d(p3.x, p3.y, p3.z);
        }
    }

    glEnd();  //配置終了
    
    glDisable(GL_TEXTURE_2D);  //テクスチャ無効化
}


