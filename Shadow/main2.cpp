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

//グローバル変数
int winID[3];  //ウィンドウID
double eDist[2], eDegX[2], eDegY[2];  //視点極座標
int mX[2], mY[2], mState[2], mButton[2];  //マウス座標
int winW[3], winH[3];  //ウィンドウサイズ
double fr = 30.0;
Vec_3D objPos;  //影物体
Vec_3D lightPos0;  //LED光源座標
int dispMode = 0;  //表示モード
double scanW = 400.0, scanH = 360.0;  //スキャンエリア
double aspectRate = 1.0/2.0;
double lightW = 256.0, lightH = lightW*aspectRate;  //照明エリア（2:1としている）
double reso = 2.0;  //
Vec_3D touchPos, touchPos0;  //手を触れた場所
unsigned char shadowVal = 255;  //影かどうか（0は影）
Vec_3D lightVec;  //光線ベクトル
Vec_3D lightCollPos;  //光線が物体と当たる場所
Vec_3D footPoint[POINTMAX];
int footNum;
int chaseFlg = 0;
int ObjectFlg = 1;

cv::Mat frameImage0, frameImage, frameImage1, shadowAreaImage, shadowAreaGrayImage;

// シリアル通信用
int serialFD = -1;
const char* SERIAL_PORT = "/dev/cu.usbserial-10"; // 自動検出したポート
const int BAUD_RATE = 115200;

//テクスチャ生成関数のパラメータ（）
static double genfunc[][4] = {
    {1.0, 0.0, 0.0, 0.0},
    {0.0, 1.0, 0.0, 0.0},
    {0.0, 0.0, 1.0, 0.0},
    {0.0, 0.0, 0.0, 1.0},
};

double rDisp = 1.0;

//メイン関数
int main(int argc, char *argv[])
{
    glutInit(&argc, argv);  //OpenGL/GLUTの初期化
    initGL();  //初期設定

     // シリアルポート初期化
    serialFD = initSerial(SERIAL_PORT);
    if (serialFD == -1) {
        printf("ポートが開けません: %s\n", SERIAL_PORT);
    } else {
        printf("ポート開きます！: %s\n", SERIAL_PORT);
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
    winID[0] = glutCreateWindow("CG0");  //ウィンドウの生成
    //コールバック関数の指定
    glutDisplayFunc(display0);  //ディスプレイコールバック関数の指定
    glutReshapeFunc(reshape0);  //リシェイプコールバック関数の指定
    glutMouseFunc(mouse0);  //マウスクリックコールバック関数の指定
    glutMotionFunc(motion0);  //マウスドラッグコールバック関数の指定
    glutTimerFunc(1000/fr, timer0, 0);  //タイマー
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
    glutInitWindowSize(scanW*reso, scanH*reso);  //ウィンドウサイズの指定
    glutInitWindowPosition(512, 0);  //ウィンドウ位置の指定
    winID[1] = glutCreateWindow("CG1");  //ウィンドウの生成
    //コールバック関数の指定
    glutDisplayFunc(display1);  //ディスプレイコールバック関数の指定
    glutReshapeFunc(reshape1);  //リシェイプコールバック関数の指定
    glutMouseFunc(mouse1);  //マウスクリックコールバック関数の指定
    glutMotionFunc(motion1);  //マウスドラッグコールバック関数の指定
    glutKeyboardFunc(keyboard);  //キーボードコールバック関数の指定
    //各種設定
    glClearColor(0.0, 0.2, 0.2, 1.0);  //ウィンドウクリア色の指定（RGBA）
    glEnable(GL_DEPTH_TEST);  //デプスバッファの有効化
    glEnable(GL_NORMALIZE);  //ベクトル正規化有効化
    glEnable(GL_BLEND);  //アルファチャンネル有効化
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);    //アルファ関数の設定
    glEnable(GL_ALPHA_TEST);  //アルファテスト有効化
    glAlphaFunc(GL_GREATER, 0.1);  //アルファ値比較関数の設定
    //視点関係
    eDist[1] = 250.0; eDegX[1] = 90.0; eDegY[1] = 0.0;  //視点極座標

    //描画ウィンドウ生成2（照明用映像の生成）
    glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);  //ディスプレイモードの指定
    glutInitWindowSize(800, 800*aspectRate);  //ウィンドウサイズの指定
    glutInitWindowPosition(512, scanH*reso+100);  //ウィンドウ位置の指定
    winID[2] = glutCreateWindow("CG2");  //ウィンドウの生成
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
        glutSetWindow(winID[i]);  //設定対象ウィンドウ

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
    objPos.u = 50.0;  //横幅
    objPos.x = 0.0; 
    objPos.y = objPos.u*objPos.t/objPos.s*0.5; 
    objPos.z = 100.0;  //位置
    
    //光源位置
    lightPos0.x = 0.0; lightPos0.y = lightH/2.0; lightPos0.z = scanH/2.0;
    lightCollPos.z = objPos.z;  //光源と物体との衝突位置
}

//ユーザに知覚させたいオブジェクトの描画
void scene()
{
    if(ObjectFlg == 0) {
        //オブジェクト（本体）
        glDisable(GL_LIGHTING);
        glColor4d(1.0, 1.0, 1.0, 1.0);
        glPushMatrix();
        glTranslated(objPos.x, objPos.y, objPos.z);
        glScaled(objPos.u, objPos.u*objPos.t/objPos.s, 1.0);
        glMyTexPlane(objPos.id, objPos.s/50, objPos.t/50);
        glPopMatrix();
    }
    
    if (ObjectFlg == 1) {//立方体

        glDisable(GL_LIGHTING);
        glColor4d(1.0, 0.0, 0.0, 1.0);
        glPushMatrix();
        glTranslated(0.0, 15.0, objPos.z);
        glScaled(30.0, 30.0, 30.0);
       glutSolidCube(1.0);
        glPopMatrix();
       
    }
    
    if (ObjectFlg == 2) {//立方体と球体
        glDisable(GL_LIGHTING);
        glColor4d(1.0, 0.0, 0.0, 1.0);
        glPushMatrix();
        glTranslated(40.0, 15.0, objPos.z);
        glScaled(30.0, 30.0, 30.0);
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
    
    if (dispMode) {
        //LED光源位置
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
    gluPerspective(120.0, (double)winW[0]/(double)winH[0], 50.0, 1000.0);  //透視投影ビューボリューム設定
    
    //モデルビュー変換の設定
    glMatrixMode(GL_MODELVIEW);  //変換行列の指定（設定対象はモデルビュー変換行列）
    glLoadIdentity();  //行列初期化
    gluLookAt(lightPos0.x, lightPos0.y, lightPos0.z, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);  //視点視線設定（視野変換行列を乗算）
    
    scene();  //ユーザび知覚させたいオブジェクトの描画
    
    if (dispMode) {
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
    glReadPixels(0, 0, winW[0]*rDisp, winH[0]*rDisp, GL_BGR, GL_UNSIGNED_BYTE, frameImage0.data);
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
    e.x = eDist[1]*cos(eDegX[1]*M_PI/180.0)*sin(eDegY[1]*M_PI/180.0);
    e.y = eDist[1]*sin(eDegX[1]*M_PI/180.0);
    e.z = eDist[1]*cos(eDegX[1]*M_PI/180.0)*cos(eDegY[1]*M_PI/180.0);
    
    //モデルビュー変換の設定
    glMatrixMode(GL_MODELVIEW);  //変換行列の指定（設定対象はモデルビュー変換行列）
    glLoadIdentity();  //行列初期化
    if (eDegX[1]>90.0) {
        gluLookAt(e.x, e.y, e.z, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0);  //視点視線設定（視野変換行列を乗算）
    }
    else {
        gluLookAt(e.x, e.y, e.z, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);  //視点視線設定（視野変換行列を乗算）
    }
    
    if (dispMode) {
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
    gluPerspective(120.0, (GLdouble)winW[0]/(GLdouble)winH[0], 50.0, 1000.0);  //display0と同じ投影変換
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
    glScaled(scanW*1.01, 1.0, scanH*1.01);
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
    glReadPixels(0, 0, winW[1]*rDisp, winH[1]*rDisp, GL_BGR, GL_UNSIGNED_BYTE, shadowAreaImage.data);
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
    glColor4d(1.0, 2*lightPos0.y/lightH, 2*lightPos0.y/lightH, 1.0);
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
    
    winW[wID] = w; winH[wID] = h;  //ウィンドウサイズ格納
    glViewport(0, 0, winW[wID], winH[wID]);  //ビューポート設定
    frameImage0 = cv::Mat(cv::Size(winW[wID], winH[wID]), CV_8UC3);
}

//リシェイプコールバック関数
void reshape1(int w, int h)
{
    int wID = 1;
    
    winW[wID] = w; winH[wID] = h;  //ウィンドウサイズ格納
    
    glViewport(0, 0, winW[wID], winH[wID]);  //ビューポート設定
    
    //投影変換の設定
    glMatrixMode(GL_PROJECTION);  //変換行列の指定（設定対象は投影変換行列）
    glLoadIdentity();  //行列初期化
    //gluPerspective(40.0, (double)winW[wID]/(double)winH[wID], 50.0, 1000.0);  //透視投影ビューボリューム設定
    glFrustum(-scanW*0.5*0.1, scanW*0.5*0.1, -scanH*0.5*0.1, scanH*0.5*0.1, eDist[wID]*0.1, 1000.0);

    winW[wID] = w; winH[wID] = h;  //ウィンドウサイズ格納
    shadowAreaImage = cv::Mat(cv::Size(winW[wID], winH[wID]), CV_8UC3);
}

//リシェイプコールバック関数
void reshape2(int w, int h)
{
    int wID = 2;
    
    winW[wID] = w; winH[wID] = h;  //ウィンドウサイズ格納
    
    glViewport(0, 0, winW[wID], winH[wID]);  //ビューポート設定
    
    //投影変換の設定
    glMatrixMode(GL_PROJECTION);  //変換行列の指定（設定対象は投影変換行列）
    glLoadIdentity();  //行列初期化
    gluOrtho2D(-lightW*0.5, lightW*0.5, 0.0, lightH);
    
    winW[wID] = w; winH[wID] = h;  //ウィンドウサイズ格納
}

//マウスクリックコールバック関数
void mouse0(int button, int state, int x, int y)
{
    int wID = 0;
    
    //マウスボタンが押された
    if (state==GLUT_DOWN) {
        mX[wID] = x; mY[wID] = y; mState[wID] = state; mButton[wID] = button;  //マウス情報保持
    }
}

//マウスクリックコールバック関数
void mouse1(int button, int state, int x, int y)
{
    int wID = 1;
    
    //マウスボタンが押された
    if (state==GLUT_DOWN) {
        mX[wID] = x; mY[wID] = y; mState[wID] = state; mButton[wID] = button;  //マウス情報保持
        
        if (button==GLUT_LEFT_BUTTON) {
            //タッチ位置
            Vec_3D touchPosX;
            touchPosX.x = (mX[wID]-scanW*reso*0.5)/reso; touchPosX.y = 0.0; touchPosX.z = (mY[wID]-scanH*reso*0.5)/reso;
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
                shadowVal = shadowAreaGrayImage.at<unsigned char>(mY[wID], mX[wID]);
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

    if (mButton[wID]==GLUT_RIGHT_BUTTON) {
        //マウスの移動量を角度変化量に変換
        lightPos0.x += (mX[wID]-x)*0.5;  //マウス横方向→水平角
        lightPos0.y += (y-mY[wID])*0.5;  //マウス縦方向→垂直角
    }
    
    //マウス座標をグローバル変数に保存
    mX[wID] = x; mY[wID] = y;
}

//マウスドラッグコールバック関数
void motion1(int x, int y)
{
    int wID = 1;
    
    if (mButton[wID]==GLUT_RIGHT_BUTTON) {
        //マウスの移動量を角度変化量に変換
        eDegY[wID] = eDegY[wID]+(mX[wID]-x)*0.5;  //マウス横方向→水平角
        eDegX[wID] = eDegX[wID]+(y-mY[wID])*0.5;  //マウス縦方向→垂直角
    }
    else if (mButton[wID]==GLUT_LEFT_BUTTON && shadowVal==0) {
        //タッチ位置
        touchPos.x = (mX[wID]-scanW*reso*0.5)/reso; touchPos.y = 0.0; touchPos.z = (mY[wID]-scanH*reso*0.5)/reso;
        
        lightVec.x = lightCollPos.x-touchPos.x;
        lightVec.y = lightCollPos.y-touchPos.y;
        lightVec.z = lightCollPos.z-touchPos.z;
        lightVec = vectorNormalize(lightVec);

        double t = (lightPos0.z-touchPos.z)/lightVec.z;
        lightPos0.x = touchPos.x+lightVec.x*t;
        lightPos0.y = touchPos.y+lightVec.y*t;
    }

    
    //マウス座標をグローバル変数に保存
    mX[wID] = x; mY[wID] = y;
}

//キーボードコールバック関数(key:キーの種類，x,y:座標)
void keyboard(unsigned char key, int x, int y)
{
    switch (key) {
        case 27:  //[ESC]
            exit(0);
            
        case 'd':  //dispMode切り替え
            dispMode = 1-dispMode;
            break;
            
        case 'r':
            eDegX[1] = 90.0;
            eDegY[1] = 0.0;
            break;

        case 't':

         lightPos0.x = 0.0; 
         lightPos0.y = lightH/2.0; 
         lightPos0.z = scanH/2.0;
            break;

        case '1'://立方体
            ObjectFlg = 1;
            break;

        case '2'://立方体と球体
            ObjectFlg = 2;
            break;

        case '0'://テクスチャ
            ObjectFlg = 0;
            break;

        case 'f':
            glutFullScreen();
            break;
            
        default:
            break;
    }
}

//タイマーコールバック関数
void timer0(int value)
{
    glutSetWindow(winID[0]);
    glutTimerFunc(1000/fr, timer0, 0);  //タイマー再設定
    glutPostRedisplay();  //ディスプレイイベント強制発生
    
    glutSetWindow(winID[1]);
    glutPostRedisplay();  //ディスプレイイベント強制発生
    
    glutSetWindow(winID[2]);
    glutPostRedisplay();  //ディスプレイイベント強制発生
    
    
    int wID = 1;
    FILE *fp = fopen("../LIDAR1b/footpoint.txt", "r");
    footNum = 0;
    if (fp!=NULL) {
        int tmpNum;
        fscanf(fp, "%d", &tmpNum);
        double tmpX, tmpZ;
        fscanf(fp, "%lf,%lf", &tmpX, &tmpZ);
        fclose(fp);
        //printf("%f, %f\n", tmpX, tmpZ);
        touchPos0.x = tmpZ*10.0-scanW/2.0;
        touchPos0.z = tmpX*10.0;
        touchPos0.y = 0.0;
        //printf("touchPos0 = (%f, %f, %f)\n", touchPos0.x, touchPos0.y, touchPos0.z);

        int imgX, imgY;
        imgX = (touchPos0.x-(-scanW/2.0))/scanW*shadowAreaGrayImage.cols;
        imgY = (touchPos0.z-(-scanH/2.0))/scanH*shadowAreaGrayImage.rows;
        unsigned char shadowVal0 = shadowAreaGrayImage.at<unsigned char>(imgY, imgX);

        if (shadowVal0==0) {
            if (chaseFlg==0) {
                chaseFlg = 1;
                touchPos = touchPos0;
                //
                lightVec.x = lightPos0.x-touchPos.x; lightVec.y = lightPos0.y-touchPos.y; lightVec.z = lightPos0.z-touchPos.z;
                lightVec = vectorNormalize(lightVec);
                //printf("lightVec = (%f, %f, %f)\n", lightVec.x, lightVec.y, lightVec.z);
                double t = (objPos.z-touchPos.z)/lightVec.z;
                lightCollPos.x = touchPos.x+lightVec.x*t;
                lightCollPos.y = touchPos.y+lightVec.y*t;
            }
            else {
                double len = sqrt(pow(touchPos.x-touchPos0.x,2)+pow(touchPos.z-touchPos0.z,2));
                if (len<30.0 && chaseFlg==1) {
                    //printf("Drug\n");
                    touchPos = touchPos0;
                    lightVec.x = lightCollPos.x-touchPos0.x;
                    lightVec.y = lightCollPos.y-touchPos0.y;
                    lightVec.z = lightCollPos.z-touchPos0.z;
                    lightVec = vectorNormalize(lightVec);
                    
                    double t = (lightPos0.z-touchPos0.z)/lightVec.z;
                    lightPos0.x = touchPos0.x+lightVec.x*t;
                    lightPos0.y = touchPos0.y+lightVec.y*t;
                    touchPos = touchPos0;
                    chaseFlg = 1;
                }
            }
        }
        else {
            chaseFlg = 0;
        }
    }

    // シリアル送信
    if (serialFD != -1) {
        char buf[64];
        //LEDパネル用に変換
        double serial_x = (lightPos0.x + lightW/2.0)*64.0/lightW;
        //double serial_y = (lightH - lightPos0.y)*32.0/lightH;
        double serial_y = (lightH - lightPos0.y)*32.0/lightH;
        // double out_min_x = 0.0f;
        // double out_max_x = 64.0f;//64
        // double serial_x = out_min_x + (lightPos0.x - in_min_x) * (out_max_x - out_min_x) / (in_max_x - in_min_x);

        // double in_min_y = -70.0f;//-70
        // double in_max_y = 233.0f ;//233
        // double out_min_y = 32.0f;//32
        // double out_max_y = 0.0f;//0
        // double serial_y = out_min_y + (lightPos0.y - in_min_y) * (out_max_y - out_min_y) / (in_max_y - in_min_y);

        int len = snprintf(buf, sizeof(buf), "%.2f,%.2f\n", serial_x, serial_y);
        write(serialFD, buf, len);
        printf("LightPos: (%.2f, %.2f, %.2f)\n", lightPos0.x, lightPos0.y, lightPos0.z);
        printf("Sent: %s", buf);
    }
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


