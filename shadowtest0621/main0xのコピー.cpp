//g++ -O3 -std=c++11 main0x.cpp -framework OpenGL -framework GLUT `pkg-config --cflags --libs opencv4` -Wno-deprecated
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <GLUT/glut.h>  //OpenGL/GLUTの使用
#include <opencv2/opencv.hpp>  //OpenCV関連ヘッダ

//定数宣言
#define TEX_SIZE 512  //テクスチャサイズ
#define POINTMAX 20
#define KX 640
#define KY 480

//三次元ベクトル構造体
typedef struct _Vec_3D
{
    double x, y, z;
    double s, t, u;
    int id;
} Vec_3D;

//関数名の宣言
void initGL();
void display0();
void display1();
void display2();
void display3();
void reshape0(int w, int h);
void reshape1(int w, int h);
void reshape2(int w, int h);
void reshape3(int w, int h);
void mouse0(int button, int state, int x, int y);
void mouse1(int button, int state, int x, int y);
void mouse2(int button, int state, int x, int y);
void motion0(int x, int y);
void motion1(int x, int y);
void motion2(int x, int y);
void keyboard(unsigned char key, int x, int y);
void timer0(int value);
Vec_3D normcrossprod(Vec_3D v1, Vec_3D v2);
Vec_3D vectorNormalize(Vec_3D v0);
void glMyTexPlane(int texID, int w, int h);
void scene();

//グローバル変数
int winID[4];  //ウィンドウID
double eDist[4], eDegX[4], eDegY[4];  //視点極座標
int mX[4], mY[4], mState[4], mButton[4];  //マウス座標
int winW[4], winH[4];  //ウィンドウサイズ
double fr = 30.0;
Vec_3D objPos;  //影物体
Vec_3D lightPos0;  //LED光源座標
Vec_3D lightPos1;  //LED光源座標
int dispMode = 0;  //表示モード
double scanW = 40.0, scanH = 29.0;  //スキャンエリア
double aspectRate = 9.0/16.0;
double lightW = 40.0, lightH = lightW*aspectRate;  //照明エリア（16:9としている）
double reso = 20.0;  //
Vec_3D touchPos;  //手を触れた場所
Vec_3D touchPosV;  //手を触れた場所
Vec_3D touchPosX0, touchPosX1, touchPosX2;  //手を触れた場所
unsigned char shadowVal = 255;  //影かどうか（0は影）
cv::Vec3b shadowColVal = cv::Vec3b(255,255,255);  //影かどうか（0は影）
Vec_3D lightVec0;  //光線ベクトル
Vec_3D lightVec1;  //光線ベクトル
Vec_3D lightCollPos0;  //光線が物体と当たる場所
Vec_3D lightCollPos1;  //光線が物体と当たる場所
Vec_3D footPoint[POINTMAX];
int footNum;
int chaseFlg0 = 0;
int chaseFlg1 = 0;
int chaseFlg2 = 0;

cv::Mat frameImage0a, frameImage0, frameImage0b;
cv::Mat frameImage1a, frameImage1, frameImage1b;
cv::Mat shadowAreaImage, shadowAreaGrayImage;

//テクスチャ生成関数のパラメータ（）
static double genfunc[][4] = {
    {1.0, 0.0, 0.0, 0.0},
    {0.0, 1.0, 0.0, 0.0},
    {0.0, 0.0, 1.0, 0.0},
    {0.0, 0.0, 0.0, 1.0},
};

GLdouble model0[16], model1[16];  //モデルビュー変換行列用
GLdouble proj0[16], proj1[16];  //投影変換行列用
GLint view0[4], view1[4];  //ビューポート設定用

Vec_3D objData[KX][KY];

double rDisp = 1.0;

//メイン関数
int main(int argc, char *argv[])
{
    glutInit(&argc, argv);  //OpenGL/GLUTの初期化
    initGL();  //初期設定
    
    glutMainLoop();  //イベント待ち無限ループ
    
    return 0;
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

    //描画ウィンドウ生成1（ユーザに知覚させたい映像の生成）
    glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);  //ディスプレイモードの指定
    glutInitWindowSize(TEX_SIZE, TEX_SIZE);  //ウィンドウサイズの指定
    glutInitWindowPosition(0, 512);  //ウィンドウ位置の指定
    winID[1] = glutCreateWindow("CG1");  //ウィンドウの生成
    //コールバック関数の指定
    glutDisplayFunc(display1);  //ディスプレイコールバック関数の指定
    glutReshapeFunc(reshape1);  //リシェイプコールバック関数の指定
    glutMouseFunc(mouse1);  //マウスクリックコールバック関数の指定
    glutMotionFunc(motion1);  //マウスドラッグコールバック関数の指定
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

    //描画ウィンドウ生成2（影エリア映像の生成）
    glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);  //ディスプレイモードの指定
    glutInitWindowSize(scanW*reso, scanH*reso);  //ウィンドウサイズの指定
    glutInitWindowPosition(512, 0);  //ウィンドウ位置の指定
    winID[2] = glutCreateWindow("CG2");  //ウィンドウの生成
    //コールバック関数の指定
    glutDisplayFunc(display2);  //ディスプレイコールバック関数の指定
    glutReshapeFunc(reshape2);  //リシェイプコールバック関数の指定
    glutMouseFunc(mouse2);  //マウスクリックコールバック関数の指定
    glutMotionFunc(motion2);  //マウスドラッグコールバック関数の指定
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
    eDist[2] = 250.0; eDegX[2] = 90.0; eDegY[2] = 0.0;  //視点極座標

    //描画ウィンドウ生成3（照明用映像の生成）
    glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);  //ディスプレイモードの指定
    glutInitWindowSize(800, 800*aspectRate);  //ウィンドウサイズの指定
    glutInitWindowPosition(512, scanH*reso+100);  //ウィンドウ位置の指定
    winID[3] = glutCreateWindow("CG3");  //ウィンドウの生成
    //コールバック関数の指定
    glutDisplayFunc(display3);  //ディスプレイコールバック関数の指定
    glutReshapeFunc(reshape3);  //リシェイプコールバック関数の指定
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
    for (int i=0; i<4; i++) {
        glutSetWindow(winID[i]);  //設定対象ウィンドウ

        //テクスチャ
        //テクスチャオブジェクト生成(#100)
        textureImage = cv::imread("spot1.png", cv::IMREAD_UNCHANGED);
        objPos.s = textureImage.cols; objPos.t = textureImage.rows;
        objPos.id = 1;
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

    frameImage0 = cv::Mat(cv::Size(TEX_SIZE, TEX_SIZE), CV_8UC3);
    frameImage0b = cv::Mat(cv::Size(TEX_SIZE, TEX_SIZE), CV_8UC4);
    frameImage1 = cv::Mat(cv::Size(TEX_SIZE, TEX_SIZE), CV_8UC3);
    frameImage1b = cv::Mat(cv::Size(TEX_SIZE, TEX_SIZE), CV_8UC4);

    //表示パネルサイズ・位置
    objPos.z = 5.0;  //位置
    
    //光源位置
    lightPos0.x = 10.0; lightPos0.y = 15.0; lightPos0.z = scanH/2.0;
    //lightCollPos0.z = objPos.z;  //光源と物体との衝突位置
    lightPos1.x = -10.0; lightPos1.y = 15.0; lightPos1.z = scanH/2.0;
    //lightCollPos1.z = objPos.z;  //光源と物体との衝突位置
    
    //
    
}

//ユーザに知覚させたいオブジェクトの描画
void scene()
{
    //オブジェクト（本体）
    glDisable(GL_LIGHTING);
    glColor4d(1.0, 0.0, 0.0, 1.0);
    glPushMatrix();
    glTranslated(0.0, 2.5, 5.0);
    glScaled(5.0, 5.0, 5.0);
    glutSolidCube(1.0);
    glPopMatrix();

    
    if (dispMode) {
        
        
        //LED光源位置
        glDisable(GL_LIGHTING);
        glColor4d(0.0, 1.0, 0.0, 1.0);
        glPushMatrix();
        glTranslated(lightPos0.x, lightPos0.y, lightPos0.z);
        glScaled(0.5, 0.5, 0.5);
        glutSolidSphere(1.0, 36, 18);
        glPopMatrix();
        
        //LED光源衝突位置
        glDisable(GL_LIGHTING);
        glColor4d(1.0, 1.0, 0.0, 1.0);
        glPushMatrix();
        glTranslated(lightCollPos0.x, lightCollPos0.y, lightCollPos0.z);
        glScaled(0.5, 0.5, 0.5);
        glutSolidSphere(1.0, 36, 18);
        glPopMatrix();
        
        //LED光源位置
        glDisable(GL_LIGHTING);
        glColor4d(1.0, 0.0, 1.0, 1.0);
        glPushMatrix();
        glTranslated(lightPos1.x, lightPos1.y, lightPos1.z);
        glScaled(0.5, 0.5, 0.5);
        glutSolidSphere(1.0, 36, 18);
        glPopMatrix();
        
        //LED光源衝突位置
        glDisable(GL_LIGHTING);
        glColor4d(0.0, 1.0, 1.0, 1.0);
        glPushMatrix();
        glTranslated(lightCollPos1.x, lightCollPos1.y, lightCollPos1.z);
        glScaled(0.5, 0.5, 0.5);
        glutSolidSphere(1.0, 36, 18);
        glPopMatrix();
        
        /*
        glColor4d(1.0, 0.0, 0.0, 1.0);
        glPushMatrix();
        glTranslated(touchPos.x, 0.0, touchPos.z);
        glScaled(0.5, 0.5, 0.5);
        glutSolidSphere(1.0, 36, 18);
        glPopMatrix();
        */

        /*
        glColor4d(1.0, 1.0, 1.0, 1.0);
        for (int i=0; i<footNum; i++) {
            glPushMatrix();
            glTranslated(footPoint[i].x, 0.0, footPoint[i].z);
            glScaled(1.0, 1.0, 1.0);
            glutWireCube(1.0);
            glPopMatrix();
        }
        */
    }
}

//ディスプレイコールバック関数
void display0()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);  //画面消去
    
    //投影変換の設定
    glMatrixMode(GL_PROJECTION);  //変換行列の指定（設定対象は投影変換行列）
    glLoadIdentity();  //行列初期化
    gluPerspective(110.0, (double)winW[0]/(double)winH[0], 1.0, 100.0);  //透視投影ビューボリューム設定
    
    //モデルビュー変換の設定
    glMatrixMode(GL_MODELVIEW);  //変換行列の指定（設定対象はモデルビュー変換行列）
    glLoadIdentity();  //行列初期化
    gluLookAt(lightPos0.x, lightPos0.y, lightPos0.z, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);  //視点視線設定（視野変換行列を乗算）
    
    glGetDoublev(GL_MODELVIEW_MATRIX, model0);  //モデルビュー変換行列の取得
    glGetDoublev(GL_PROJECTION_MATRIX, proj0);  //投影変換行列の取得
    glGetIntegerv(GL_VIEWPORT, view0);  //ビューポート設定の取得

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

    //描画したシーンを画像としてframeImage0に格納
    glReadPixels(0, 0, winW[0]*rDisp, winH[0]*rDisp, GL_BGR, GL_UNSIGNED_BYTE, frameImage0a.data);
    cv::resize(frameImage0a, frameImage0, frameImage0.size());
    cv::flip(frameImage0, frameImage0, 0);
    
    //アルファチャンネル付き
    for (int j=0; j<TEX_SIZE; j++) {
        for (int i=0; i<TEX_SIZE; i++) {
            cv::Vec3b s;
            cv::Vec4b s1;

            s = frameImage0.at<cv::Vec3b>(j, i);
            if (s[0]<1 && s[1]>254 && s[2]<1) {
                s1[0] = 0; s1[1] = 0; s1[2] = 0;
                s1[3] = 0;
            }
            else {
                s1[0] = 255; s1[1] = 0; s1[2] = 0;
                s1[3] = 255;
            }
            frameImage0b.at<cv::Vec4b>(j, i) = s1;
        }
    }
    
    //cv::imshow("frame0", frameImage0b);
    
    glutSwapBuffers();  //描画実行
}

//ディスプレイコールバック関数
void display1()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);  //画面消去
    
    //投影変換の設定
    glMatrixMode(GL_PROJECTION);  //変換行列の指定（設定対象は投影変換行列）
    glLoadIdentity();  //行列初期化
    gluPerspective(110.0, (double)winW[1]/(double)winH[1], 1.0, 100.0);  //透視投影ビューボリューム設定
    
    //モデルビュー変換の設定
    glMatrixMode(GL_MODELVIEW);  //変換行列の指定（設定対象はモデルビュー変換行列）
    glLoadIdentity();  //行列初期化
    gluLookAt(lightPos1.x, lightPos1.y, lightPos1.z, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);  //視点視線設定（視野変換行列を乗算）
    
    glGetDoublev(GL_MODELVIEW_MATRIX, model1);  //モデルビュー変換行列の取得
    glGetDoublev(GL_PROJECTION_MATRIX, proj1);  //投影変換行列の取得
    glGetIntegerv(GL_VIEWPORT, view1);  //ビューポート設定の取得

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
    
    //描画したシーンを画像としてframeImage1に格納
    glReadPixels(0, 0, winW[1]*rDisp, winH[1]*rDisp, GL_BGR, GL_UNSIGNED_BYTE, frameImage1a.data);
    cv::resize(frameImage1a, frameImage1, frameImage1.size());
    cv::flip(frameImage1, frameImage1, 0);
    
    //アルファチャンネル付き
    for (int j=0; j<TEX_SIZE; j++) {
        for (int i=0; i<TEX_SIZE; i++) {
            cv::Vec3b s;
            cv::Vec4b s1;
            
            s = frameImage1.at<cv::Vec3b>(j, i);
            if (s[0]<1 && s[1]>254 && s[2]<1) {
                s1[0] = 0; s1[1] = 0; s1[2] = 0;
                s1[3] = 0;
            }
            else {
                s1[0] = 0; s1[1] = 0; s1[2] = 255;
                s1[3] = 255;
            }
            frameImage1b.at<cv::Vec4b>(j, i) = s1;
        }
    }
    
    
    //cv::imshow("frame1", frameImage1b);
    
    glutSwapBuffers();  //描画実行
}

//ディスプレイコールバック関数
void display2()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);  //画面消去
    
    //視点座標の計算
    Vec_3D e;
    e.x = eDist[2]*cos(eDegX[2]*M_PI/180.0)*sin(eDegY[2]*M_PI/180.0);
    e.y = eDist[2]*sin(eDegX[2]*M_PI/180.0);
    e.z = eDist[2]*cos(eDegX[2]*M_PI/180.0)*cos(eDegY[2]*M_PI/180.0);
    
    //モデルビュー変換の設定
    glMatrixMode(GL_MODELVIEW);  //変換行列の指定（設定対象はモデルビュー変換行列）
    glLoadIdentity();  //行列初期化
    if (eDegX[2]>90.0) {
        gluLookAt(e.x, e.y, e.z, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0);  //視点視線設定（視野変換行列を乗算）
    }
    else {
        gluLookAt(e.x, e.y, e.z, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);  //視点視線設定（視野変換行列を乗算）
    }
    
    glColor4d(1.0, 1.0, 0.0, 1.0);
    //for (int i=0; i<footNum; i++) {
        glPushMatrix();
        glTranslated(touchPosV.x, touchPosV.y, touchPosV.z);
        glScaled(1.0, 1.0, 1.0);
        glutWireCube(1.0);
        glPopMatrix();
    //}

    if (dispMode) {
        
//        //LED光源衝突位置
//        glDisable(GL_LIGHTING);
//        glColor4d(1.0, 1.0, 0.0, 1.0);
//        glPushMatrix();
//        glTranslated(lightCollPos0.x, lightCollPos0.y, lightCollPos0.z);
//        glScaled(0.5, 0.5, 0.5);
//        glutSolidSphere(1.0, 36, 18);
//        glPopMatrix();
//        
//        //LED光源衝突位置
//        glDisable(GL_LIGHTING);
//        glColor4d(0.0, 1.0, 1.0, 1.0);
//        glPushMatrix();
//        glTranslated(lightCollPos1.x, lightCollPos1.y, lightCollPos1.z);
//        glScaled(0.5, 0.5, 0.5);
//        glutSolidSphere(1.0, 36, 18);
//        glPopMatrix();

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

    //単純な白い床を描画
    glDisable(GL_TEXTURE_2D);  //テクスチャ無効化
    glDisable(GL_LIGHTING);
    glColor4d(1.0, 1.0, 1.0, 1.0);
    glPushMatrix();
    glTranslated(0.0, -0.1, 0.0);
    glScaled(scanW*1.01, 1.0, scanH*1.01);
    glBegin(GL_QUADS);
    glVertex3d(-0.5, 0.0, 0.5);
    glVertex3d(0.5, 0.0, 0.5);
    glVertex3d(0.5, 0.0, -0.5);
    glVertex3d(-0.5, 0.0, -0.5);
    glEnd();
    glPopMatrix();
    
    //display0の映像視点に基づくテクスチャ座標の自動生成
    glMatrixMode(GL_TEXTURE);  //変換行列の指定（テクスチャ行列）
    glLoadIdentity();  //行列初期化
    glTranslated(0.5, 0.5, 0.5);  //テクスチャ座標と同じ0~1になるように0.5だけ平行移動
    glScaled(0.5, 0.5, 0.5);  //視点座標系は-1~1なので，-0.5~0.5になるように縮小
    glRotated(180.0, 1.0, 0.0, 0.0);  //上下反転
    gluPerspective(90.0, (GLdouble)winW[0]/(GLdouble)winH[0], 50.0, 1000.0);  //display0と同じ投影変換
    gluLookAt(lightPos0.x, lightPos0.y, lightPos0.z, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);  //display0と同じ視点
    //自動生成されたテクスチャ座標を用いて，床面に相当する平面にdisplay0で生成した映像をテクスチャマッピング
    glEnable(GL_TEXTURE_2D);  //テクスチャ有効化
    glBindTexture(GL_TEXTURE_2D, 0);  //テクスチャ0を指定
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, TEX_SIZE, TEX_SIZE, 0, GL_BGRA, GL_UNSIGNED_BYTE, frameImage0b.data);  //display0で生成した映像をテクスチャ画像として登録
    glMatrixMode(GL_MODELVIEW);  //変換行列の指定（設定対象はモデルビュー変換行列）
    //床面相当の平面を描画
    glDisable(GL_LIGHTING);
    glColor4d(1.0, 1.0, 1.0, 0.5);
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
    
    //display1の映像視点に基づくテクスチャ座標の自動生成
    glMatrixMode(GL_TEXTURE);  //変換行列の指定（テクスチャ行列）
    glLoadIdentity();  //行列初期化
    glTranslated(0.5, 0.5, 0.5);  //テクスチャ座標と同じ0~1になるように0.5だけ平行移動
    glScaled(0.5, 0.5, 0.5);  //視点座標系は-1~1なので，-0.5~0.5になるように縮小
    glRotated(180.0, 1.0, 0.0, 0.0);  //上下反転
    gluPerspective(90.0, (GLdouble)winW[1]/(GLdouble)winH[1], 50.0, 1000.0);  //display0と同じ投影変換
    gluLookAt(lightPos1.x, lightPos1.y, lightPos1.z, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);  //display0と同じ視点
    //自動生成されたテクスチャ座標を用いて，床面に相当する平面にdisplay0で生成した映像をテクスチャマッピング
    glEnable(GL_TEXTURE_2D);  //テクスチャ有効化
    glBindTexture(GL_TEXTURE_2D, 0);  //テクスチャ0を指定
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, TEX_SIZE, TEX_SIZE, 0, GL_BGRA, GL_UNSIGNED_BYTE, frameImage1b.data);  //display0で生成した映像をテクスチャ画像として登録
    glMatrixMode(GL_MODELVIEW);  //変換行列の指定（設定対象はモデルビュー変換行列）
    //床面相当の平面を描画
    glDisable(GL_LIGHTING);
    glColor4d(1.0, 1.0, 1.0, 0.5);
    glPushMatrix();
    glTranslated(0.0, 0.1, 0.0);
    glScaled(1000.0, 1.0, 1000.0);
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
    
    glDisable(GL_TEXTURE_2D);  //テクスチャ無効化

    //======================================================================================================================

    glutSwapBuffers();  //描画実行
    
    //描画したシーンを画像としてframeImageに格納
    glReadPixels(0, 0, winW[2]*rDisp, winH[2]*rDisp, GL_BGR, GL_UNSIGNED_BYTE, shadowAreaImage.data);
    cv::flip(shadowAreaImage, shadowAreaImage, 0);
    cv::cvtColor(shadowAreaImage, shadowAreaGrayImage, cv::COLOR_BGR2GRAY);
    //cv::imshow("shadowAreaImage", shadowAreaGrayImage);
}

//ディスプレイコールバック関数
void display3()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);  //画面消去
        
    //モデルビュー変換の設定
    glMatrixMode(GL_MODELVIEW);  //変換行列の指定（設定対象はモデルビュー変換行列）
    glLoadIdentity();  //行列初期化
    
    //照明0
    glColor4d(1.0, 2*lightPos0.y/lightH, 2*lightPos0.y/lightH, 1.0);
    glPushMatrix();
    glTranslated(-lightPos0.x, lightPos0.y, 0.0);
    glScaled(3.0, 3.0, 1.0);
    glMyTexPlane(1, 2, 2);
    glPopMatrix();

    //照明1
    glColor4d(1.0, 2*lightPos1.y/lightH, 2*lightPos1.y/lightH, 1.0);
    glPushMatrix();
    glTranslated(-lightPos1.x, lightPos1.y, 0.0);
    glScaled(3.0, 3.0, 1.0);
    glMyTexPlane(1, 2, 2);
    glPopMatrix();

    glutSwapBuffers();  //描画実行
}

//リシェイプコールバック関数
void reshape0(int w, int h)
{
    int wID = 0;
    
    winW[wID] = w; winH[wID] = h;  //ウィンドウサイズ格納
    glViewport(0, 0, winW[wID], winH[wID]);  //ビューポート設定
    frameImage0a = cv::Mat(cv::Size(winW[wID], winH[wID]), CV_8UC3);
}

//リシェイプコールバック関数
void reshape1(int w, int h)
{
    int wID = 1;
    
    winW[wID] = w; winH[wID] = h;  //ウィンドウサイズ格納
    glViewport(0, 0, winW[wID], winH[wID]);  //ビューポート設定
    frameImage1a = cv::Mat(cv::Size(winW[wID], winH[wID]), CV_8UC3);
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
    //gluPerspective(40.0, (double)winW[wID]/(double)winH[wID], 50.0, 1000.0);  //透視投影ビューボリューム設定
    glFrustum(-scanW*0.5*0.1, scanW*0.5*0.1, -scanH*0.5*0.1, scanH*0.5*0.1, eDist[wID]*0.1, 1000.0);

    winW[wID] = w; winH[wID] = h;  //ウィンドウサイズ格納
    shadowAreaImage = cv::Mat(cv::Size(winW[wID], winH[wID]), CV_8UC3);
}

//リシェイプコールバック関数
void reshape3(int w, int h)
{
    int wID = 3;
    
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
    }
}

//マウスクリックコールバック関数
void mouse2(int button, int state, int x, int y)
{
    int wID = 2;
    
    //マウスボタンが押された
    if (state==GLUT_DOWN) {
        mX[wID] = x; mY[wID] = y; mState[wID] = state; mButton[wID] = button;  //マウス情報保持
        
        if (button==GLUT_LEFT_BUTTON) {
            //タッチ位置
            Vec_3D touchPos0;
            touchPos0.x = (mX[wID]-scanW*reso*0.5)/reso; touchPos0.y = 0.0; touchPos0.z = (mY[wID]-scanH*reso*0.5)/reso;
            double len = sqrt(pow(touchPos.x-touchPos0.x,2)+pow(touchPos.z-touchPos0.z,2));
            touchPos = touchPos0;
            //printf("touchPos = (%f, %f, %f)\n", touchPos.x, touchPos.y, touchPos.z);

            //タッチ位置シャドウ画像画素値
            shadowVal = shadowAreaGrayImage.at<unsigned char>(mY[wID], mX[wID]);
            shadowColVal = shadowAreaImage.at<cv::Vec3b>(mY[wID], mX[wID]);
            //printf("shadowVal = %d\n", shadowVal);
            printf("shadowColVal = %d, %d, %d\n", shadowColVal[0], shadowColVal[1], shadowColVal[2]);
            
            //if (shadowVal==0 && len<10.0) {  //影を動かす
            if (shadowColVal[0]==255 && shadowColVal[1]==128 && shadowColVal[2]==128 && len<10.0) {  //影を動かす
                lightVec0.x = lightPos0.x-touchPos.x; lightVec0.y = lightPos0.y-touchPos.y; lightVec0.z = lightPos0.z-touchPos.z;
                lightVec0 = vectorNormalize(lightVec0);
                //printf("lightVec0 = (%f, %f, %f)\n", lightVec0.x, lightVec0.y, lightVec0.z);
                double t0 = (objPos.z-touchPos.z)/lightVec0.z;
                lightCollPos0.x = touchPos.x+lightVec0.x*t0;
                lightCollPos0.y = touchPos.y+lightVec0.y*t0;

//                lightVec0.x = lightCollPos0.x-touchPos.x;
//                lightVec0.y = lightCollPos0.y-touchPos.y;
//                lightVec0.z = lightCollPos0.z-touchPos.z;
//                lightVec0 = vectorNormalize(lightVec0);
//                
//                double t0 = (lightPos0.z-touchPos.z)/lightVec0.z;
//                lightPos0.x = touchPos.x+lightVec0.x*t0;
//                lightPos0.y = touchPos.y+lightVec0.y*t0;
            }
            else if (shadowColVal[0]==128 && shadowColVal[1]==128 && shadowColVal[2]==255 && len<10.0) {  //影を動かす
                lightVec1.x = lightPos1.x-touchPos.x; lightVec1.y = lightPos1.y-touchPos.y; lightVec1.z = lightPos1.z-touchPos.z;
                lightVec1 = vectorNormalize(lightVec1);
                //printf("lightVec1 = (%f, %f, %f)\n", lightVec1.x, lightVec1.y, lightVec1.z);
                double t1 = (objPos.z-touchPos.z)/lightVec1.z;
                lightCollPos1.x = touchPos.x+lightVec1.x*t1;
                lightCollPos1.y = touchPos.y+lightVec1.y*t1;

//                lightVec1.x = lightCollPos1.x-touchPos.x;
//                lightVec1.y = lightCollPos1.y-touchPos.y;
//                lightVec1.z = lightCollPos1.z-touchPos.z;
//                lightVec1 = vectorNormalize(lightVec1);
//                
//                double t1 = (lightPos1.z-touchPos.z)/lightVec1.z;
//                lightPos1.x = touchPos.x+lightVec1.x*t1;
//                lightPos1.y = touchPos.y+lightVec1.y*t1;
            }
            else if (shadowColVal[0]==128 && shadowColVal[1]==64 && shadowColVal[2]==91 && len<10.0) {  //影を動かす
                lightVec0.x = lightPos0.x-touchPos.x; lightVec0.y = lightPos0.y-touchPos.y; lightVec0.z = lightPos0.z-touchPos.z;
                lightVec0 = vectorNormalize(lightVec0);
                //printf("lightVec0 = (%f, %f, %f)\n", lightVec0.x, lightVec0.y, lightVec0.z);
                double t0 = (objPos.z-touchPos.z)/lightVec0.z;
                lightCollPos0.x = touchPos.x+lightVec0.x*t0;
                lightCollPos0.y = touchPos.y+lightVec0.y*t0;

                lightVec1.x = lightPos1.x-touchPos.x; lightVec1.y = lightPos1.y-touchPos.y; lightVec1.z = lightPos1.z-touchPos.z;
                lightVec1 = vectorNormalize(lightVec1);
                //printf("lightVec1 = (%f, %f, %f)\n", lightVec1.x, lightVec1.y, lightVec1.z);
                double t1 = (objPos.z-touchPos.z)/lightVec1.z;
                lightCollPos1.x = touchPos.x+lightVec1.x*t1;
                lightCollPos1.y = touchPos.y+lightVec1.y*t1;

//                lightVec0.x = lightCollPos0.x-touchPos.x;
//                lightVec0.y = lightCollPos0.y-touchPos.y;
//                lightVec0.z = lightCollPos0.z-touchPos.z;
//                lightVec0 = vectorNormalize(lightVec0);
//                double t0 = (lightPos0.z-touchPos.z)/lightVec0.z;
//                lightPos0.x = touchPos.x+lightVec0.x*t0;
//                lightPos0.y = touchPos.y+lightVec0.y*t0;
//
//                lightVec1.x = lightCollPos1.x-touchPos.x;
//                lightVec1.y = lightCollPos1.y-touchPos.y;
//                lightVec1.z = lightCollPos1.z-touchPos.z;
//                lightVec1 = vectorNormalize(lightVec1);
//                double t1 = (lightPos1.z-touchPos.z)/lightVec1.z;
//                lightPos1.x = touchPos.x+lightVec1.x*t1;
//                lightPos1.y = touchPos.y+lightVec1.y*t1;
            }
            else {
                if (shadowColVal[0]==255 && shadowColVal[1]==128 && shadowColVal[2]==128) {  //影を動かす
                    lightVec0.x = lightPos0.x-touchPos.x; lightVec0.y = lightPos0.y-touchPos.y; lightVec0.z = lightPos0.z-touchPos.z;
                    lightVec0 = vectorNormalize(lightVec0);
                    //printf("lightVec0 = (%f, %f, %f)\n", lightVec0.x, lightVec0.y, lightVec0.z);
                    double t0 = (objPos.z-touchPos.z)/lightVec0.z;
                    lightCollPos0.x = touchPos.x+lightVec0.x*t0;
                    lightCollPos0.y = touchPos.y+lightVec0.y*t0;
                }
                else if (shadowColVal[0]==128 && shadowColVal[1]==128 && shadowColVal[2]==255) {  //影を動かす
                    lightVec1.x = lightPos1.x-touchPos.x; lightVec1.y = lightPos1.y-touchPos.y; lightVec1.z = lightPos1.z-touchPos.z;
                    lightVec1 = vectorNormalize(lightVec1);
                    //printf("lightVec1 = (%f, %f, %f)\n", lightVec1.x, lightVec1.y, lightVec1.z);
                    double t1 = (objPos.z-touchPos.z)/lightVec1.z;
                    lightCollPos1.x = touchPos.x+lightVec1.x*t1;
                    lightCollPos1.y = touchPos.y+lightVec1.y*t1;
                }
                else if (shadowColVal[0]==128 && shadowColVal[1]==64 && shadowColVal[2]==191) {  //影を動かす
                    lightVec0.x = lightPos0.x-touchPos.x; lightVec0.y = lightPos0.y-touchPos.y; lightVec0.z = lightPos0.z-touchPos.z;
                    lightVec0 = vectorNormalize(lightVec0);
                    //printf("lightVec0 = (%f, %f, %f)\n", lightVec0.x, lightVec0.y, lightVec0.z);
                    double t0 = (objPos.z-touchPos.z)/lightVec0.z;
                    lightCollPos0.x = touchPos.x+lightVec0.x*t0;
                    lightCollPos0.y = touchPos.y+lightVec0.y*t0;
                    
                    lightVec1.x = lightPos1.x-touchPos.x; lightVec1.y = lightPos1.y-touchPos.y; lightVec1.z = lightPos1.z-touchPos.z;
                    lightVec1 = vectorNormalize(lightVec1);
                    //printf("lightVec1 = (%f, %f, %f)\n", lightVec1.x, lightVec1.y, lightVec1.z);
                    double t1 = (objPos.z-touchPos.z)/lightVec1.z;
                    lightCollPos1.x = touchPos.x+lightVec1.x*t1;
                    lightCollPos1.y = touchPos.y+lightVec1.y*t1;
                }
            }
        }
        
        //printf("lightCollPos0 = (%f, %f, %f)\n", lightCollPos0.x, lightCollPos0.y, lightCollPos0.z);
        //printf("lightCollPos1 = (%f, %f, %f)\n", lightCollPos1.x, lightCollPos1.y, lightCollPos1.z);

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
        lightPos1.x += (mX[wID]-x)*0.5;  //マウス横方向→水平角
        lightPos1.y += (y-mY[wID])*0.5;  //マウス縦方向→垂直角
    }
    
    //マウス座標をグローバル変数に保存
    mX[wID] = x; mY[wID] = y;
}

//マウスドラッグコールバック関数
void motion2(int x, int y)
{
    int wID = 2;
    
    if (mButton[wID]==GLUT_RIGHT_BUTTON) {
        //マウスの移動量を角度変化量に変換
        eDegY[wID] = eDegY[wID]+(mX[wID]-x)*0.5;  //マウス横方向→水平角
        eDegX[wID] = eDegX[wID]+(y-mY[wID])*0.5;  //マウス縦方向→垂直角
    }
    //else if (mButton[wID]==GLUT_LEFT_BUTTON && shadowVal==0) {
    else if (mButton[wID]==GLUT_LEFT_BUTTON && shadowColVal[0]==255 && shadowColVal[1]==128 && shadowColVal[2]==128) {
        //タッチ位置
        touchPos.x = (mX[wID]-scanW*reso*0.5)/reso; touchPos.y = 0.0; touchPos.z = (mY[wID]-scanH*reso*0.5)/reso;
        
        lightVec0.x = lightCollPos0.x-touchPos.x;
        lightVec0.y = lightCollPos0.y-touchPos.y;
        lightVec0.z = lightCollPos0.z-touchPos.z;
        lightVec0 = vectorNormalize(lightVec0);

        double t0 = (lightPos0.z-touchPos.z)/lightVec0.z;
        lightPos0.x = touchPos.x+lightVec0.x*t0;
        lightPos0.y = touchPos.y+lightVec0.y*t0;
    }
    else if (mButton[wID]==GLUT_LEFT_BUTTON && shadowColVal[0]==128 && shadowColVal[1]==128 && shadowColVal[2]==255) {
        //タッチ位置
        touchPos.x = (mX[wID]-scanW*reso*0.5)/reso; touchPos.y = 0.0; touchPos.z = (mY[wID]-scanH*reso*0.5)/reso;
        
        lightVec1.x = lightCollPos1.x-touchPos.x;
        lightVec1.y = lightCollPos1.y-touchPos.y;
        lightVec1.z = lightCollPos1.z-touchPos.z;
        lightVec1 = vectorNormalize(lightVec1);
        
        double t1 = (lightPos1.z-touchPos.z)/lightVec1.z;
        lightPos1.x = touchPos.x+lightVec1.x*t1;
        lightPos1.y = touchPos.y+lightVec1.y*t1;
    }
    else if (mButton[wID]==GLUT_LEFT_BUTTON && shadowColVal[0]==128 && shadowColVal[1]==64 && shadowColVal[2]==191) {
        //タッチ位置
        touchPos.x = (mX[wID]-scanW*reso*0.5)/reso; touchPos.y = 0.0; touchPos.z = (mY[wID]-scanH*reso*0.5)/reso;
        
        lightVec0.x = lightCollPos0.x-touchPos.x;
        lightVec0.y = lightCollPos0.y-touchPos.y;
        lightVec0.z = lightCollPos0.z-touchPos.z;
        lightVec0 = vectorNormalize(lightVec0);
        double t0 = (lightPos0.z-touchPos.z)/lightVec0.z;
        lightPos0.x = touchPos.x+lightVec0.x*t0;
        lightPos0.y = touchPos.y+lightVec0.y*t0;

        lightVec1.x = lightCollPos1.x-touchPos.x;
        lightVec1.y = lightCollPos1.y-touchPos.y;
        lightVec1.z = lightCollPos1.z-touchPos.z;
        lightVec1 = vectorNormalize(lightVec1);
        double t1 = (lightPos1.z-touchPos.z)/lightVec1.z;
        lightPos1.x = touchPos.x+lightVec1.x*t1;
        lightPos1.y = touchPos.y+lightVec1.y*t1;
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
            eDegX[2] = 90.0;
            eDegY[2] = 0.0;
            break;
            
        case 'l':
            lightPos0.x = 10.0; lightPos0.y = 15.0; lightPos0.z = scanH/2.0;
            lightPos1.x = -10.0; lightPos1.y = 15.0; lightPos1.z = scanH/2.0;
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
    static int cnt = 0;
    cnt++;
    if (cnt>=30) cnt = 0;
    
    glutSetWindow(winID[0]);
    glutTimerFunc(1000/fr, timer0, 0);  //タイマー再設定
    glutPostRedisplay();  //ディスプレイイベント強制発生

    glutSetWindow(winID[1]);
    glutPostRedisplay();  //ディスプレイイベント強制発生

    glutSetWindow(winID[2]);
    glutPostRedisplay();  //ディスプレイイベント強制発生
    
    glutSetWindow(winID[3]);
    glutPostRedisplay();  //ディスプレイイベント強制発生
    
    
    int wID = 2;
    FILE *fp = fopen("LIDAR1x/footpoint.txt", "r");
    footNum = 0;
    if (fp!=NULL) {
        int tmpNum;
        fscanf(fp, "%d", &tmpNum);
        double tmpX, tmpZ;
        for (int i=0; i<tmpNum; i++) {
            fscanf(fp, "%lf,%lf", &tmpX, &tmpZ);
            //if (tmpZ>180.0 || tmpZ<80.0) continue;
            
            footPoint[footNum].x = -tmpX;
            footPoint[footNum].z = tmpZ-scanH/2;
            footPoint[footNum].y = 0.0;
            footNum++;
            if (footNum==POINTMAX) break;
        }
        fclose(fp);
        
//        if (cnt==0) {
//            for (int i=0; i<footNum; i++) {
//                printf("(%f, %f)\n", footPoint[i].x, footPoint[i].z);
//            }
//        }
        
        double lenMax = sqrt(pow(scanW,2)+pow(scanH,2));
        int tID = -1;
        for (int i=0; i<footNum; i++) {
            double len = sqrt(pow(footPoint[i].x,2)+pow(footPoint[i].z-scanH/2,2));
            if (len<lenMax) {
                lenMax = 0;
                tID = i;
            }
        }
        //printf("tID = %d\n", tID);
        
        int flg0 = 0, flg1 = 0, flg2 = 0;
        
        for (int i=0; i<footNum; i++) {
            if (tID!=i) continue;
            
            touchPosV = footPoint[i];

            int imgX, imgY;
            imgX = (footPoint[i].x-(-scanW/2.0))/scanW*shadowAreaGrayImage.cols;
            imgY = (footPoint[i].z-(-scanH/2.0))/scanH*shadowAreaGrayImage.rows;
            
            unsigned char shadowVal0 = shadowAreaGrayImage.at<unsigned char>(imgY, imgX);
            cv::Vec3b shadowColVal0 = shadowAreaImage.at<cv::Vec3b>(imgY, imgX);

            //if (shadowVal0==0) {
            if (shadowColVal0[0]==255 & shadowColVal0[1]==128 & shadowColVal0[2]==128) {
                if (chaseFlg0==0 && flg0==0) {
                    touchPosX0 = footPoint[i];
                    chaseFlg0 = 1;
                    //
                    lightVec0.x = lightPos0.x-touchPosX0.x; lightVec0.y = lightPos0.y-touchPosX0.y; lightVec0.z = lightPos0.z-touchPosX0.z;
                    lightVec0 = vectorNormalize(lightVec0);
                    
                    double disp0X, disp0Y, disp0Z;
                    float disp0Zd;
                    gluProject(touchPosV.x, touchPosV.y, touchPosV.z, model0, proj0, view0, &disp0X, &disp0Y, &disp0Z);
                    glutSetWindow(winID[0]);
                    glReadPixels(disp0X, disp0Y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &disp0Zd);
                    gluUnProject(disp0X, disp0Y, disp0Zd, model0, proj0, view0, &lightCollPos0.x, &lightCollPos0.y, &lightCollPos0.z);

                    //printf("lightVec0 = (%f, %f, %f)\n", lightVec0.x, lightVec0.y, lightVec0.z);
                    //double t = (objPos.z-touchPosX0.z)/lightVec0.z;
                    //lightCollPos0.x = touchPosX0.x+lightVec0.x*t;
                    //lightCollPos0.y = touchPosX0.y+lightVec0.y*t;

                    //printf("%f, %f\n", touchPosX0.x, touchPosX0.z);
                    //break;
                }
                else if (chaseFlg0==1 && flg0==0) {
                    double len = sqrt(pow(touchPosX0.x-footPoint[i].x,2)+pow(touchPosX0.z-footPoint[i].z,2));
                    if (len<2.0) {
                        lightVec0.x = lightCollPos0.x-touchPosX0.x;
                        lightVec0.y = lightCollPos0.y-touchPosX0.y;
                        lightVec0.z = lightCollPos0.z-touchPosX0.z;
                        lightVec0 = vectorNormalize(lightVec0);
                        
                        double t = (lightPos0.z-touchPosX0.z)/lightVec0.z;
                        lightPos0.x = touchPosX0.x+lightVec0.x*t;
                        lightPos0.y = touchPosX0.y+lightVec0.y*t;
                        touchPosX0 = footPoint[i];
                        chaseFlg0 = 1;
                        if (lightPos0.x>lightW*0.5)
                            lightPos0.x = lightW*0.5;
                        if (lightPos0.x<-lightW*0.5)
                            lightPos0.x = -lightW*0.5;
                        if (lightPos0.y>lightH)
                            lightPos0.y = lightH;
                        if (lightPos0.y<6.0)
                            lightPos0.y = 6.0;

                        //break;
                    }
                }
                flg0 = 1;
            }
            else if (shadowColVal0[0]==128 & shadowColVal0[1]==128 & shadowColVal0[2]==255) {
                if (chaseFlg1==0 && flg1==0) {
                    touchPosX1 = footPoint[i];
                    chaseFlg1 = 1;
                    //
                    lightVec1.x = lightPos1.x-touchPosX1.x; lightVec1.y = lightPos1.y-touchPosX1.y; lightVec1.z = lightPos1.z-touchPosX1.z;
                    lightVec1 = vectorNormalize(lightVec1);

                    double disp1X, disp1Y, disp1Z;
                    float disp1Zd;
                    gluProject(touchPosV.x, touchPosV.y, touchPosV.z, model1, proj1, view1, &disp1X, &disp1Y, &disp1Z);
                    glutSetWindow(winID[1]);
                    glReadPixels(disp1X, disp1Y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &disp1Zd);
                    gluUnProject(disp1X, disp1Y, disp1Zd, model1, proj1, view1, &lightCollPos1.x, &lightCollPos1.y, &lightCollPos1.z);

                    //printf("lightVec1 = (%f, %f, %f)\n", lightVec1.x, lightVec1.y, lightVec1.z);
                    //double t = (objPos.z-touchPosX1.z)/lightVec1.z;
                    //lightCollPos1.x = touchPosX1.x+lightVec1.x*t;
                    //lightCollPos1.y = touchPosX1.y+lightVec1.y*t;
                    //printf("%f, %f\n", touchPosX1.x, touchPosX1.z);
                    //break;
                }
                else if (chaseFlg1==1 && flg1==0) {
                    double len = sqrt(pow(touchPosX1.x-footPoint[i].x,2)+pow(touchPosX1.z-footPoint[i].z,2));
                    if (len<2.0) {
                        lightVec1.x = lightCollPos1.x-touchPosX1.x;
                        lightVec1.y = lightCollPos1.y-touchPosX1.y;
                        lightVec1.z = lightCollPos1.z-touchPosX1.z;
                        lightVec1 = vectorNormalize(lightVec1);
                        
                        double t = (lightPos1.z-touchPosX1.z)/lightVec1.z;
                        lightPos1.x = touchPosX1.x+lightVec1.x*t;
                        lightPos1.y = touchPosX1.y+lightVec1.y*t;
                        touchPosX1 = footPoint[i];
                        chaseFlg1 = 1;
                        if (lightPos1.x>lightW*0.5)
                            lightPos1.x = lightW*0.5;
                        if (lightPos1.x<-lightW*0.5)
                            lightPos1.x = -lightW*0.5;
                        if (lightPos1.y>lightH)
                            lightPos1.y = lightH;
                        if (lightPos1.y<6.0)
                            lightPos1.y = 6.0;
                        //break;
                    }
                }
                flg1 = 1;
            }
            else if (shadowColVal0[0]==128 & shadowColVal0[1]==64 & shadowColVal0[2]==191) {
                if (chaseFlg2==0 && flg2==0) {
                    touchPosX2 = footPoint[i];
                    chaseFlg2 = 1;
                    //
                    lightVec0.x = lightPos0.x-touchPosX2.x; lightVec0.y = lightPos0.y-touchPosX2.y; lightVec0.z = lightPos0.z-touchPosX2.z;
                    lightVec0 = vectorNormalize(lightVec0);
                    //printf("lightVec0 = (%f, %f, %f)\n", lightVec0.x, lightVec0.y, lightVec0.z);
                    double disp0X, disp0Y, disp0Z;
                    float disp0Zd;
                    gluProject(touchPosV.x, touchPosV.y, touchPosV.z, model0, proj0, view0, &disp0X, &disp0Y, &disp0Z);
                    glutSetWindow(winID[0]);
                    glReadPixels(disp0X, disp0Y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &disp0Zd);
                    gluUnProject(disp0X, disp0Y, disp0Zd, model0, proj0, view0, &lightCollPos0.x, &lightCollPos0.y, &lightCollPos0.z);

                    //double t0 = (objPos.z-touchPosX2.z)/lightVec0.z;
                    //lightCollPos0.x = touchPosX2.x+lightVec0.x*t0;
                    //lightCollPos0.y = touchPosX2.y+lightVec0.y*t0;
                    //
                    lightVec1.x = lightPos1.x-touchPosX2.x; lightVec1.y = lightPos1.y-touchPosX2.y; lightVec1.z = lightPos1.z-touchPosX2.z;
                    lightVec1 = vectorNormalize(lightVec1);

                    double disp1X, disp1Y, disp1Z;
                    float disp1Zd;
                    gluProject(touchPosV.x, touchPosV.y, touchPosV.z, model1, proj1, view1, &disp1X, &disp1Y, &disp1Z);
                    glutSetWindow(winID[1]);
                    glReadPixels(disp1X, disp1Y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &disp1Zd);
                    gluUnProject(disp1X, disp1Y, disp1Zd, model1, proj1, view1, &lightCollPos1.x, &lightCollPos1.y, &lightCollPos1.z);

                    //printf("lightVec1 = (%f, %f, %f)\n", lightVec1.x, lightVec1.y, lightVec1.z);
                    //double t1 = (objPos.z-touchPosX2.z)/lightVec1.z;
                    //lightCollPos1.x = touchPosX2.x+lightVec1.x*t1;
                    //lightCollPos1.y = touchPosX2.y+lightVec1.y*t1;
                    
                    //printf("%f, %f\n", touchPosX2.x, touchPosX2.z);
                    //break;
                }
                else if (chaseFlg2==1 && flg2==0) {
                    double len = sqrt(pow(touchPosX2.x-footPoint[i].x,2)+pow(touchPosX2.z-footPoint[i].z,2));
                    if (len<2.0) {
                        lightVec0.x = lightCollPos0.x-touchPosX2.x;
                        lightVec0.y = lightCollPos0.y-touchPosX2.y;
                        lightVec0.z = lightCollPos0.z-touchPosX2.z;
                        lightVec0 = vectorNormalize(lightVec0);

                        lightVec1.x = lightCollPos1.x-touchPosX2.x;
                        lightVec1.y = lightCollPos1.y-touchPosX2.y;
                        lightVec1.z = lightCollPos1.z-touchPosX2.z;
                        lightVec1 = vectorNormalize(lightVec1);
                        
                        double t0 = (lightPos0.z-touchPosX2.z)/lightVec0.z;
                        lightPos0.x = touchPosX2.x+lightVec0.x*t0;
                        lightPos0.y = touchPosX2.y+lightVec0.y*t0;
                        if (lightPos0.x>lightW*0.5)
                            lightPos0.x = lightW*0.5;
                        if (lightPos0.x<-lightW*0.5)
                            lightPos0.x = -lightW*0.5;
                        if (lightPos0.y>lightH)
                            lightPos0.y = lightH;
                        if (lightPos0.y<6.0)
                            lightPos0.y = 6.0;

                        double t1 = (lightPos1.z-touchPosX2.z)/lightVec1.z;
                        lightPos1.x = touchPosX2.x+lightVec1.x*t1;
                        lightPos1.y = touchPosX2.y+lightVec1.y*t1;
                        if (lightPos1.x>lightW*0.5)
                            lightPos1.x = lightW*0.5;
                        if (lightPos1.x<-lightW*0.5)
                            lightPos1.x = -lightW*0.5;
                        if (lightPos1.y>lightH)
                            lightPos1.y = lightH;
                        if (lightPos1.y<6.0)
                            lightPos1.y = 6.0;

                        touchPosX2 = footPoint[i];
                        
                        chaseFlg2 = 1;
                        flg2 = 1;
                        //break;
                    }
                }
                flg2 = 1;
            }
        }
        
        if (flg0==0) {
            chaseFlg0 = 0;
        }
        if (flg1==0) {
            chaseFlg1 = 0;
        }
        if (flg2==0) {
            chaseFlg2 = 0;
        }
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
