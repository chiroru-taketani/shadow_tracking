//g++ -O3 -std=c++11 main0.cpp -framework OpenGL -framework GLUT `pkg-config --cflags --libs opencv4` -Wno-deprecated
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <GLUT/glut.h>  //OpenGL/GLUTの使用
#include <opencv2/opencv.hpp>  //OpenCV関連ヘッダ

//定数宣言
#define TEX_SIZE 1024  //テクスチャサイズ

//三次元ベクトル構造体
typedef struct _Vec_3D
{
    double x, y, z;
    double s, t;
    int id;
} Vec_3D;

//関数名の宣言
void initGL();
void display0();
void display1();
void reshape0(int w, int h);
void reshape1(int w, int h);
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
int winID[2];  //ウィンドウID
double eDist[2], eDegX[2], eDegY[2];  //視点極座標
int mX[2], mY[2], mState[2], mButton[2];  //マウス座標
int winW[2], winH[2];  //ウィンドウサイズ
double fr = 30.0;
Vec_3D objPos;  //影物体
Vec_3D lightPos0;  //LED光源座標
int dispMode = 1;
double scanW = 300.0, scanH = 200.0;

cv::Mat frameImage0, frameImage, frameImage1, shadowAreaImage, shadowAreaGrayImage;

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
    //視点関係 照明とカメラの位置⭐︎
    lightPos0.x = 0.0; lightPos0.y = 100.0; lightPos0.z = 100.0;

    //描画ウィンドウ生成1（客観視点映像の生成）
    glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);  //ディスプレイモードの指定
    glutInitWindowSize(scanW*3, scanH*3);  //ウィンドウサイズの指定
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
    eDist[1] = 250.0; eDegX[1] = 30.0; eDegY[1] = 0.0;  //視点極座標

    //光源やテクスチャの初期設定
    cv::Mat textureImage;
    for (int i=0; i<2; i++) {
        glutSetWindow(winID[i]);  //設定対象ウィンドウ

        //光源設定
        GLfloat col[4] = {0.8, 0.8, 0.8, 1.0};  //パラメータ(RGBA)
        glEnable(GL_LIGHTING);  //陰影付けの有効化
        glEnable(GL_LIGHT0);  //光源0の有効化
        glLightfv(GL_LIGHT0, GL_DIFFUSE, col);  //光源0の拡散反射の強度
        glLightfv(GL_LIGHT0, GL_SPECULAR, col);  //光源0の鏡面反射の強度
        col[0] = 0.2; col[1] = 0.2; col[2] = 0.2; col[3] = 1.0;
        glLightfv(GL_LIGHT0, GL_AMBIENT, col);  //光源0の環境光の強度

        //テクスチャ
        //テクスチャオブジェクト生成(#100)
        textureImage = cv::imread("friends.png", cv::IMREAD_UNCHANGED);
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
}

//ユーザに知覚させたいオブジェクトの描画
void scene()
{
    //オブジェクト（本体）
    glDisable(GL_LIGHTING);
    glColor4d(1.0, 1.0, 1.0, 1.0);
    glPushMatrix();
    glTranslated(0.0, 40.0*objPos.t/objPos.s, 0.0);
    glScaled(80.0, 80.0*objPos.t/objPos.s, 1.0);
    glMyTexPlane(objPos.id, objPos.s/50, objPos.t/50);
    glPopMatrix();
        
    //LED光源位置
    glDisable(GL_LIGHTING);
    glColor4d(0.0, 1.0, 0.0, 1.0);
    glPushMatrix();
    glTranslated(lightPos0.x, lightPos0.y, lightPos0.z);
    glScaled(2.5, 2.5, 2.5);
    glutSolidSphere(1.0, 36, 18);
    glPopMatrix();
}

//ディスプレイコールバック関数
void display0()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);  //画面消去
    
    //投影変換の設定
    glMatrixMode(GL_PROJECTION);  //変換行列の指定（設定対象は投影変換行列）
    glLoadIdentity();  //行列初期化
    gluPerspective(60.0, (double)winW[0]/(double)winH[0], 50.0, 1000.0);  //透視投影ビューボリューム設定
    
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
    gluPerspective(60.0, (GLdouble)winW[0]/(GLdouble)winH[0], 50.0, 1000.0);  //display0と同じ投影変換
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
    cv::imshow("shadowAreaImage", shadowAreaGrayImage);
    cv::rotate(shadowAreaGrayImage, shadowAreaGrayImage, cv::ROTATE_90_COUNTERCLOCKWISE);
    cv::imwrite("../ShadowImg/ShadowArea.png", shadowAreaGrayImage);

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
