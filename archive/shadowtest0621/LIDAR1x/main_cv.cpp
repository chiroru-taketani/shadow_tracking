//LiDAR
/*
g++ -O3 main_cv.cpp -std=c++11 `pkg-config --cflags --libs opencv4` -framework OpenGL -framework GLUT Connection_information.cpp liburg_cpp_intel.a -Wno-deprecated
*/
#include <iostream>
#include <time.h>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <GLUT/glut.h>
#include "Urg_driver.h"
#include "Connection_information.h"

//三次元ベクトル構造体: Vec_3D
typedef struct _Vec_3D
{
    double x, y, z;
} Vec_3D;

//関数名の宣言
void display();
void reshape(int w, int h);
void timer(int value);
void keyboard(unsigned char key, int x, int y);
void initGL();

//グローバル変数
double fr = 60;
GLdouble model[16], proj[16];
GLint view[4];

//URG
qrk::Urg_driver urg;
std::vector<long> distData, distData0;

//スキャン設定
int scanAngle = 180;  //角度
double scanAngleStep = 0.35;  //スキャンステップ角（UBG-04LX-F01）
//double scanAngleStep = 0.125;  //スキャンステップ角（UST-20LX-H01）
double scanAreaW, scanAreaH;  //領域サイズ（cm）
double lidarPosY;
double scanReso;  //1画素あたりのサイズ（cm）
cv::Mat scanImage, binImage, scanImage0;
cv::Mat element;

//ファイル
FILE *fp;
char areaSizeFile[] = "scanarea.txt";
char footPointFile[] = "footpoint.txt";

double rDisp = 1.0;  //Retina

//メイン関数
int main(int argc, char* argv[])
{
    glutInit(&argc, argv);  //OpenGL初期化
    
    //URG
    qrk::Connection_information information(argc, argv);
    //接続
    if (!urg.open(information.device_or_ip_name(), information.baudrate_or_port_number(),information.connection_type())) {
        printf("URG failed!\n");
        return 1;
    }
    urg.set_scanning_parameter(urg.deg2step(-scanAngle/2), urg.deg2step(scanAngle/2), 0);  //スキャン角度範囲
    urg.start_measurement(qrk::Urg_driver::Distance, qrk::Urg_driver::Infinity_times, 0);  //スキャン開始
    
    printf("LiDAR OK\n");

    initGL();  //GL初期設定
    
    glutMainLoop();  //イベント待ち無限ループ突入
    
    return 0;
}

//GL初期設定
void initGL()
{
    fp = fopen(areaSizeFile, "r");
    fscanf(fp, "%lf,%lf,%lf,%lf", &scanAreaW, &scanAreaH, &scanReso, &lidarPosY);
    fclose(fp);
    printf("scanAreaW = %f, scanAreaH = %f, scanReso = %f, lidarPosY = %f\n", scanAreaW, scanAreaH, scanReso, lidarPosY);
    scanImage = cv::Mat(cv::Size(scanAreaW/scanReso, scanAreaH/scanReso), CV_8UC3);
    binImage = cv::Mat(cv::Size(scanAreaW/scanReso, scanAreaH/scanReso), CV_8UC1);
    scanImage0 = cv::Mat(cv::Size(scanAreaW/scanReso*rDisp, scanAreaH/scanReso*rDisp), CV_8UC3);
    cv::namedWindow("Bin");
    cv::moveWindow("Bin", 0,scanAreaH/scanReso);
    cv::namedWindow("Scan");
    cv::moveWindow("Scan", scanAreaW/scanReso,scanAreaH/scanReso);

    //描画ウィンドウ生成
    glutInitWindowSize(scanAreaW/scanReso, scanAreaH/scanReso);  //ウィンドウサイズ指定
    glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);  //ディスプレイモード設定
    glutInitWindowPosition(0, 0);
    glutCreateWindow("CG");  //ウィンドウ生成
    
    //コールバック関数
    glutDisplayFunc(display);  //ディスプレイ
    glutReshapeFunc(reshape);  //リシェイプ
    glutKeyboardFunc(keyboard);  //キーボード
    glutTimerFunc(1000/fr, timer, 0);  //タイマー
    
    //背景色
    glClearColor(0.0, 0.0, 0.0, 1.0);
    //Zバッファの有効化
    //glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);  //アルファブレンド有効化
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);  //ブレンド方法の設定
    glEnable(GL_ALPHA_TEST);  //アルファテスト有効化
    glAlphaFunc(GL_GREATER, 0.01);

    cv::Mat textureImage = cv::imread("mark.png", cv::IMREAD_UNCHANGED);
    glBindTexture(GL_TEXTURE_2D, 0);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, textureImage.cols, textureImage.rows, 0, GL_BGRA, GL_UNSIGNED_BYTE, textureImage.data);
    
    element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5));
}

void display()
{
    //画面消去
    //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    //行列初期化
    glLoadIdentity();

    //スキャン点CG描画色
    glEnable(GL_TEXTURE_2D);
    glColor4d(1.0, 1.0, 1.0, 0.05);
    glBindTexture(GL_TEXTURE_2D, 0);
    for (int i=0; i<distData.size(); i++) {
        //スキャン点情報を座標に変換
        double len = distData[i]*0.1;
        double rad = urg.index2rad(i);  //i番目のスキャン点の角度(ラジアン)
        double x = -len*sin(rad);  //i番目のスキャン点のx座標
        double y = len*cos(rad)-lidarPosY;  //i番目のスキャン点のz座標
        double z = 0;
        
        double circleR = 6.0*sqrt(len)*tan(scanAngleStep*M_PI/180.0/scanReso);  //スキャン画像作成用円半径
        
        //スキャン点CG描画
        glPushMatrix();
        glTranslated(x, y, z);
        glRotated(rad*180.0/M_PI, 0.0, 0.0, 1.0);
        glScaled(circleR, circleR, 1);
        glBegin(GL_QUADS);
        glTexCoord2d(0, 0);
        glVertex3d(-0.5, 0.5, 0.0);
        glTexCoord2d(0, 1);
        glVertex3d(-0.5, -0.5, 0.0);
        glTexCoord2d(1, 1);
        glVertex3d(0.5, -0.5, 0.0);
        glTexCoord2d(1, 0);
        glVertex3d(0.5, 0.5, 0.0);
        glEnd();
        glPopMatrix();
    }
    glDisable(GL_TEXTURE_2D);
    
    glColor4d(0.0, 0.0, 0.0, 0.15);
    glTranslated(0.0, scanAreaH*0.5, 0.0);
    glScaled(scanAreaW, scanAreaH, 1.0);
    glBegin(GL_QUADS);
    glVertex3d(-0.5, 0.5, 0.0);
    glVertex3d(-0.5, -0.5, 0.0);
    glVertex3d(0.5, -0.5, 0.0);
    glVertex3d(0.5, 0.5, 0.0);
    glEnd();

    glutSwapBuffers();

    glReadPixels(0, 0, scanImage0.cols, scanImage0.rows, GL_BGR, GL_UNSIGNED_BYTE, scanImage0.data);
    cv::resize(scanImage0, scanImage, scanImage.size());
    cv::flip(scanImage, scanImage, 0);
    cv::cvtColor(scanImage, binImage, cv::COLOR_BGR2GRAY);
    cv::threshold(binImage, binImage, 128, 255, cv::THRESH_BINARY);
    //cv::dilate(binImage, binImage, element, cv::Point(-1,-1), 4);
    //cv::erode(binImage, binImage, element, cv::Point(-1,-1), 2);
    //cv::dilate(binImage, binImage, element, cv::Point(-1,-1), 2);

    cv::imshow("Bin", binImage);

    std::vector< std::vector<cv::Point> > contours;
    cv::findContours(binImage, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    std::vector<Vec_3D> footPoints;
    for (int i=0; i<contours.size(); i++) { //検出された領域数だけループ
        double area = contourArea(contours[i]);
        if (area>50) {
            cv::Moments m = cv::moments(contours[i]);
            cv::Point p = cv::Point(m.m10/m.m00 , m.m01/m.m00);
            cv::circle(scanImage, cv::Point(p.x , p.y), 5, cv::Scalar(0,0,255), -1);
            
            Vec_3D pointF;
            gluUnProject(p.x, binImage.rows-p.y, 0, model, proj, view, &pointF.x, &pointF.y, &pointF.z);
            footPoints.push_back(pointF);
            printf("%f, %f\n", pointF.x, pointF.y);
        }
    }
    printf("%ld\n", footPoints.size());
    printf("===========\n");
    
    fp = fopen(footPointFile, "w");
    fprintf(fp, "%ld\n", footPoints.size());
    for (int i=0; i<footPoints.size(); i++) {
        fprintf(fp, "%f,%f\n", footPoints[i].x, footPoints[i].y);
    }
    fclose(fp);
    
    cv::imshow("Scan", scanImage);
}

//リシェイプコールバック関数
void reshape(int w, int h)
{
    glViewport(0, 0, w*rDisp, h*rDisp);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-scanAreaW*0.5, scanAreaW*0.5, 0.0, scanAreaH, -100, 100);
    glMatrixMode(GL_MODELVIEW);
    
    glGetDoublev(GL_MODELVIEW_MATRIX, model);  //モデルビュー変換行列を"model[]"に格納
    glGetDoublev(GL_PROJECTION_MATRIX, proj);  //投影変換行列を"proj[]"に格納
    glGetIntegerv(GL_VIEWPORT, view);  //ビューポート設定を"view[]"に格納
    for (int i=0; i<4; i++) {
        view[i] /= rDisp;
    }
}

//タイマーコールバック関数
void timer(int value)
{
    glutTimerFunc(1000/fr, timer, 0);  //タイマー再設定

    //LiDAR
    long time_stamp = 0;
    for (int i=0; i<2; i++) {
        urg.get_distance(distData, &time_stamp);  //センサから値を取得→distData
        //printf("%ld\n", time_stamp);
    }
    
    //ディスプレイイベント発生
    glutPostRedisplay();  //ディスプレイイベント強制発生
}

//キーボードコールバック関数(key:キーの種類，x,y:座標)
void keyboard(unsigned char key, int x, int y)
{
    switch (key) {
        case 27:
            urg.close();
            exit(0);
            break;
            
        default:
            break;
    }
}
