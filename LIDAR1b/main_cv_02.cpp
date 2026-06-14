// LiDAR
/*
 * g++ -O3 main_cv.cpp -std=c++11 `pkg-config --cflags --libs opencv4` -framework OpenGL -framework GLUT Connection_information.cpp liburg_cpp.a -Wno-deprecated
 */
#include <iostream>
#include <time.h>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <GLUT/glut.h>

//URGセンサー用
#include "Urg_driver.h"
#include "Connection_information.h"

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
