// LiDAR (Paper Sec. 2.2 Specification: 10-frame accumulation, area range, top-2 selection, inter-frame distance labeling)
/*
 * g++ -O3 main_cv.cpp -std=c++11 $(pkg-config --cflags --libs opencv5) -framework OpenGL -framework GLUT Connection_information.cpp liburg_cpp.a -Wno-deprecated -o a.out
 */
#include "Connection_information.h" // URGセンサーの接続情報用
#include "Urg_driver.h"             // URGセンサー用ドライバ
#include <GLUT/glut.h>              // グラフィックスライブラリ GLUT (OpenGLの補助)
#include <algorithm>                // std::sort 用
#include <deque>                    // 10フレーム蓄積用のリングバッファ
#include <iostream>                 // 標準入出力
#include <math.h>                   // 数学関数 (sin, cos, sqrt, tan, M_PI)
#include <opencv2/geometry.hpp>     // OpenCV 5: moments, contourArea 用
#include <opencv2/opencv.hpp>       // 画像処理ライブラリ OpenCV
#include <time.h>                   // 時間関連
#include <vector>

// URGセンサーのモデルに応じてデータ数を定義
#define DATASIZE 513 // UBG-04LX-F01

// 三次元ベクトル構造体
typedef struct _Vec_3D
{
    double x, y, z;
} Vec_3D;

// 候補領域（距離ソート用）
struct HandCandidate
{
    int contourIdx;
    cv::Point pixelCentroid;
    Vec_3D worldPos;
    double distFromPanel;
};

// フレーム間距離ラベリング追跡用の構造体
struct TrackedHand
{
    int id;              // ラベルID (0 or 1)
    Vec_3D pos;          // 実世界座標 (cm)
    cv::Point pixelPos;  // 描画用ピクセル座標
    int contourIdx;      // 今フレームの対応輪郭インデックス
};

//---- 関数プロトタイプ宣言 ----
void display();
void reshape(int w, int h);
void timer(int value);
void keyboard(unsigned char key, int x, int y);
void initGL();

//---- グローバル変数 ----
double fr = 35; // フレームレート(fps): 論文記述およびUBG-04LX-F01仕様(35fps)に合わせる
GLdouble model[16], proj[16];
GLint view[4];

qrk::Urg_driver urg;
std::vector<long> distData;

//---- スキャン・パラメータ ----
int scanAngle = 160;            // スキャン角度範囲 (±80度)
double scanAngleStep = 0.35;    // UBG-04LX-F01 角度分解能
double minObjectSize = 600.0;   // 手領域の最小面積閾値 (px)
double maxObjectSize = 25000.0; // 手領域の最大面積閾値 (px)
const int ACCUM_FRAMES = 10;    // 蓄積する最新フレーム数 (10フレーム)
const int MAX_HANDS = 1;        // 選択する最大手領域数 (最大2つ)
double maxTrackDist = 20.0;     // フレーム間同一ラベル判定の最大距離閾値 (cm)


//---- 揺れ(ブルブル)抑制 & 遅延解消パラメータ ----
double accumNormFactor = 0.75; // 蓄積画像の正規化係数 (0.4 -> 0.75: 古い残像のしっぽを切って遅延を解消)
double deadbandDist = 0.15;    // 静止判定の微小移動閾値 (cm): 1.5mm未満の揺れは完全に無視してピタッと止める
double fastMoveDist = 1.5;     // 高速移動判定距離 (cm): 1.5cm以上の移動時は平滑化重みを0にして遅延ゼロにする
double maxSmoothAlpha = 0.80;  // 低速移動時の過去座標の最大重み (0.80 = 過去80% : 最新20%)


double scanAreaW, scanAreaH;
double scanReso;
cv::Mat scanImage, currentBinImage, accumImage, binImage;
std::deque<cv::Mat> frameHistory; // 最新10フレームの二値画像バッファ
cv::Mat element;

// フレーム間追跡状態
std::vector<TrackedHand> trackedHands;
int nextHandId = 0;

//---- ファイル関連 ----
FILE *fp;
char areaSizeFile[] = "scanarea.txt";
char footPointFile[] = "footpoint.txt";

double rDisp = 1.0;

//============================================================
// メイン関数
//============================================================
int main(int argc, char *argv[])
{
    glutInit(&argc, argv);

    qrk::Connection_information information(argc, argv);
    if (!urg.open(information.device_or_ip_name(),
                  information.baudrate_or_port_number(),
                  information.connection_type()))
    {
        printf("URGの接続に失敗しました!\n");
        return 1;
    }
    urg.set_scanning_parameter(urg.deg2step(-scanAngle / 2), urg.deg2step(scanAngle / 2), 0);
    urg.start_measurement(qrk::Urg_driver::Distance, qrk::Urg_driver::Infinity_times, 0);

    initGL();
    glutMainLoop();

    return 0;
}

//============================================================
// GL初期設定関数
//============================================================
void initGL()
{
    fp = fopen(areaSizeFile, "r");
    if (fp == NULL)
    {
        printf("scanarea.txtファイルが開けません\n");
    }
    else
    {
        fscanf(fp, "%lf,%lf,%lf", &scanAreaW, &scanAreaH, &scanReso);
        fclose(fp);
    }

    cv::Size imgSize(scanAreaW / scanReso, scanAreaH / scanReso);
    scanImage = cv::Mat(imgSize, CV_8UC3);
    currentBinImage = cv::Mat(imgSize, CV_8UC1);
    accumImage = cv::Mat(imgSize, CV_32FC1);
    binImage = cv::Mat(imgSize, CV_8UC1);

    glutInitWindowSize(imgSize.width, imgSize.height);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
    glutInitWindowPosition(0, 0);
    glutCreateWindow("CG");

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutTimerFunc(1000 / fr, timer, 0);

    glClearColor(0.0, 0.0, 0.0, 1.0);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_ALPHA_TEST);
    glAlphaFunc(GL_GREATER, 0.01);

    cv::Mat textureImage = cv::imread("mark.png", cv::IMREAD_UNCHANGED);
    if (!textureImage.empty())
    {
        glBindTexture(GL_TEXTURE_2D, 0);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexImage2D(GL_TEXTURE_2D,
                     0,
                     GL_RGBA,
                     textureImage.cols,
                     textureImage.rows,
                     0,
                     GL_BGRA,
                     GL_UNSIGNED_BYTE,
                     textureImage.data);
    }
    else
    {
        printf("mark.png not found!\n");
    }

    element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));

    cv::namedWindow("binImage");
    cv::moveWindow("binImage", 500, 0);
    cv::namedWindow("Scan");
    cv::moveWindow("Scan", 1000, 0);
}

//============================================================
// ディスプレイコールバック関数 - 描画・解析処理
//============================================================
void display()
{
    //---- [1] 現フレームのLiDARスキャン点をOpenGLで描画 ----
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    glEnable(GL_TEXTURE_2D);
    glColor4d(1.0, 1.0, 1.0, 1.0);
    glBindTexture(GL_TEXTURE_2D, 0);

    for (int i = 0; i < (int)distData.size(); i++)
    {
        double len = distData[i] * 0.1; // mm -> cm
        if (len < 3.0)
        {
            continue;
        }

        double rad = urg.index2rad(i);
        double x = -len * sin(rad);
        double y = len * cos(rad);
        double z = 0.0;

        double circleR = 40.0 * sqrt(len) * tan(scanAngleStep * M_PI / 180.0 / 2);

        glPushMatrix();
        glTranslated(x, y, z);
        glRotated(rad * 180.0 / M_PI, 0.0, 0.0, 1.0);
        glScaled(circleR * 1.5, circleR * 1.0, 1);
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
    glutSwapBuffers();

    //---- [2] 最新10フレームの二値画像蓄積・減衰とノイズ除去 ----
    glReadPixels(0, 0, scanImage.cols, scanImage.rows, GL_BGR, GL_UNSIGNED_BYTE, scanImage.data);
    cv::flip(scanImage, scanImage, 0);
    cv::cvtColor(scanImage, currentBinImage, cv::COLOR_BGR2GRAY);
    cv::threshold(currentBinImage, currentBinImage, 128, 255, cv::THRESH_BINARY);

    // 最新10フレームのキューに追加
    frameHistory.push_front(currentBinImage.clone());
    if ((int)frameHistory.size() > ACCUM_FRAMES)
    {
        frameHistory.pop_back();
    }

    // 10フレームを段階的に減衰させながら蓄積 (最新フレーム重み 1.0 -> 10フレーム前 0.1)
    accumImage.setTo(0.0f);
    double weightSum = 0.0;
    for (int f = 0; f < (int)frameHistory.size(); f++)
    {
        double weight = (double)(ACCUM_FRAMES - f) / (double)ACCUM_FRAMES;
        cv::Mat floatFrame;
        frameHistory[f].convertTo(floatFrame, CV_32FC1, weight);
        accumImage += floatFrame;
        weightSum += weight;
    }
    // 0〜255に正規化して8bit画像へ戻す
    accumImage.convertTo(binImage, CV_8UC1, 1.0 / (weightSum * accumNormFactor));

    // 蓄積画像の閾値処理と小領域除去（モルフォロジー処理）
    cv::threshold(binImage, binImage, 128, 255, cv::THRESH_BINARY);
    cv::dilate(binImage, binImage, element, cv::Point(-1, -1), 2);
    cv::erode(binImage, binImage, element, cv::Point(-1, -1), 4);
    cv::dilate(binImage, binImage, element, cv::Point(-1, -1), 2);
    cv::erode(binImage, binImage, element, cv::Point(-1, -1), 2);
    cv::dilate(binImage, binImage, element, cv::Point(-1, -1), 5);

    cv::imshow("binImage", binImage);

    //---- [3] 面積範囲フィルタ・重心計算・最近傍2領域の選択 ----
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binImage, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<HandCandidate> candidates;
    for (int i = 0; i < (int)contours.size(); i++)
    {
        double area = cv::contourArea(contours[i]);
        // 面積が定義範囲内 (minObjectSize <= area <= maxObjectSize) の領域を抽出
        if (area >= minObjectSize && area <= maxObjectSize)
        {
            cv::Moments m = cv::moments(contours[i]);
            if (m.m00 == 0.0)
                continue;
            cv::Point p(m.m10 / m.m00, m.m01 / m.m00);

            Vec_3D pointF;
            gluUnProject(p.x, binImage.rows - p.y, 0, model, proj, view, &pointF.x, &pointF.y, &pointF.z);

            double distFromPanel = sqrt(pointF.x * pointF.x + pointF.y * pointF.y);
            candidates.push_back({i, p, pointF, distFromPanel});
        }
    }

    // LEDパネル（センサ）に近い順にソートし、最大2領域を選択
    std::sort(candidates.begin(),
              candidates.end(),
              [](const HandCandidate &a, const HandCandidate &b) {
                  return a.distFromPanel < b.distFromPanel;
              });
    if ((int)candidates.size() > MAX_HANDS)
    {
        candidates.resize(MAX_HANDS);
    }

    //---- [4] フレーム間距離に基づくラベリング追跡 ----
    std::vector<TrackedHand> newTrackedHands;
    std::vector<bool> prevUsed(trackedHands.size(), false);

    for (int c = 0; c < (int)candidates.size(); c++)
    {
        int bestPrevIdx = -1;
        double minDist = maxTrackDist;

        for (int p = 0; p < (int)trackedHands.size(); p++)
        {
            if (prevUsed[p])
                continue;
            double dx = candidates[c].worldPos.x - trackedHands[p].pos.x;
            double dy = candidates[c].worldPos.y - trackedHands[p].pos.y;
            double d = sqrt(dx * dx + dy * dy);
            if (d < minDist)
            {
                minDist = d;
                bestPrevIdx = p;
            }
        }

        TrackedHand th;
       if (bestPrevIdx != -1)
        {
            prevUsed[bestPrevIdx] = true;
            th.id = trackedHands[bestPrevIdx].id; // 前フレームのラベルIDを継承

            // 速度適応型スムージング（静止時のブルブル抑制と高速移動時の低遅延を両立）
            Vec_3D prevPos = trackedHands[bestPrevIdx].pos;
            Vec_3D rawPos = candidates[c].worldPos;
            if (minDist < deadbandDist)
            {
                // 1.5mm未満の微小な揺れはノイズとみなして前回座標を維持（ブルブル完全停止）
                th.pos = prevPos;
            }
            else if (minDist < fastMoveDist)
            {
                // 低速〜中速移動時は移動量に応じて過去重み alpha を 0.80 -> 0.0 に線形補間
                double ratio = (minDist - deadbandDist) / (fastMoveDist - deadbandDist);
                double alpha = maxSmoothAlpha * (1.0 - ratio);
                th.pos.x = prevPos.x * alpha + rawPos.x * (1.0 - alpha);
                th.pos.y = prevPos.y * alpha + rawPos.y * (1.0 - alpha);
                th.pos.z = prevPos.z * alpha + rawPos.z * (1.0 - alpha);
            }
            else
            {
                // 1.5cm以上の素早い移動時は遅延ゼロで最新座標に追従
                th.pos = rawPos;
            }
        }
        else{
            th.id = nextHandId++; // 新規ラベルIDを付与
            th.pos = candidates[c].worldPos;
        }
        th.pixelPos = candidates[c].pixelCentroid;
        th.contourIdx = candidates[c].contourIdx;
        newTrackedHands.push_back(th);
    }

    // ラベルID順に整列して出力順序を安定化
    std::sort(newTrackedHands.begin(),
              newTrackedHands.end(),
              [](const TrackedHand &a, const TrackedHand &b) { return a.id < b.id; });
    trackedHands = newTrackedHands;


    // 検出・追跡結果の描画
    for (int i = 0; i < (int)trackedHands.size(); i++)
    {
        cv::Scalar color = (i == 0) ? cv::Scalar(255, 0, 0) : cv::Scalar(0, 255, 0);
        cv::drawContours(scanImage, contours, trackedHands[i].contourIdx, color, 2);
        cv::circle(scanImage, trackedHands[i].pixelPos, 5, cv::Scalar(0, 0, 255), -1);
        cv::putText(scanImage,
                    "ID:" + std::to_string(trackedHands[i].id),
                    trackedHands[i].pixelPos + cv::Point(8, -8),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    cv::Scalar(0, 255, 255),
                    1);
    }

    printf("%ld 個の手領域を追跡中\n", trackedHands.size());
    printf("===========\n");

    //---- [5] 結果を footpoint.txt に出力 ----
    fp = fopen(footPointFile, "w");
    fprintf(fp, "%ld\n", trackedHands.size());
    for (int i = 0; i < (int)trackedHands.size(); i++)
    {
        fprintf(fp, "%f,%f\n", trackedHands[i].pos.x, trackedHands[i].pos.y);
    }
    fclose(fp);

    cv::imshow("Scan", scanImage);
}

//============================================================
// リシェイプコールバック関数
//============================================================
void reshape(int w, int h)
{
    glViewport(0, 0, w * rDisp, h * rDisp);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-scanAreaW * 0.5, scanAreaW * 0.5, 0.0, scanAreaH, -100, 100);
    glMatrixMode(GL_MODELVIEW);

    glGetDoublev(GL_MODELVIEW_MATRIX, model);
    glGetDoublev(GL_PROJECTION_MATRIX, proj);
    glGetIntegerv(GL_VIEWPORT, view);
    for (int i = 0; i < 4; i++)
    {
        view[i] /= rDisp;
    }
}

//============================================================
// タイマーコールバック関数
//============================================================
void timer(int value)
{
    glutTimerFunc(1000 / fr, timer, 0);

    long time_stamp = 0;
    for (int i = 0; i < 2; i++)
    {
        urg.get_distance(distData, &time_stamp);
    }

    glutPostRedisplay();
}

//============================================================
// キーボードコールバック関数
//============================================================
void keyboard(unsigned char key, int x, int y)
{
    switch (key)
    {
    case 27:
        urg.close();
        exit(0);
        break;
    default:
        break;
    }
}
