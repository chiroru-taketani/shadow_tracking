// LiDAR
/*
 * g++ -O3 main_cv_02.cpp -std=c++11 `pkg-config --cflags --libs opencv4` -framework OpenGL -framework GLUT Connection_information.cpp liburg_cpp.a -Wno-deprecated
 */
#include "Connection_information.h" // URGセンサーの接続情報用
#include "Urg_driver.h"             // URGセンサー用ドライバ
#include <GLUT/glut.h>              // グラフィックスライブラリ GLUT (OpenGLの補助)
#include <iostream>                 // 標準入出力
#include <math.h>                   // 数学関数 (sin, cos, sqrt, tan, M_PI)
#include <opencv2/opencv.hpp>       // 画像処理ライブラリ OpenCV
#include <time.h>                   // 時間関連

// URGセンサーのモデルに応じてデータ数を定義
// #define DATASIZE 1440 // 例: UST-20LX-H01
#define DATASIZE 513 // 例: UBG-04LX-F01

// 三次元ベクトルを格納するための構造体: Vec_3D
typedef struct _Vec_3D
{
    double x, y, z;
} Vec_3D;

//---- 関数プロトタイプ宣言 ----
void display();                                 // 描画処理を行うコールバック関数
void reshape(int w, int h);                     // ウィンドウサイズ変更時に呼ばれるコールバック関数
void timer(int value);                          // 一定時間ごとに呼ばれるコールバック関数
void keyboard(unsigned char key, int x, int y); // キーボード入力時に呼ばれるコールバック関数
void initGL();                                  // OpenGLおよび各種設定の初期化を行う関数

//---- グローバル変数 ----
double fr = 60; // フレームレート(fps)
// OpenGLの座標変換行列を格納する変数 (gluUnProjectで使用)
GLdouble model[16], proj[16];
GLint view[4];

// URGセンサーのドライバインスタンス
qrk::Urg_driver urg;
// URGから取得した距離データ(mm)を格納するベクター
std::vector<long> distData;

//---- スキャン設定 ----
int scanAngle = 160;         // スキャンする角度の範囲 (正面を0度として±90度)
double scanAngleStep = 0.35; // URGセンサーの1ステップあたりの角度 (UBG-04LX-F01)
double objectSize = 600;     // 物体の最小面積 (ピクセル)
// double scanAngleStep = 0.125;  // (UST-20LX-H01の場合)
double scanAreaW, scanAreaH; // 描画・処理する領域のサイズ(cm) (ファイルから読み込む)
double scanReso;             // 1画素あたりのサイズ(cm) (ファイルから読み込む)
cv::Mat scanImage, binImage; // OpenCVで画像処理を行うための画像データ格納用
cv::Mat element;             // モルフォロジー処理(膨張・収縮)で使用する構造要素

//ノイズ除去のスムージング用関数
Vec_3D tmpPoint = {0, 0, 0}; // 一時保存用座標
bool isFirst = true;         // 初回フラグ
double smoothFactor = 0.3;   // 0.0~1.0 (大きいほど滑らかになるが遅延が発生)
double moveThreshold = 0.6;  //最小移動距離の閾値


//---- ファイル関連 ----
FILE *fp;                               // ファイルポインタ
char areaSizeFile[] = "scanarea.txt";   // スキャン領域の設定ファイル名
char footPointFile[] = "footpoint.txt"; // 検出結果の出力ファイル名

double rDisp = 1.0; // Retinaディスプレイなど高解像度ディスプレイ用の補正値

//============================================================
// メイン関数 - プログラムのエントリーポイント
//============================================================
int main(int argc, char *argv[])
{
    // OpenGL(GLUT)ライブラリの初期化
    glutInit(&argc, argv);

    //---- URGセンサーの準備 ----
    qrk::Connection_information information(argc, argv); // 接続情報(IPアドレス,ポート等)を準備
    // センサーへ接続
    if (!urg.open(information.device_or_ip_name(),
                  information.baudrate_or_port_number(),
                  information.connection_type()))
    {
        printf("URGの接続に失敗しました!\n");
        return 1;
    }
    // スキャンパラメータ(スキャン範囲)を設定 (-90度から+90度まで)
    urg.set_scanning_parameter(urg.deg2step(-scanAngle / 2), urg.deg2step(scanAngle / 2), 0);

    // 距離データの計測を無限に繰り返すように設定して開始
    urg.start_measurement(qrk::Urg_driver::Distance, qrk::Urg_driver::Infinity_times, 0);

    // OpenGLおよび各種設定の初期化関数を呼び出し
    initGL();

    // GLUTのイベント処理ループを開始。これ以降、登録したコールバック関数がイベントに応じて呼ばれる
    glutMainLoop();

    return 0;
}

//============================================================
// GL初期設定関数
//============================================================
void initGL()
{
    // scanarea.txtファイルを開き、スキャン領域の幅・高さ・解像度を読み込む
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

    // 読み込んだ情報に基づき、OpenCVで使う画像領域をメモリ上に確保
    scanImage = cv::Mat(cv::Size(scanAreaW / scanReso, scanAreaH / scanReso), CV_8UC3); // カラー画像用
    binImage = cv::Mat(cv::Size(scanAreaW / scanReso, scanAreaH / scanReso), CV_8UC1);  // 白黒(2値)画像用


    //---- GLUTウィンドウの生成 ----
    glutInitWindowSize(scanAreaW / scanReso, scanAreaH / scanReso); // ウィンドウサイズ指定
    glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH |
                        GLUT_DOUBLE); // ディスプレイモード設定(RGBA色, Zバッファ, ダブルバッファ)
    glutInitWindowPosition(0, 0);     // ウィンドウの表示位置
    glutCreateWindow("CG");           // "CG"というタイトルでウィンドウを生成

    //---- コールバック関数の登録 ----
    glutDisplayFunc(display);           // 描画にはdisplay関数を使用
    glutReshapeFunc(reshape);           // ウィンドウサイズ変更時にはreshape関数を使用
    glutKeyboardFunc(keyboard);         // キーボード入力時にはkeyboard関数を使用
    glutTimerFunc(1000 / fr, timer, 0); // 1000/frミリ秒後からtimer関数を呼び出し開始

    //---- OpenGLの描画設定 ----
    glClearColor(0.0, 0.0, 0.0, 1.0); // 背景色を黒に設定
    //glEnable(GL_DEPTH_TEST);                           // 深度テストを有効化 (3D描画で前後関係を正しく描画するため)
    glEnable(GL_BLEND);                                // アルファブレンディング(透過処理)を有効化
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // ブレンド方法の設定
    glEnable(GL_ALPHA_TEST);                           // アルファテストを有効化
    glAlphaFunc(GL_GREATER, 0.01);                     // 透明度が0.01より大きいピクセルのみ描画

    //---- テクスチャの読み込み ----
    // スキャン点を描画するための画像(mark.png)を読み込む
    cv::Mat textureImage = cv::imread("mark.png", cv::IMREAD_UNCHANGED);
    if (!textureImage.empty())
    {
        glBindTexture(GL_TEXTURE_2D, 0); // テクスチャID 0番を有効化
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

    // モルフォロジー処理で使用する構造要素(5x5の楕円形)を作成
    element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));

    //確認用ウィンドウの生成
    cv::namedWindow("binImage");
    cv::moveWindow("binImage", 500, 0);
    cv::namedWindow("Scan");
    cv::moveWindow("Scan", 1000, 0);
}

//============================================================
// ディスプレイコールバック関数 - 描画処理の本体
//============================================================
void display()
{
    //---- [1] LiDARデータをOpenGLで描画 ----
    //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // 画面(カラーバッファとデプスバッファ)をクリア

    glLoadIdentity();              // 座標変換行列を初期化
    glColor4d(0.0, 0.0, 0.0, 0.3); //段々黒くなる
    glBegin(GL_QUADS);
    glVertex3d(-0.5 * scanAreaW, 0.0, 0.0);
    glVertex3d(-0.5 * scanAreaW, scanAreaH, 0.0);
    glVertex3d(0.5 * scanAreaW, scanAreaH, 0.0);
    glVertex3d(0.5 * scanAreaW, 0.0, 0.0);
    glEnd();

    // スキャン点を描画するための設定
    glEnable(GL_TEXTURE_2D);         // テクスチャマッピングを有効化
    glColor4d(1.0, 1.0, 1.0, 1.0);   // 描画色を白に設定
    glBindTexture(GL_TEXTURE_2D, 0); // 使用するテクスチャを選択

    // LiDARから取得した全てのデータ点についてループ
    for (int i = 0; i < distData.size(); i++)
    {
        // スキャン点情報を直交座標(x, y)に変換
        double len = distData[i] * 0.1; // 距離(mm)をcmに変換
        if (len < 5.0)
        { //距離が近い点は描画しない
            continue;
        }

        double rad = urg.index2rad(i); // i番目のスキャン点の角度(ラジアン)を取得
        double x = -len * sin(rad);    // i番目のスキャン点のx座標 (センサー座標系→OpenGL座標系)
        double y = len * cos(rad);     // i番目のスキャン点のy座標 (センサー座標系→OpenGL座標系)
        double z = 0;                  // 2Dなのでzは0

        // 距離が遠い点ほど大きく描画し、スキャン点間の隙間を埋める
        // 距離の平方根に比例させることで、変化を緩やかにしている
        double circleR = 40.0 * sqrt(len) * tan(scanAngleStep * M_PI / 180.0 / 2); // 描画する円の半径を計算

        // スキャン点をテクスチャを貼った四角形として描画
        glPushMatrix();                               // 現在の座標変換行列を保存
        glTranslated(x, y, z);                        // 計算した座標へ移動
        glRotated(rad * 180.0 / M_PI, 0.0, 0.0, 1.0); // 描画する四角形を回転
        glScaled(circleR * 1.5, circleR * 1.0, 1);    // 計算した半径に拡大・縮小
        glBegin(GL_QUADS);                            // 四角形の描画開始
        glTexCoord2d(0, 0);
        glVertex3d(-0.5, 0.5, 0.0);
        glTexCoord2d(0, 1);
        glVertex3d(-0.5, -0.5, 0.0);
        glTexCoord2d(1, 1);
        glVertex3d(0.5, -0.5, 0.0);
        glTexCoord2d(1, 0);
        glVertex3d(0.5, 0.5, 0.0);
        glEnd();       // 描画終了
        glPopMatrix(); // 保存した座標変換行列を復元
    }
    glDisable(GL_TEXTURE_2D); // テクスチャマッピングを無効化

    // ダブルバッファリング: バックバッファに描画した内容をフロントバッファ(画面)に反映
    glutSwapBuffers();

    //---- [2] OpenGLの描画結果をOpenCVで画像処理 ----
    // OpenGLのフレームバッファからピクセルデータを読み取り、scanImageに格納
    glReadPixels(0, 0, scanImage.cols, scanImage.rows, GL_BGR, GL_UNSIGNED_BYTE, scanImage.data);
    cv::flip(scanImage, scanImage, 0);                     // 画像が上下逆に読み込まれるため、垂直方向に反転
    cv::cvtColor(scanImage, binImage, cv::COLOR_BGR2GRAY); // 処理のためにグレースケール画像に変換
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

    //デバック用ウィンドウ表示

    cv::imshow("binImage", binImage);

    //---- [3] 輪郭検出と重心計算 ----
    std::vector<std::vector<cv::Point>> contours; // 検出された輪郭の情報を格納するベクター
    // 画像から輪郭を検出する (RETR_EXTERNAL: 最も外側の輪郭のみを検出)
    cv::findContours(binImage, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    int closestTargetIndex = -1;
    double minDistance = 99999999.0; //距離を比較するための初期値
    Vec_3D closestPointF;            //最も近い物体の座標を一時保存する変数


    //1, 全ての輪郭をチェックして『一番センサーに近い物体』を探す
    for (int i = 0; i < contours.size(); i++)
    {                                           //輪郭の数分ループ
        double area = contourArea(contours[i]); //輪郭の頂点リストから面積を計算

        if (area > objectSize)
        {
            // 【修正】重心の代わりに、物体を囲む最小の傾いた長方形（RotatedRect）を計算
            cv::RotatedRect rotatedBox = cv::minAreaRect(contours[i]);
            // 長方形の中心点を取得
            cv::Point2f p = rotatedBox.center;

            //現実世界の座標(cm)に変換
            Vec_3D pointF;
            gluUnProject(p.x, binImage.rows - p.y, 0, model, proj, view, &pointF.x, &pointF.y, &pointF.z);

            double distFromSensor = sqrt(pow(pointF.x, 2) + pow(pointF.y, 2));

            if (distFromSensor < minDistance)
            {
                minDistance = distFromSensor; // 最小距離を更新
                closestTargetIndex = i;       // 輪郭の番号を記録
                closestPointF = pointF;       // 座標を記録
            }
        }
    }
    std::vector<Vec_3D> handPoint; // 検出した物体の重心座標を格納するベクター

    // 【ステップ2】一番近い物体が見つかった場合のみ、描画と保存を行う
    if (closestTargetIndex != -1)
    {
        // 青色で一番近い物体の輪郭を描画
        cv::drawContours(scanImage, contours, closestTargetIndex, cv::Scalar(255, 0, 0), 2);

        // 【修正】描画用に、一番近い物体の最小外接矩形の中心を再度求める
        cv::RotatedRect rotatedBox = cv::minAreaRect(contours[closestTargetIndex]);
        cv::Point2f p = rotatedBox.center;

        // 赤色で中心位置を描画
        cv::circle(scanImage, p, 5, cv::Scalar(0, 0, 255), -1);

        // 外接する長方形自体も緑色で描画して確認できるようにする（デバッグ用）
        cv::Point2f vertices[4];
        rotatedBox.points(vertices);
        for (int j = 0; j < 4; j++)
        {
            cv::line(scanImage, vertices[j], vertices[(j + 1) % 4], cv::Scalar(0, 255, 0), 2);
        }

        if (isFirst)
        {
            //初回は最新の座標を基準として保存する
            tmpPoint = closestPointF;
            isFirst = false;
        }
        else
        {
            // 前回座標と今回座標の距離を計算
            double dx = closestPointF.x - tmpPoint.x;
            double dy = closestPointF.y - tmpPoint.y;
            double dist = sqrt(dx * dx + dy * dy);

            //微小なブレ（ノイズ）を無視するため、一定距離(moveThreshold)以上動いた時だけ更新する
            if (dist > moveThreshold)
            {
                //// 指数移動平均の計算：(過去の座標 * 重み) + (最新の座標 * (1 - 重み))
                tmpPoint.x = (tmpPoint.x * smoothFactor) + (closestPointF.x * (1.0 - smoothFactor));
                tmpPoint.y = (tmpPoint.y * smoothFactor) + (closestPointF.y * (1.0 - smoothFactor));
                tmpPoint.z = (tmpPoint.z * smoothFactor) + (closestPointF.z * (1.0 - smoothFactor));
            }
        }

        // ファイル出力用に座標を保存
        handPoint.push_back(tmpPoint);
    }
    else
    {
        // なにも見つからなかった場合は初回フラグをリセットし、
        isFirst = true;
    }

    printf("%ld 個の物体(一番手前のみ)を認識しました\n", handPoint.size()); // (デバッグ用)
    printf("===========\n");

    //---- [4] 結果をファイルに出力 ----
    fp = fopen(footPointFile, "w");         // 出力ファイルを開く (w: 上書きモード)
    fprintf(fp, "%ld\n", handPoint.size()); // 最初に検出した物体の数を出力
    for (int i = 0; i < handPoint.size(); i++)
    { // 各物体の座標を出力
        fprintf(fp, "%f,%f\n", handPoint[i].x, handPoint[i].y);
    }
    fclose(fp); // ファイルを閉じる

    // OpenCVのウィンドウで、処理結果の画像を表示


    cv::imshow("Scan", scanImage);
}

//============================================================
// リシェイプコールバック関数 - ウィンドウサイズ変更時の処理
//============================================================
void reshape(int w, int h)
{
    // ウィンドウサイズに合わせて、描画領域(ビューポート)を再設定
    glViewport(0, 0, w * rDisp, h * rDisp);

    // 投影変換行列の設定
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity(); // 行列を初期化
    // 2D正投影を設定 (左, 右, 下, 上, 前方クリップ, 後方クリップ)
    glOrtho(-scanAreaW * 0.5, scanAreaW * 0.5, 0.0, scanAreaH, -100, 100);
    glMatrixMode(GL_MODELVIEW); // 行列モードをモデルビューに戻す

    // gluUnProjectで座標逆変換を行うために、現在の各種行列とビューポート設定を取得・保存しておく
    glGetDoublev(GL_MODELVIEW_MATRIX, model); // モデルビュー変換行列を"model[]"に格納
    glGetDoublev(GL_PROJECTION_MATRIX, proj); // 投影変換行列を"proj[]"に格納
    glGetIntegerv(GL_VIEWPORT, view);         // ビューポート設定を"view[]"に格納
    for (int i = 0; i < 4; i++)
    { // Retinaディスプレイ用の補正
        view[i] /= rDisp;
    }
}

//============================================================
// タイマーコールバック関数 - 定期処理
//============================================================
void timer(int value)
{
    // 次のタイマーイベントを予約する (これにより、この関数が定期的に呼ばれ続ける)
    glutTimerFunc(1000 / fr, timer, 0);

    // LiDARから最新の距離データを取得する
    long time_stamp = 0;
    // 2回呼ぶことで、バッファに残っている古いデータを読み飛ばし、より最新のデータを取得する
    for (int i = 0; i < 2; i++)
    {
        urg.get_distance(distData, &time_stamp);
    }

    // ディスプレイイベントを強制的に発生させる。これによりdisplay()関数が呼ばれ、画面が更新される
    glutPostRedisplay();
}

//============================================================
// キーボードコールバック関数 - キー入力処理
//============================================================
void keyboard(unsigned char key, int x, int y)
{
    switch (key)
    {
    case 27:     // ESCキー(ASCIIコード 27)が押された場合
        exit(0); // プログラムを終了する
        break;

    default:
        break;
    }
}
