// LiDAR
/*
 * g++ -O3 main_cv.cpp -std=c++11 `pkg-config --cflags --libs opencv4` -framework OpenGL -framework GLUT Connection_information.cpp liburg_cpp.a -Wno-deprecated
 */
#include <iostream>                 // 標準入出力
#include <time.h>                   // 時間関連
#include <math.h>                   // 数学関数 (sin, cos, sqrt, tan, M_PI)
#include <opencv2/opencv.hpp>       // 画像処理ライブラリ OpenCV
#include <GLUT/glut.h>              // グラフィックスライブラリ GLUT (OpenGLの補助)
#include "Urg_driver.h"             // URGセンサー用ドライバ
#include "Connection_information.h" // URGセンサーの接続情報用

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
int scanAngle = 180;         // スキャンする角度の範囲 (正面を0度として±90度)
double scanAngleStep = 0.35; // URGセンサーの1ステップあたりの角度 (UBG-04LX-F01)
double objectSize = 300; // 物体の最小面積 (ピクセル)
// double scanAngleStep = 0.125;  // (UST-20LX-H01の場合)
double scanAreaW, scanAreaH; // 描画・処理する領域のサイズ(cm) (ファイルから読み込む)
double scanReso;             // 1画素あたりのサイズ(cm) (ファイルから読み込む)
cv::Mat scanImage, binImage; // OpenCVで画像処理を行うための画像データ格納用
cv::Mat element;             // モルフォロジー処理(膨張・収縮)で使用する構造要素


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
    if (!urg.open(information.device_or_ip_name(), information.baudrate_or_port_number(), information.connection_type()))
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
    fscanf(fp, "%lf,%lf,%lf", &scanAreaW, &scanAreaH, &scanReso);
    fclose(fp);
    // 読み込んだ情報に基づき、OpenCVで使う画像領域をメモリ上に確保
    scanImage = cv::Mat(cv::Size(scanAreaW / scanReso, scanAreaH / scanReso), CV_8UC3); // カラー画像用
    binImage = cv::Mat(cv::Size(scanAreaW / scanReso, scanAreaH / scanReso), CV_8UC1);  // 白黒(2値)画像用


    //---- GLUTウィンドウの生成 ----
    glutInitWindowSize(scanAreaW / scanReso, scanAreaH / scanReso); // ウィンドウサイズ指定
    glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);      // ディスプレイモード設定(RGBA色, Zバッファ, ダブルバッファ)
    glutInitWindowPosition(0, 0);                                   // ウィンドウの表示位置
    glutCreateWindow("CG");                                         // "CG"というタイトルでウィンドウを生成

    //---- コールバック関数の登録 ----
    glutDisplayFunc(display);           // 描画にはdisplay関数を使用
    glutReshapeFunc(reshape);           // ウィンドウサイズ変更時にはreshape関数を使用
    glutKeyboardFunc(keyboard);         // キーボード入力時にはkeyboard関数を使用
    glutTimerFunc(1000 / fr, timer, 0); // 1000/frミリ秒後からtimer関数を呼び出し開始

    //---- OpenGLの描画設定 ----
    glClearColor(0.0, 0.0, 0.0, 1.0);                  // 背景色を黒に設定
    glEnable(GL_DEPTH_TEST);                           // 深度テストを有効化 (3D描画で前後関係を正しく描画するため)
    glEnable(GL_BLEND);                                // アルファブレンディング(透過処理)を有効化
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // ブレンド方法の設定
    glEnable(GL_ALPHA_TEST);                           // アルファテストを有効化
    glAlphaFunc(GL_GREATER, 0.01);                     // 透明度が0.01より大きいピクセルのみ描画

    //---- テクスチャの読み込み ----
    // スキャン点を描画するための画像(mark.png)を読み込む
    cv::Mat textureImage = cv::imread("mark.png", cv::IMREAD_UNCHANGED);
    glBindTexture(GL_TEXTURE_2D, 0); // テクスチャID 0番を有効化
    // テクスチャの拡大・縮小時のフィルタリング方法を設定(ニアレストネイバー)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    // 読み込んだ画像をテクスチャとしてVRAMに転送
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, textureImage.cols, textureImage.rows, 0, GL_BGRA, GL_UNSIGNED_BYTE, textureImage.data);

    // モルフォロジー処理で使用する構造要素(5x5の楕円形)を作成
    element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));

}

//============================================================
// ディスプレイコールバック関数 - 描画処理の本体
//============================================================
void display()
{
    //---- [1] LiDARデータをOpenGLで描画 ----


    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // 画面(カラーバッファとデプスバッファ)をクリア
    glLoadIdentity();                                   // 座標変換行列を初期化

    // スキャン点を描画するための設定
    glEnable(GL_TEXTURE_2D);         // テクスチャマッピングを有効化
    glColor4d(1.0, 1.0, 1.0, 1.0);   // 描画色を白に設定
    glBindTexture(GL_TEXTURE_2D, 0); // 使用するテクスチャを選択

    // LiDARから取得した全てのデータ点についてループ
    for (int i = 0; i < distData.size(); i++)
    {
        // スキャン点情報を直交座標(x, y)に変換
        double len = distData[i] * 0.1; // 距離(mm)をcmに変換
        if(len < 5.0){//距離が近い点は描画しない
            continue;
        }

        double rad = urg.index2rad(i);  // i番目のスキャン点の角度(ラジアン)を取得
        double x = -len * sin(rad);     // i番目のスキャン点のx座標 (センサー座標系→OpenGL座標系)
        // if(x < -2.0){
        //     continue;
        // }
        double y = len * cos(rad);      // i番目のスキャン点のy座標 (センサー座標系→OpenGL座標系)
        double z = 0;                   // 2Dなのでzは0

        // 距離が遠い点ほど大きく描画し、スキャン点間の隙間を埋める
        // 距離の平方根に比例させることで、変化を緩やかにしている
        double circleR = 80.0 * sqrt(len) * tan(scanAngleStep * M_PI / 180.0 / 2); // 描画する円の半径を計算

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
    cv::flip(scanImage, scanImage, 0);// 画像が上下逆に読み込まれるため、垂直方向に反転
    cv::cvtColor(scanImage, binImage, cv::COLOR_BGR2GRAY); // 処理のためにグレースケール画像に変換

    // // モルフォロジー処理でノイズ除去と領域の結合を行う
    cv::dilate(binImage, binImage, element, cv::Point(-1, -1), 2); // 2回膨張させて、スキャン点間の隙間を埋める
    cv::erode(binImage, binImage, element, cv::Point(-1, -1), 2);  // 2回収縮させて、膨張した領域を元に戻しつつ、ノイズを除去
    cv::dilate(binImage, binImage, element, cv::Point(-1,-1), 5); // 必要なら再度膨張
    cv::erode(binImage, binImage, element, cv::Point(-1, -1), 2);  // 2回収縮させて、膨張した領域を元に戻しつつ、ノイズを除去

    //---- [3] 輪郭検出と重心計算 ----
    std::vector<std::vector<cv::Point>> contours; // 検出された輪郭の情報を格納するベクター
    // 画像から輪郭を検出する (RETR_EXTERNAL: 最も外側の輪郭のみを検出)
    cv::findContours(binImage, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    std::vector<Vec_3D> footPoints; // 検出した物体の重心座標(実世界座標)を格納するベクター
    for (int i = 0; i < contours.size(); i++)
    {                                           // 検出された全ての輪郭についてループ
        double area = contourArea(contours[i]); // 輪郭の面積を計算
        if (area > objectSize)
        { // 面積が50ピクセルより大きいものだけを物体として認識 (ノイズ除去)

            // 輪郭のモーメントを計算して、重心を求める
            cv::Moments m = cv::moments(contours[i]);
            cv::Point p = cv::Point(m.m10 / m.m00, m.m01 / m.m00); // 重心のピクセル座標(x, y)
            // 確認のため、処理結果画像(scanImage)の重心位置に赤い円を描画
            cv::circle(scanImage, cv::Point(p.x, p.y), 5, cv::Scalar(0, 0, 255), -1);

            // 画面上の2Dピクセル座標を、OpenGLの世界座標(cm)に逆変換する
            Vec_3D pointF;
            gluUnProject(p.x, binImage.rows - p.y, 0, model, proj, view, &pointF.x, &pointF.y, &pointF.z);
            footPoints.push_back(pointF); // 変換後の実世界座標を格納
             printf("%f, %f\n", pointF.x, pointF.y); // (デバッグ用)座標をコンソールに表示

            if (footPoints.size() >= 1) {
                break;
            }
// break; // 複数物体を検出するためにbreakを削除
        }
    }
     printf("%ld\n", footPoints.size()); // (デバッグ用)検出した物体数を表示
     printf("===========\n");

    //---- [4] 結果をファイルに出力 ----
    fp = fopen(footPointFile, "w");          // 出力ファイルを開く (w: 上書きモード)
    fprintf(fp, "%ld\n", footPoints.size()); // 最初に検出した物体の数を出力
    for (int i = 0; i < footPoints.size(); i++)
    { // 各物体の座標を出力
        fprintf(fp, "%f,%f\n", footPoints[i].x, footPoints[i].y);
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
