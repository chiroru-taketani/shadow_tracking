#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <XnOpenNI.h>  //Kinect
#include <XnCodecIDs.h>  //Kinect
#include <XnCppWrapper.h>  //Kinect
#include <XnPropNames.h>  //Kinect
//#include <map>  //Kinect
#include <GLUT/glut.h>  //OpenGL
#include <opencv2/opencv.hpp>  //OpenCV関連ヘッダ

using namespace xn;

//定数の宣言
#define GL_WIN_SIZE_X 640
#define GL_WIN_SIZE_Y 480
#define GL_WIN_TOTAL 307200
#define MAX_DEPTH 10000
#define XN_CALIBRATION_FILE_NAME "UserCalibration.bin"
#define MAX_USER 10

#define SAMPLE_XML_PATH "SamplesConfig.xml"

#define CHECK_RC(nRetVal, what)                        \
if (nRetVal != XN_STATUS_OK)                            \
{                                                                \
printf("%s failed: %s\n", what, xnGetStatusString(nRetVal));\
return nRetVal;                                            \
}

//三次元ベクトル構造体: Vec_3D
typedef struct _Vec_3D
{
    double x, y, z, d;
} Vec_3D;

//関数名宣言
//Kinect関係
//std::map<XnUInt32, std::pair<XnCalibrationStatus, XnPoseDetectionStatus> > m_Errors;
int initKinect();
//void XN_CALLBACK_TYPE MyCalibrationInProgress(SkeletonCapability& capability, XnUserID id, XnCalibrationStatus calibrationError, void* pCookie);
//void XN_CALLBACK_TYPE MyPoseInProgress(PoseDetectionCapability& capability, const XnChar* strPose, XnUserID id, XnPoseDetectionStatus poseError, void* pCookie);
void CleanupExit();
void XN_CALLBACK_TYPE User_NewUser(UserGenerator& generator, XnUserID nId, void* pCookie);
void XN_CALLBACK_TYPE User_LostUser(UserGenerator& generator, XnUserID nId, void* pCookie);
void XN_CALLBACK_TYPE UserPose_PoseDetected(PoseDetectionCapability& capability, const XnChar* strPose, XnUserID nId, void* pCookie);
void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(SkeletonCapability& capability, XnUserID nId, void* pCookie);
void XN_CALLBACK_TYPE UserCalibration_CalibrationComplete(SkeletonCapability& capability, XnUserID nId, XnCalibrationStatus eStatus, void* pCookie);
int getJoint();
void getDepthImage();

void display();
void initGL();
void resize(int w, int h);
void mouse(int button, int state, int x, int y);
void motion(int x, int y);
void timer(int value);
void keyboard(unsigned char key, int x, int y);
Vec_3D vectorNormalize(Vec_3D v0);
void initCV();

//グローバル変数
//Kinect関連
Context g_Context;
ScriptNode g_scriptNode;
DepthGenerator g_DepthGenerator;
UserGenerator g_UserGenerator;
ImageGenerator g_ImageGenerator;
Player g_Player;
XnBool g_bNeedPose = FALSE;
XnChar g_strPose[20] = "";
XnBool g_bDrawBackground = TRUE;
XnBool g_bDrawPixels = TRUE;
XnBool g_bDrawSkeleton = TRUE;
XnBool g_bPrintID = TRUE;
XnBool g_bPrintState = TRUE;
XnBool g_bPause = false;
XnBool g_bRecord = false;
XnBool g_bQuit = false;
XnRGB24Pixel* g_pTexMap = NULL;
unsigned int g_nTexMapX = 0;
unsigned int g_nTexMapY = 0;
SceneMetaData sceneMD;
DepthMetaData depthMD;
ImageMetaData imageMD;

int winW, winH;  //ウィンドウサイズ
Vec_3D e, tg;  //視点，目標点
double eDegY, eDegX, eDist;  //視点の水平角，垂直角，距離
int mX, mY, mState, mButton;  //マウスクリック位置格納用
double fLate = 60;  //フレームレート
int jointNum = 15;  //Kinectで取得する関節の数
XnPoint3D t1[GL_WIN_TOTAL], pointPos[GL_WIN_TOTAL];  //深度情報，座標
XnRGB24Pixel pointCol[GL_WIN_TOTAL];  //色情報
XnPoint3D jointPos[MAX_USER][24];  //関節座標
XnLabel pointLabel[GL_WIN_TOTAL];  //人ラベル情報

cv::Mat depthImage, depthDispImage, binImage;

//メイン関数
int main(int argc, char **argv)
{
    //Kinect関連初期化
    if (initKinect()>0)
        return 1;
    
    //初期化処理
    glutInit(&argc, argv);
    initGL();
    initCV();
    
    //イベント待ち無限ループ
    glutMainLoop();
    return 0;
}

//------------------------------------------------------- 関数 --------------------------------------------------------
void initCV()
{
    //画像
    depthImage = cv::Mat(cv::Size(GL_WIN_SIZE_X, GL_WIN_SIZE_Y), CV_32FC1);  //深度値生データ
    depthDispImage = cv::Mat(cv::Size(GL_WIN_SIZE_X, GL_WIN_SIZE_Y), CV_8UC1);  //深度値表示用
    binImage = cv::Mat(cv::Size(GL_WIN_SIZE_X, GL_WIN_SIZE_Y), CV_8UC1);  //領域用
}

void initGL()
{
    //視点
    eDegY = 180.0; eDegX = 0.0; eDist = 2000.0;
    //目標点
    tg.x = 0.0; tg.y = 0.0; tg.z = 2000;
    
    //ウィンドウ
    winW = 1024; winH = 768;
    glutInitWindowSize(winW, winH);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
    glutCreateWindow("KINECT");
    
    //コールバック関数の指定
    glutDisplayFunc(display);
    glutReshapeFunc(resize);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutTimerFunc(1000.0/fLate, timer, 0);
    glutKeyboardFunc(keyboard);
    
    //その他設定
    glClearColor(0.0, 0.0, 0.0, 1.0);
    glEnable(GL_DEPTH_TEST);  //Zバッファの有効化
    glEnable(GL_BLEND);
    glEnable(GL_NORMALIZE);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glAlphaFunc(GL_GREATER, 0.02);
    
    //光源
    //glEnable(GL_LIGHTING);  //陰影付けの有効化
    
    //光源の各種パラメータ設定
    GLfloat col[4];  //色指定用配列
    glEnable(GL_LIGHT0);  //光源0の有効化
    col[0] = 0.8; col[1] = 0.8; col[2] = 0.8; col[3] = 1.0;
    glLightfv(GL_LIGHT0, GL_DIFFUSE, col);  //拡散反射光
    col[0] = 0.2; col[1] = 0.2; col[2] = 0.2; col[3] = 1.0;
    glLightfv(GL_LIGHT0, GL_AMBIENT, col);  //環境光
    col[0] = 1.0; col[1] = 1.0; col[2] = 1.0; col[3] = 1.0;
    glLightfv(GL_LIGHT0, GL_SPECULAR, col);  //鏡面反射光
}

//ディスプレイコールバック関数
void display()
{
    //描画用バッファおよびZバッファの消去
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    //投影変換行列の設定
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();  //行列初期化
    gluPerspective(43.0, (double)winW/(double)winH, 50.0, 10000);
    
    //行列初期化(モデルビュー変換行列)
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    //視点視線の設定
    double eRad_y = M_PI*eDegY/180.0;
    double eRad_x = M_PI*eDegX/180.0;  //角度からラジアンに変換
    e.x = eDist*cos(eRad_x)*sin(eRad_y)+tg.x;
    e.y = eDist*sin(eRad_x)+tg.y;
    e.z = eDist*cos(eRad_x)*cos(eRad_y)+tg.z;
    gluLookAt(e.x, e.y, e.z, tg.x, tg.y, tg.z, 0.0, 1.0, 0.0);
    
    GLfloat pos[4], col[4], val[1];
    
    //光源0の位置指定
    pos[0] = 1300.0; pos[1] = 1800.0; pos[2] = -1200.0; pos[3] = 1.0;
    glLightfv(GL_LIGHT0, GL_POSITION, pos);
    
    //------------------------------Kinectデータ取得，処理，表示------------------------------
    //データ取得準備
    XnStatus rc = XN_STATUS_OK;
    
    //フレーム読み込み
    rc = g_Context.WaitAnyUpdateAll();
    if (rc != XN_STATUS_OK) {
        printf("Read failed: %s\n", xnGetStatusString(rc));
        return;
    }
    
    //データ処理
    g_DepthGenerator.GetMetaData(depthMD);
    g_UserGenerator.GetUserPixels(0, sceneMD);
    g_ImageGenerator.GetMetaData(imageMD);
    
    //関節点"jointPos[]"取得
    int rtn = getJoint();
    
    //スキャン点"pointPos[GL_WIN_TOTAL]"と色"pointCol[]"取得
    getDepthImage();
    
//    double depthMin = 10000;
//    //pointPos[]のデータからdepthImage生成
//    for (int i=0; i<GL_WIN_TOTAL; i++) {
//        int N = i/GL_WIN_SIZE_X;
//        int M = GL_WIN_SIZE_X-(i-N*GL_WIN_SIZE_X)-1;
//
//        double depth = pointPos[i].Z;
//        if (depth<100 || depth>10000) depth = 10000;
//
//        depthImage.at<float>(N, M) = depth;
//        depthDispImage.at<unsigned char>(N, M) = MAX(0, (5000.0-depth)/5000.0*255);
//
//        if (depth<depthMin) depthMin = depth;
//    }
//
//    cv::threshold(depthImage, depthImage, depthMin+300, 255, cv::THRESH_BINARY_INV);
//    depthImage.convertTo(binImage, CV_8UC1);
//
//    cv::erode(binImage, binImage, cv::Mat(), cv::Point(-1,-1), 2);
//    cv::dilate(binImage, binImage, cv::Mat(), cv::Point(-1,-1), 4);
//    cv::erode(binImage, binImage, cv::Mat(), cv::Point(-1,-1), 2);
//
//    std::vector< std::vector<cv::Point> > contours;  //輪郭格納用ベクトル
//    cv::findContours(binImage, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
//
//    std::vector< std::vector<cv::Point2f> > corners(0,std::vector<cv::Point2f>(5));  //輪郭格納用ベクトル
//
//    for (int i=0; i<contours.size(); i++) {
//        double area = cv::contourArea(contours[i]);
//        if (area<2500) continue;
//
//        cv::RotatedRect rRect = cv::minAreaRect(contours[i]);
//        cv::Point2f corner[4], center;
//        rRect.points(corner);
//        std::vector<cv::Point2f> cornersTemp;
//
//        for (int j=0; j<4; j++) {
//            center += corner[j];
//            cv::line(depthDispImage, corner[j], corner[(j+1)%4], 255, 2, 8);
//        }
//        center /= 4.0;
//        cv::circle(depthDispImage, center, 2, 255, -1, 8);
//
//        for (int j=0; j<4; j++) {
//            cornersTemp.push_back((corner[j]-center)*0.5+center);
//            cv::circle(depthDispImage, cornersTemp[j], 2, 255, -1, 8);
//        }
//        cornersTemp.push_back(center);
//
//        corners.push_back(cornersTemp);
//    }

    //Kinect取得点表示
    //関節点"jointPos[]"表示
//    glColor3d(1.0, 1.0, 0.0);
//    for (int i=0; i<jointNum; i++) {
//        glPushMatrix();
//        glTranslated(jointPos[i].X, jointPos[i].Y, jointPos[i].Z);
//        glutSolidSphere(10, 10, 5);
//        glPopMatrix();
//    }
    
    //関節点
    //0:頭，1:首，2:腹
    //3:左肩，4:左肘，5:左手，6:右肩，7:右肘，8:右手
    //9:左腰，10:左膝，11:左足，12:右腰，13:右膝，14:右足
//    glEnable(GL_LIGHTING);
//    col[0] = 1.0; col[1] = 1.0; col[2] = 0.0; col[3] = 1.0;  //拡散反射係数
//    spe[0] = 1.0; spe[1] = 1.0; spe[2] = 1.0; spe[3] = 1.0;  //鏡面反射係数
//    shi[0] = 50;
//    glMaterialfv(GL_FRONT, GL_DIFFUSE, col);  //拡散反射
//    glMaterialfv(GL_FRONT, GL_AMBIENT, col);  //環境光
//    glMaterialfv(GL_FRONT, GL_SPECULAR, spe);  //鏡面反射
//    glMaterialfv(GL_FRONT, GL_SHININESS, shi);  //ハイライト
    
    glColor4d(0.0, 1.0, 0.0, 1.0);
    for (int i=0; i<rtn; i++) {
        for (int j=0; j<jointNum; j++) {
            glPushMatrix();
            glTranslated(jointPos[i][j].X, jointPos[i][j].Y, jointPos[i][j].Z);  //人物iのj番目の関節点の座標
            glutSolidSphere(25.0, 18, 9);  //関節点に球を配置
            glPopMatrix();
        }
    }
    //glDisable(GL_LIGHTING);
    //骨格
    glColor4d(1.0, 1.0, 0.0, 1.0);
    glLineWidth(2.0);
    for (int i=0; i<rtn; i++) {
        glBegin(GL_LINES);
        //頭-首
        glVertex3d(jointPos[i][0].X, jointPos[i][0].Y, jointPos[i][0].Z);
        glVertex3d(jointPos[i][1].X, jointPos[i][1].Y, jointPos[i][1].Z);
        //首-腹
        glVertex3d(jointPos[i][1].X, jointPos[i][1].Y, jointPos[i][1].Z);
        glVertex3d(jointPos[i][2].X, jointPos[i][2].Y, jointPos[i][2].Z);
        //首-左肩
        glVertex3d(jointPos[i][1].X, jointPos[i][1].Y, jointPos[i][1].Z);
        glVertex3d(jointPos[i][3].X, jointPos[i][3].Y, jointPos[i][3].Z);
        //左肩-左肘
        glVertex3d(jointPos[i][3].X, jointPos[i][3].Y, jointPos[i][3].Z);
        glVertex3d(jointPos[i][4].X, jointPos[i][4].Y, jointPos[i][4].Z);
        //左肘-左手
        glVertex3d(jointPos[i][4].X, jointPos[i][4].Y, jointPos[i][4].Z);
        glVertex3d(jointPos[i][5].X, jointPos[i][5].Y, jointPos[i][5].Z);
        //首-右肩
        glVertex3d(jointPos[i][1].X, jointPos[i][1].Y, jointPos[i][1].Z);
        glVertex3d(jointPos[i][6].X, jointPos[i][6].Y, jointPos[i][6].Z);
        //右肩-右肘
        glVertex3d(jointPos[i][6].X, jointPos[i][6].Y, jointPos[i][6].Z);
        glVertex3d(jointPos[i][7].X, jointPos[i][7].Y, jointPos[i][7].Z);
        //右肘-右手
        glVertex3d(jointPos[i][7].X, jointPos[i][7].Y, jointPos[i][7].Z);
        glVertex3d(jointPos[i][8].X, jointPos[i][8].Y, jointPos[i][8].Z);
        //腹-左腰
        glVertex3d(jointPos[i][2].X, jointPos[i][2].Y, jointPos[i][2].Z);
        glVertex3d(jointPos[i][9].X, jointPos[i][9].Y, jointPos[i][9].Z);
        //左腰-左膝
        glVertex3d(jointPos[i][9].X, jointPos[i][9].Y, jointPos[i][9].Z);
        glVertex3d(jointPos[i][10].X, jointPos[i][10].Y, jointPos[i][10].Z);
        //左膝-左足
        glVertex3d(jointPos[i][10].X, jointPos[i][10].Y, jointPos[i][10].Z);
        glVertex3d(jointPos[i][11].X, jointPos[i][11].Y, jointPos[i][11].Z);
        //腹-右腰
        glVertex3d(jointPos[i][2].X, jointPos[i][2].Y, jointPos[i][2].Z);
        glVertex3d(jointPos[i][12].X, jointPos[i][12].Y, jointPos[i][12].Z);
        //右腰-右膝
        glVertex3d(jointPos[i][12].X, jointPos[i][12].Y, jointPos[i][12].Z);
        glVertex3d(jointPos[i][13].X, jointPos[i][13].Y, jointPos[i][13].Z);
        //右膝-右足
        glVertex3d(jointPos[i][13].X, jointPos[i][13].Y, jointPos[i][13].Z);
        glVertex3d(jointPos[i][14].X, jointPos[i][14].Y, jointPos[i][14].Z);
        //腹-左肩
        glVertex3d(jointPos[i][2].X, jointPos[i][2].Y, jointPos[i][2].Z);
        glVertex3d(jointPos[i][3].X, jointPos[i][3].Y, jointPos[i][3].Z);
        //腹-右肩
        glVertex3d(jointPos[i][2].X, jointPos[i][2].Y, jointPos[i][2].Z);
        glVertex3d(jointPos[i][6].X, jointPos[i][6].Y, jointPos[i][6].Z);
        //左腰-右腰
        glVertex3d(jointPos[i][9].X, jointPos[i][9].Y, jointPos[i][9].Z);
        glVertex3d(jointPos[i][12].X, jointPos[i][12].Y, jointPos[i][12].Z);
        glEnd();
    }

    //スキャン点"pointPos[]"点描
    glPointSize(2.0);
    glBegin(GL_POINTS);
    for (int i=0; i<GL_WIN_TOTAL; i++) {
        double alpha = 1.0;
        if (pointLabel[i]!=0) alpha = 0.5;
        
        glColor4d(pointCol[i].nRed/255.0, pointCol[i].nGreen/255.0, pointCol[i].nBlue/255.0, alpha);
        glVertex3f(pointPos[i].X, pointPos[i].Y, pointPos[i].Z);  //空間に点を配置
    }
    glEnd();
    

//    for (int i=0; i<corners.size(); i++) {
//        glColor4d(1.0, 1.0, 0.0, 0.5);
//        int m, n, k;
//        glBegin(GL_QUADS);
//        for (int j=0; j<4; j++) {
//            m = corners[i][j].x; n = corners[i][j].y;
//            //k = n*GL_WIN_SIZE_X+GL_WIN_SIZE_X-m-1;
//            k = (int)corners[i][j].y*GL_WIN_SIZE_X+GL_WIN_SIZE_X-(int)corners[i][j].x-1;
//            glVertex3d(pointPos[k].X, pointPos[k].Y, pointPos[k].Z);
//        }
//        glEnd();
//
//        m = corners[i][4].x; n = corners[i][4].y;
//        k = n*GL_WIN_SIZE_X+GL_WIN_SIZE_X-m-1;
//        glColor4d(1.0, 0.0, 0.0, 0.5);
//        glPushMatrix();
//        glTranslated(pointPos[k].X, pointPos[k].Y, pointPos[k].Z);
//        glutSolidSphere(10, 36, 18);
//        glPopMatrix();
//    }
//
//    cv::imshow("DEPTH", depthDispImage);
//    cv::imshow("BIN", binImage);
    
    //描画実行
    glutSwapBuffers();
}

//リサイズコールバック理関数
void resize(int w, int h)
{
    //ビューポート設定
    glViewport(0, 0, w, h);
    
    //投影変換行列の設定
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();  //行列初期化
    gluPerspective(43.0, (double)w/(double)h, 100.0, 10000);
    
    //モデルビュー変換行列の設定準備
    glMatrixMode(GL_MODELVIEW);
    
    winW = w; winH = h;
}

//マウスクリックコールバック関数
void mouse(int button, int state, int x, int y)
{
    GLfloat win_x, win_y, win_z;  //ウィンドウ座標
    GLdouble obj_x, obj_y, obj_z;  //ワールド座標
    GLdouble model[16], proj[16];  //変換用行列
    GLint view[4];  //変換用行列
    
    if (state==GLUT_DOWN) {
        //クリックしたマウス座標を(mX, mY)に格納
        mX = x; mY = y;
        
        //ボタン情報
        mButton = button; mState = state;
        
        //左クリックの場合，クリック点のワールド座標取得
        if (button==GLUT_LEFT_BUTTON) {
            //マウス座標をウィンドウ座標に変換
            win_x = x; win_y = winH-y;
            glReadPixels(win_x, win_y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &win_z);  //デプスバッファ取り出し
            //モデルビュー変換行列・透視変換行列・ビューポート変換行列を取り出す
            glGetDoublev(GL_MODELVIEW_MATRIX, model);
            glGetDoublev(GL_PROJECTION_MATRIX, proj);
            glGetIntegerv(GL_VIEWPORT, view);
            //ウィンドウ座標をワールド座標に変換
            gluUnProject(win_x, win_y, win_z, model, proj, view, &obj_x, &obj_y, &obj_z);
            
            printf("obj_x = %f, obj_y = %f, obj_z = %f\n", obj_x, obj_y, obj_z);
            
        }
    }
}

//マウスドラッグコールバック関数
void motion(int x, int y)
{
    if (mButton==GLUT_RIGHT_BUTTON) {
        //マウスのx方向の移動(mX-x)：水平角の変化
        eDegY = eDegY+(mX-x)*0.5;
        if (eDegY>360) eDegY-=360;
        if (eDegY<-0) eDegY+=360;
        
        //マウスのy方向の移動(y-mY)：垂直角の変化
        eDegX = eDegX+(y-mY)*0.5;
        if (eDegX>80) eDegX=80;
        if (eDegX<-80) eDegX=-80;
    }
    
    //現在のマウス座標を(mX, mY)に格納
    mX = x; mY = y;
}

//引数のベクトルを単位ベクトル化して戻す
Vec_3D vectorNormalize(Vec_3D v0)
{
    double len;  //ベクトル長
    Vec_3D v;  //戻り値用ベクトル
    
    //ベクトル長を計算
    len = sqrt(v0.x*v0.x+v0.y*v0.y+v0.z*v0.z);
    //ベクトル各成分をベクトル長で割って正規化
    if (len>0) {
        v.x = v0.x/len;
        v.y = v0.y/len;
        v.z = v0.z/len;
    }
    
    return v;  //正規化したベクトルを返す
}

//タイマーコールバック関数
void timer(int value)
{
    glutPostRedisplay();  //ディスプレイイベント強制発生
    glutTimerFunc(1000.0/fLate, timer, 0);
}

//キーボードコールバック関数
void keyboard(unsigned char key, int x, int y)
{
    switch (key) {
        case 27:
            CleanupExit();
            
        case 'f':
            glutFullScreen();
            break;
            
        default:
            break;
    }
}

//------------------------------------------------------- Kinect関係 --------------------------------------------------------
//Kinect初期化
int initKinect()
{
    XnStatus nRetVal = XN_STATUS_OK;
    
    EnumerationErrors errors;
    nRetVal = g_Context.InitFromXmlFile(SAMPLE_XML_PATH, g_scriptNode, &errors);
    if (nRetVal == XN_STATUS_NO_NODE_PRESENT)
    {
        XnChar strError[1024];
        errors.ToString(strError, 1024);
        printf("%s\n", strError);
        return (nRetVal);
    }
    else if (nRetVal != XN_STATUS_OK)
    {
        printf("Open failed: %s\n", xnGetStatusString(nRetVal));
        return (nRetVal);
    }
    
    nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
    if (nRetVal != XN_STATUS_OK)
    {
        printf("No depth generator found. Using a default one...");
        MockDepthGenerator mockDepth;
        nRetVal = mockDepth.Create(g_Context);
        CHECK_RC(nRetVal, "Create mock depth");
        
        // set some defaults
        XnMapOutputMode defaultMode;
        defaultMode.nXRes = 320;
        defaultMode.nYRes = 240;
        defaultMode.nFPS = 30;
        nRetVal = mockDepth.SetMapOutputMode(defaultMode);
        CHECK_RC(nRetVal, "set default mode");
        
        // set FOV
        XnFieldOfView fov;
        fov.fHFOV = 1.0225999419141749;
        fov.fVFOV = 0.79661567681716894;
        nRetVal = mockDepth.SetGeneralProperty(XN_PROP_FIELD_OF_VIEW, sizeof(fov), &fov);
        CHECK_RC(nRetVal, "set FOV");
        
        XnUInt32 nDataSize = defaultMode.nXRes * defaultMode.nYRes * sizeof(XnDepthPixel);
        XnDepthPixel* pData = (XnDepthPixel*)xnOSCallocAligned(nDataSize, 1, XN_DEFAULT_MEM_ALIGN);
        
        nRetVal = mockDepth.SetData(1, 0, nDataSize, pData);
        CHECK_RC(nRetVal, "set empty depth map");
        
        g_DepthGenerator = mockDepth;
    }
    
    nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
    if (nRetVal != XN_STATUS_OK)
    {
        nRetVal = g_UserGenerator.Create(g_Context);
        CHECK_RC(nRetVal, "Find user generator");
    }
    
    nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_IMAGE, g_ImageGenerator);
    if (nRetVal != XN_STATUS_OK)
    {
        printf("No image node exists! Check your XML.");
        return 1;
    }
    
    XnCallbackHandle hUserCallbacks, hCalibrationStart, hCalibrationComplete, hPoseDetected, hCalibrationInProgress, hPoseInProgress;
    if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON))
    {
        printf("Supplied user generator doesn't support skeleton\n");
        return 1;
    }
    nRetVal = g_UserGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);
    CHECK_RC(nRetVal, "Register to user callbacks");
    nRetVal = g_UserGenerator.GetSkeletonCap().RegisterToCalibrationStart(UserCalibration_CalibrationStart, NULL, hCalibrationStart);
    CHECK_RC(nRetVal, "Register to calibration start");
    nRetVal = g_UserGenerator.GetSkeletonCap().RegisterToCalibrationComplete(UserCalibration_CalibrationComplete, NULL, hCalibrationComplete);
    CHECK_RC(nRetVal, "Register to calibration complete");
    
    if (g_UserGenerator.GetSkeletonCap().NeedPoseForCalibration())
    {
        g_bNeedPose = TRUE;
        if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION))
        {
            printf("Pose required, but not supported\n");
            return 1;
        }
        nRetVal = g_UserGenerator.GetPoseDetectionCap().RegisterToPoseDetected(UserPose_PoseDetected, NULL, hPoseDetected);
        CHECK_RC(nRetVal, "Register to Pose Detected");
        g_UserGenerator.GetSkeletonCap().GetCalibrationPose(g_strPose);
    }
    
    g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);
    
    //nRetVal = g_UserGenerator.GetSkeletonCap().RegisterToCalibrationInProgress(MyCalibrationInProgress, NULL, hCalibrationInProgress);
    //CHECK_RC(nRetVal, "Register to calibration in progress");
    
    //nRetVal = g_UserGenerator.GetPoseDetectionCap().RegisterToPoseInProgress(MyPoseInProgress, NULL, hPoseInProgress);
    //CHECK_RC(nRetVal, "Register to pose in progress");
    
    nRetVal = g_Context.StartGeneratingAll();
    CHECK_RC(nRetVal, "StartGenerating");
    
    //テクスチャ設定
    g_DepthGenerator.GetAlternativeViewPointCap().SetViewPoint(g_ImageGenerator);
    g_nTexMapX = (((unsigned short)(depthMD.FullXRes()-1) / 512) + 1) * 512;
    g_nTexMapY = (((unsigned short)(depthMD.FullYRes()-1) / 512) + 1) * 512;
    g_pTexMap = (XnRGB24Pixel*)malloc(g_nTexMapX * g_nTexMapY * sizeof(XnRGB24Pixel));
    
    return 0;
}

//関節点"jointPos[]"取得
int getJoint()
{
    static bool bInitialized = false;
    static GLuint depthTexID;
    static unsigned char* pDepthTexBuf;
    static int texWidth, texHeight;
    
    float topLeftX;
    float topLeftY;
    float bottomRightY;
    float bottomRightX;
    float texXpos;
    float texYpos;
    
    char strLabel[50] = "";
    XnUserID aUsers[15];
    XnUInt16 nUsers = 15;
    g_UserGenerator.GetUsers(aUsers, nUsers);
    
    XnSkeletonJointPosition joint[24];
    
    //if (nUsers>0) {
    for (int i=0; i<nUsers; i++) {
        /*
         XN_SKEL_HEAD
         XN_SKEL_NECK
         XN_SKEL_TORSO
         XN_SKEL_WAIST
         XN_SKEL_LEFT_COLLAR
         XN_SKEL_LEFT_SHOULDER
         XN_SKEL_LEFT_ELBOW
         XN_SKEL_LEFT_WRIST
         XN_SKEL_LEFT_HAND
         XN_SKEL_LEFT_FINGERTIP
         XN_SKEL_RIGHT_COLLAR
         XN_SKEL_RIGHT_SHOULDER
         XN_SKEL_RIGHT_ELBOW
         XN_SKEL_RIGHT_WRIST
         XN_SKEL_RIGHT_HAND
         XN_SKEL_RIGHT_FINGERTIP
         XN_SKEL_LEFT_HIP
         XN_SKEL_LEFT_KNEE
         XN_SKEL_LEFT_ANKLE
         XN_SKEL_LEFT_FOOT
         XN_SKEL_RIGHT_HIP
         XN_SKEL_RIGHT_KNEE
         XN_SKEL_RIGHT_ANKLE
         XN_SKEL_RIGHT_FOOT
         */
        
        g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_HEAD, joint[0]);
        g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_NECK, joint[1]);
        g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_TORSO, joint[2]);
        //g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[0], XN_SKEL_WAIST, joint[2]);  //たぶん取れない
        //g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[0], XN_SKEL_LEFT_COLLAR, joint[3]);   //たぶん取れない
        g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_LEFT_SHOULDER, joint[3]);
        g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_LEFT_ELBOW, joint[4]);
        //g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[0], XN_SKEL_LEFT_WRIST, joint[5]);
        g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_LEFT_HAND, joint[5]);
        //g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[0], XN_SKEL_LEFT_FINGERTIP, joint[6]);
        //g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[0], XN_SKEL_RIGHT_COLLAR, joint[6]);
        g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_RIGHT_SHOULDER, joint[6]);
        g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_RIGHT_ELBOW, joint[7]);
        //g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[0], XN_SKEL_RIGHT_WRIST, joint[11]);
        g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_RIGHT_HAND, joint[8]);
        //g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[0], XN_SKEL_RIGHT_FINGERTIP, joint[10]);
        g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_LEFT_HIP, joint[9]);
        g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_LEFT_KNEE, joint[10]);
        //g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[0], XN_SKEL_LEFT_ANKLE, joint[11]);
        g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_LEFT_FOOT, joint[11]);
        g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_RIGHT_HIP, joint[12]);
        g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_RIGHT_KNEE, joint[13]);
        //g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[0], XN_SKEL_RIGHT_ANKLE, joint[15]);
        g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_RIGHT_FOOT, joint[14]);
        
        //信頼度が低い場合は採用しない
        //        for (int j=0; j<jointNum; j++) {
        //            if (joint[j].fConfidence<0.5)
        //                continue;
        //        }
        
        for (int j=0; j<jointNum; j++) {
            jointPos[i][j] = joint[j].position;
        }
    }
    
    return nUsers;
}

//スキャン点"pointPos[]"と色"pointCol[]"取得
void getDepthImage()
{
    int cnt = 0;
    xnOSMemSet(g_pTexMap, 0, g_nTexMapX*g_nTexMapY*sizeof(XnRGB24Pixel));
    XnRGB24Pixel* pTexRow = g_pTexMap+imageMD.YOffset()*g_nTexMapX;
    const XnRGB24Pixel* pImageRow = imageMD.RGB24Data();
    const XnDepthPixel* pDepthRow = depthMD.Data();
    const XnLabel* pLabelRow = sceneMD.Data();
    
    for (XnUInt y=0; y<imageMD.YRes(); ++y)
    {
        const XnRGB24Pixel* pImage = pImageRow;
        const XnDepthPixel* pDepth = pDepthRow;
        const XnLabel* pLabel = pLabelRow;
        
        for (XnUInt x=0; x<imageMD.XRes(); ++x, ++pImage, ++pDepth, ++pLabel)
        {
            pointCol[cnt] = *pImage;
            pointLabel[cnt] = *pLabel;
            t1[cnt].X = x; t1[cnt].Y = y; t1[cnt].Z = *pDepth;
            cnt++;
        }
        pDepthRow += depthMD.XRes();
        pImageRow += imageMD.XRes();
        pLabelRow += sceneMD.XRes();
        pTexRow += g_nTexMapX;
    }
    g_DepthGenerator.ConvertProjectiveToRealWorld(GL_WIN_TOTAL, t1, pointPos);
}

//終了処理
void CleanupExit()
{
    g_scriptNode.Release();
    g_DepthGenerator.Release();
    g_UserGenerator.Release();
    g_Player.Release();
    g_Context.Release();
    
    exit (1);
}

/*
 void XN_CALLBACK_TYPE MyCalibrationInProgress(SkeletonCapability& capability, XnUserID id, XnCalibrationStatus calibrationError, void* pCookie)
 {
 m_Errors[id].first = calibrationError;
 }
 
 void XN_CALLBACK_TYPE MyPoseInProgress(PoseDetectionCapability& capability, const XnChar* strPose, XnUserID id, XnPoseDetectionStatus poseError, void* pCookie)
 {
 m_Errors[id].second = poseError;
 }
 */

// Callback: New user was detected
void XN_CALLBACK_TYPE User_NewUser(UserGenerator& generator, XnUserID nId, void* pCookie)
{
    XnUInt32 epochTime = 0;
    xnOSGetEpochTime(&epochTime);
    printf("%d New User %d\n", epochTime, nId);
    // New user found
    if (g_bNeedPose)
    {
        g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
    }
    else
    {
        g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
    }
}

// Callback: An existing user was lost
void XN_CALLBACK_TYPE User_LostUser(UserGenerator& generator, XnUserID nId, void* pCookie)
{
    XnUInt32 epochTime = 0;
    xnOSGetEpochTime(&epochTime);
    printf("%d Lost user %d\n", epochTime, nId);
}

// Callback: Detected a pose
void XN_CALLBACK_TYPE UserPose_PoseDetected(PoseDetectionCapability& capability, const XnChar* strPose, XnUserID nId, void* pCookie)
{
    XnUInt32 epochTime = 0;
    xnOSGetEpochTime(&epochTime);
    printf("%d Pose %s detected for user %d\n", epochTime, strPose, nId);
    g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
    g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}

// Callback: Started calibration
void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(SkeletonCapability& capability, XnUserID nId, void* pCookie)
{
    XnUInt32 epochTime = 0;
    xnOSGetEpochTime(&epochTime);
    printf("%d Calibration started for user %d\n", epochTime, nId);
}

// Callback: Finished calibration
void XN_CALLBACK_TYPE UserCalibration_CalibrationComplete(SkeletonCapability& capability, XnUserID nId, XnCalibrationStatus eStatus, void* pCookie)
{
    XnUInt32 epochTime = 0;
    xnOSGetEpochTime(&epochTime);
    if (eStatus == XN_CALIBRATION_STATUS_OK)
    {
        // Calibration succeeded
        printf("%d Calibration complete, start tracking user %d\n", epochTime, nId);
        g_UserGenerator.GetSkeletonCap().StartTracking(nId);
    }
    else
    {
        // Calibration failed
        printf("%d Calibration failed for user %d\n", epochTime, nId);
        if(eStatus==XN_CALIBRATION_STATUS_MANUAL_ABORT)
        {
            printf("Manual abort occured, stop attempting to calibrate!");
            return;
        }
        if (g_bNeedPose)
        {
            g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
        }
        else
        {
            g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
        }
    }
}
//------------------------------------------------------- Kinect関係(終わり) --------------------------------------------------------

