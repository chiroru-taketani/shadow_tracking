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
#include <cv.h>  //OpenCV
#include <cxcore.h>  //OpenCV
#include <highgui.h>  //OpenCV

using namespace xn;


//定数の宣言
#define GL_WIN_SIZE_X 640
#define GL_WIN_SIZE_Y 480
#define GL_WIN_TOTAL 307200
#define MAX_DEPTH 10000
#define XN_CALIBRATION_FILE_NAME "UserCalibration.bin"

#define SAMPLE_XML_PATH "SamplesConfig.xml"

#define CHECK_RC(nRetVal, what)						\
if (nRetVal != XN_STATUS_OK)							\
{																\
printf("%s failed: %s\n", what, xnGetStatusString(nRetVal));\
return nRetVal;											\
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
double fLate = 30;  //フレームレート
int jointNum = 15;  //Kinectで取得する関節の数
XnPoint3D t1[GL_WIN_TOTAL], pointPos[GL_WIN_TOTAL];  //深度情報，座標
XnRGB24Pixel pointCol[GL_WIN_TOTAL];  //色情報
XnPoint3D jointPos[24];  //関節座標
XnLabel pointLabel[GL_WIN_TOTAL];  //人ラベル情報

IplImage *depthImage, *depthDispImage;
CvMemStorage *storage;
CvSeq *contour = NULL;


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
   depthImage = cvCreateImage(cvSize(GL_WIN_SIZE_X, GL_WIN_SIZE_Y), IPL_DEPTH_32F, 1);  //深度値生データ
   depthDispImage = cvCreateImage(cvSize(GL_WIN_SIZE_X, GL_WIN_SIZE_Y), IPL_DEPTH_8U, 1);  //深度値表示用
   //輪郭用
   storage = cvCreateMemStorage(0);
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
	glClearColor(1.0, 1.0, 1.0, 1.0);
	glEnable(GL_DEPTH_TEST);  //Zバッファの有効化
	glEnable(GL_BLEND);
   glEnable(GL_NORMALIZE);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
   glAlphaFunc(GL_GREATER, 0.02);

   //光源
   //glEnable(GL_LIGHTING);  //陰影付けの有効化
	glEnable(GL_LIGHT0);  //光源0の有効化
	
	//光源0の各種パラメータ設定
   GLfloat col[4];  //色指定用配列
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
   //glFrustum(-236.0, 236.0, -52.0, 246.0, 700.0, 5000.0);

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
   //gluLookAt(0.0, -100.0, 0.0, 0.0, -100.0, 5000.0, 0.0, 1.0, 0.0);
   
	GLfloat pos[4];  //座標指定用配列
	GLfloat col[4];  //色指定用配列
	GLfloat val[1];  //値指定用配列

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
   
   //pointPos[]のデータからdepthImage生成
   for (int i=0; i<GL_WIN_TOTAL; i++) {
      int N = i/GL_WIN_SIZE_X;
      int M = GL_WIN_SIZE_X-(i-N*GL_WIN_SIZE_X)-1;
      
      double s = pointPos[i].Z+10.0*N/480.0;
      
      if (s<500 || s>10000) s = 10000.0;

      double s2 = 255-s/10.0;

      if (N<350) {
         cvSetReal2D(depthImage, N, M, s);
         //cvSetReal2D(depthDispImage, N, M, s2);
      }
      else {
         cvSetReal2D(depthImage, N, M, 10000.0);
         //cvSetReal2D(depthDispImage, N, M, 0);
      }
   }
   
//   double nearest = 10000.0;
//   for (int j=0; j<GL_WIN_SIZE_Y; j++) {
//      for (int i=0; i<GL_WIN_SIZE_X; i++) {
//         double s = cvGetReal2D(depthImage, j, i);
//         if (s<nearest) nearest = s;
//      }
//   }
   
   cvThreshold(depthImage, depthDispImage, 4000, 255, CV_THRESH_BINARY_INV);
    cvErode(depthDispImage, depthDispImage, NULL, 1);
    cvDilate(depthDispImage, depthDispImage, NULL, 2);
    //cvErode(depthDispImage, depthDispImage, NULL, 3);
    //cvDilate(depthDispImage, depthDispImage, NULL, 2);

    
   contour = NULL;
   cvFindContours(depthDispImage, storage, &contour, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
   CvPoint *p1, *p2, p0;
   while (contour!=NULL){
       double area = cvContourArea(contour, CV_WHOLE_SEQ);
       if (area>2000)
           cvDrawContours(depthDispImage, contour, cvScalar(255,255,255), cvScalar(255,255,255), 0, -1, 8);
      //p1 = (CvPoint*)cvGetSeqElem(contour, 0);
      //p2 = (CvPoint*)cvGetSeqElem(contour, contour->total/2);
      //p0.x = (p1->x+p2->x)*0.5; p0.y = (p1->y+p2->y)*0.5;
      //cvCircle(depthDispImage, p0, 2, CV_RGB(255,255,255), -1);
      contour=contour->h_next;  //次の輪郭に移動
   }

//   printf("(%d, %d)\n", p0.x, p0.y);
   
//   int ii = p0.y*GL_WIN_SIZE_X+(GL_WIN_SIZE_X-p0.x-1);
//
//   Vec_3D ballPos;
//   ballPos.x = pointPos[ii].X; ballPos.y = pointPos[ii].Y; ballPos.z = pointPos[ii].Z;
   
   cvShowImage("DEPTH", depthDispImage);

	//Kinect取得点表示
	//関節点"jointPos[]"表示
	glColor3d(1.0, 1.0, 0.0);
	for (int i=0; i<jointNum; i++) {
		glPushMatrix();
		glTranslated(jointPos[i].X, jointPos[i].Y, jointPos[i].Z);
		glutSolidSphere(10, 10, 5);
		glPopMatrix();
	}
	
   
	//スキャン点"pointPos[]"点描
	glPointSize(2.0);
	glBegin(GL_POINTS);
	for (int i=0; i<GL_WIN_TOTAL; i++) {
      //if (pointLabel[i]==0) continue;
      
		//int n = i/GL_WIN_SIZE_X;
		//int m = i-n*GL_WIN_SIZE_X;
      glColor4d(pointCol[i].nRed/255.0, pointCol[i].nGreen/255.0, pointCol[i].nBlue/255.0, 1.0);
      //glColor4d(0.0, 0.0, 0.0, 1.0);
      glVertex3f(pointPos[i].X, pointPos[i].Y, pointPos[i].Z);  //空間に点を配置
	}
	glEnd();
   
//   glColor4d(1.0, 0.0, 0.0, 1.0);
//   glPushMatrix();
//   glTranslated(ballPos.x, ballPos.y, ballPos.z+150.0);
//   glutWireSphere(150.0, 36, 18);
//   glPopMatrix();
   
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
		for (int j=0; j<jointNum; j++) {
			if (joint[j].fConfidence<0.5)
				continue;
		}
		
		for (int j=0; j<jointNum; j++) {
			jointPos[j] = joint[j].position;
		}
		
		return 1;
	}
	/*
	 else {
	 return 0;
	 }
	 */
	return 0;
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
