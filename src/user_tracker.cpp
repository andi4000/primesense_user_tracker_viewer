 /*******************************************************************************
*                                                                              *
*   PrimeSense NITE 1.3 - Players Sample                                       *
*   Copyright (C) 2010 PrimeSense Ltd.                                         *
*                                                                              *
*******************************************************************************/

/**
 * //TODO:
 * - test publishing rate, is it synchronized or not
 * - add SIGINT handler with ros::ok()
 * - add info, which key for which function
 * 
 * //DONE:
 * - implement transform publishing (call publishTransform in main loop)
 * 
 * //NOTE:
 * - may not work with opengles
 * 
 * //REFERENCES:
 * - integrating ROS with another wrapper library like glut
 *   http://answers.ros.org/question/11887/significance-of-rosspinonce/
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>

#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>
#include "SceneDrawer.h"

using std::string;

bool g_bDebug = false;

xn::Context g_Context;
xn::ScriptNode g_ScriptNode;
xn::DepthGenerator g_DepthGenerator;
xn::UserGenerator g_UserGenerator;
xn::Recorder* g_pRecorder;

XnUserID g_nPlayer = 0;
XnBool g_bCalibrated = FALSE;

#ifdef USE_GLUT
#if (XN_PLATFORM == XN_PLATFORM_MACOSX)
        #include <GLUT/glut.h>
#else
        #include <GL/freeglut.h>
        #include <GL/glut.h>
#endif
#else
//#include "opengles.h"
#include "kbhit.h"
#endif
//#include "signal_catch.h"

#ifndef USE_GLUT
static EGLDisplay display = EGL_NO_DISPLAY;
static EGLSurface surface = EGL_NO_SURFACE;
static EGLContext context = EGL_NO_CONTEXT;
#endif

#define GL_WIN_SIZE_X 720
#define GL_WIN_SIZE_Y 480
#define START_CAPTURE_CHECK_RC(rc, what)												\
	if (nRetVal != XN_STATUS_OK)														\
{																					\
	ROS_ERROR("Failed to %s: %s\n", what, xnGetStatusString(rc));				\
	StopCapture();															\
	return ;																	\
}

XnBool g_bPause = false;
XnBool g_bRecord = false;

XnBool g_bQuit = false;
void StopCapture()
{
	g_bRecord = false;
	if (g_pRecorder != NULL)
	{
		g_pRecorder->RemoveNodeFromRecording(g_DepthGenerator);
		g_pRecorder->Release();
		delete g_pRecorder;
	}
	g_pRecorder = NULL;
}

void CleanupExit()
{
	if (g_pRecorder)
		g_pRecorder->RemoveNodeFromRecording(g_DepthGenerator);
	StopCapture();

	exit (1);
}

void StartCapture()
{
	char recordFile[256] = {0};
	time_t rawtime;
	struct tm *timeinfo;

	time(&rawtime);
	timeinfo = localtime(&rawtime);
        XnUInt32 size;
        xnOSStrFormat(recordFile, sizeof(recordFile)-1, &size,
                 "%d_%02d_%02d[%02d_%02d_%02d].oni",
                timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday, timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);

	if (g_pRecorder != NULL)
	{
		StopCapture();
	}

	XnStatus nRetVal = XN_STATUS_OK;
	g_pRecorder = new xn::Recorder;

	g_Context.CreateAnyProductionTree(XN_NODE_TYPE_RECORDER, NULL, *g_pRecorder);
	START_CAPTURE_CHECK_RC(nRetVal, "Create recorder");

	nRetVal = g_pRecorder->SetDestination(XN_RECORD_MEDIUM_FILE, recordFile);
	START_CAPTURE_CHECK_RC(nRetVal, "set destination");
	nRetVal = g_pRecorder->AddNodeToRecording(g_DepthGenerator, XN_CODEC_16Z_EMB_TABLES);
	START_CAPTURE_CHECK_RC(nRetVal, "add node");
	g_bRecord = true;
}

XnBool AssignPlayer(XnUserID user)
{
	if (g_nPlayer != 0)
		return FALSE;

	XnPoint3D com;
	g_UserGenerator.GetCoM(user, com);
	if (com.Z == 0)
		return FALSE;

	ROS_INFO("Matching for existing calibration\n");
	g_UserGenerator.GetSkeletonCap().LoadCalibrationData(user, 0);
	g_UserGenerator.GetSkeletonCap().StartTracking(user);
	g_nPlayer = user;
	return TRUE;

}
void XN_CALLBACK_TYPE NewUser(xn::UserGenerator& generator, XnUserID user, void* pCookie)
{
	if (!g_bCalibrated) // check on player0 is enough
	{
		//ROS_INFO("Look for pose\n");
		ROS_INFO("New user %d\n", user);
		g_UserGenerator.GetPoseDetectionCap().StartPoseDetection("Psi", user);
		return;
	}

	AssignPlayer(user);
// 	if (g_nPlayer == 0)
// 	{
// 		ROS_INFO("Assigned user\n");
// 		g_UserGenerator.GetSkeletonCap().LoadCalibrationData(user, 0);
// 		g_UserGenerator.GetSkeletonCap().StartTracking(user);
// 		g_nPlayer = user;
// 	}
}
void FindPlayer()
{
	if (g_nPlayer != 0)
	{
		return;
	}
	XnUserID aUsers[20];
	XnUInt16 nUsers = 20;
	g_UserGenerator.GetUsers(aUsers, nUsers);

	for (int i = 0; i < nUsers; ++i)
	{
		if (AssignPlayer(aUsers[i]))
			return;
	}
}
void LostPlayer()
{
	g_nPlayer = 0;
	FindPlayer();

}
void XN_CALLBACK_TYPE LostUser(xn::UserGenerator& generator, XnUserID user, void* pCookie)
{
	ROS_WARN("Lost user %d\n", user);
	if (g_nPlayer == user)
	{
		LostPlayer();
	}
}
void XN_CALLBACK_TYPE PoseDetected(xn::PoseDetectionCapability& pose, const XnChar* strPose, XnUserID user, void* cxt)
{
	ROS_INFO("Found pose \"%s\" for user %d\n", strPose, user);
	g_UserGenerator.GetSkeletonCap().RequestCalibration(user, TRUE);
	g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(user);
}

void XN_CALLBACK_TYPE CalibrationStarted(xn::SkeletonCapability& skeleton, XnUserID user, void* cxt)
{
	ROS_INFO("Calibration started for user %d\n", user);
}

void XN_CALLBACK_TYPE CalibrationCompleted(xn::SkeletonCapability& skeleton, XnUserID user, XnCalibrationStatus eStatus, void* cxt)
{
	ROS_INFO("Calibration done for user %d %ssuccessfully\n", user, (eStatus == XN_CALIBRATION_STATUS_OK)?"":"un");
	if (eStatus == XN_CALIBRATION_STATUS_OK)
	{
		if (!g_bCalibrated)
		{
			g_UserGenerator.GetSkeletonCap().SaveCalibrationData(user, 0);
			g_nPlayer = user;
			g_UserGenerator.GetSkeletonCap().StartTracking(user);
			g_bCalibrated = TRUE;
		}

		XnUserID aUsers[10];
		XnUInt16 nUsers = 10;
		g_UserGenerator.GetUsers(aUsers, nUsers);
		for (int i = 0; i < nUsers; ++i)
			g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(aUsers[i]);
	}
}

void DrawProjectivePoints(XnPoint3D& ptIn, int width, double r, double g, double b)
{
	static XnFloat pt[3];

	pt[0] = ptIn.X;
	pt[1] = ptIn.Y;
	pt[2] = 0;
	glColor4f(r,
		g,
		b,
		1.0f);
	glPointSize(width);
	glVertexPointer(3, GL_FLOAT, 0, pt);
	glDrawArrays(GL_POINTS, 0, 1);

	glFlush();

}
// this function is called each frame
void glutDisplay (void)
{
	if (g_bDebug) std::cout<<"DEBUG: 1 glut display called"<<std::endl;
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Setup the OpenGL viewpoint
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();

	xn::SceneMetaData sceneMD;
	xn::DepthMetaData depthMD;
	g_DepthGenerator.GetMetaData(depthMD);
	#ifdef USE_GLUT
	glOrtho(0, depthMD.XRes(), depthMD.YRes(), 0, -1.0, 1.0);
	#else
	glOrthof(0, depthMD.XRes(), depthMD.YRes(), 0, -1.0, 1.0);
	#endif

	glDisable(GL_TEXTURE_2D);

	if (!g_bPause)
	{
		if (g_bDebug) std::cout<<"DEBUG: 2a read next available data, bPause = 0"<<std::endl;

		// Read next available data
		g_Context.WaitOneUpdateAll(g_DepthGenerator);
	}
		if (g_bDebug) std::cout<<"DEBUG: 2b process data, bPause = 1"<<std::endl;

		// Process the data
		//DRAW
		g_DepthGenerator.GetMetaData(depthMD);
		g_UserGenerator.GetUserPixels(0, sceneMD);
		DrawDepthMap(depthMD, sceneMD, g_nPlayer);

		if (g_nPlayer != 0)
		{
			XnPoint3D com;
			g_UserGenerator.GetCoM(g_nPlayer, com);
			if (com.Z == 0)
			{
				g_nPlayer = 0;
				FindPlayer();
			}
		}

	#ifdef USE_GLUT
	glutSwapBuffers();
	#endif
}


void publishTransform(XnUserID const& user, XnSkeletonJoint const& joint, string const& frame_id, string const& child_frame_id) {
    static tf::TransformBroadcaster br;

    XnSkeletonJointPosition joint_position;
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, joint, joint_position);
    double x = -joint_position.position.X / 1000.0;
    double y = joint_position.position.Y / 1000.0;
    double z = joint_position.position.Z / 1000.0;

    XnSkeletonJointOrientation joint_orientation;
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, joint, joint_orientation);

    XnFloat* m = joint_orientation.orientation.elements;
    KDL::Rotation rotation(m[0], m[1], m[2],
    					   m[3], m[4], m[5],
    					   m[6], m[7], m[8]);
    double qx, qy, qz, qw;
    rotation.GetQuaternion(qx, qy, qz, qw);

    char child_frame_no[128];
    snprintf(child_frame_no, sizeof(child_frame_no), "%s_%d", child_frame_id.c_str(), user);

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, z));
    transform.setRotation(tf::Quaternion(qx, -qy, -qz, qw));

    // #4994
    tf::Transform change_frame;
    change_frame.setOrigin(tf::Vector3(0, 0, 0));
    tf::Quaternion frame_rotation;
    frame_rotation.setEulerZYX(1.5708, 0, 1.5708);
    change_frame.setRotation(frame_rotation);

    transform = change_frame * transform;

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_no));
}

void publishTransforms(const std::string& frame_id) {
    XnUserID users[15];
    XnUInt16 users_count = 15;
    g_UserGenerator.GetUsers(users, users_count);

    for (int i = 0; i < users_count; ++i) {
        XnUserID user = users[i];
        if (!g_UserGenerator.GetSkeletonCap().IsTracking(user))
            continue;


        publishTransform(user, XN_SKEL_HEAD,           frame_id, "head");
        publishTransform(user, XN_SKEL_NECK,           frame_id, "neck");
        publishTransform(user, XN_SKEL_TORSO,          frame_id, "torso");

        publishTransform(user, XN_SKEL_LEFT_SHOULDER,  frame_id, "left_shoulder");
        publishTransform(user, XN_SKEL_LEFT_ELBOW,     frame_id, "left_elbow");
        publishTransform(user, XN_SKEL_LEFT_HAND,      frame_id, "left_hand");

        publishTransform(user, XN_SKEL_RIGHT_SHOULDER, frame_id, "right_shoulder");
        publishTransform(user, XN_SKEL_RIGHT_ELBOW,    frame_id, "right_elbow");
        publishTransform(user, XN_SKEL_RIGHT_HAND,     frame_id, "right_hand");

        publishTransform(user, XN_SKEL_LEFT_HIP,       frame_id, "left_hip");
        publishTransform(user, XN_SKEL_LEFT_KNEE,      frame_id, "left_knee");
        publishTransform(user, XN_SKEL_LEFT_FOOT,      frame_id, "left_foot");

        publishTransform(user, XN_SKEL_RIGHT_HIP,      frame_id, "right_hip");
        publishTransform(user, XN_SKEL_RIGHT_KNEE,     frame_id, "right_knee");
        publishTransform(user, XN_SKEL_RIGHT_FOOT,     frame_id, "right_foot");
    }
}



//TODO: this only called once
//DONE: this has to be called recursively
void timerCallback(int value){
	if (g_bDebug) std::cout<<"DEBUG: timer callback called"<<std::endl;

	publishTransforms("openni_depth_frame");
	ros::spinOnce();
	
	glutTimerFunc(40, timerCallback, 0);
}

#ifdef USE_GLUT
void glutIdle (void)
{
	if (g_bQuit) {
		CleanupExit();
	}

	// Display the frame
	glutPostRedisplay();
}

void glutKeyboard (unsigned char key, int x, int y)
{
	switch (key)
	{
	case 27:
		CleanupExit();
	case'p':
		g_bPause = !g_bPause;
		break;
	case 'k':
		if (g_pRecorder == NULL)
			StartCapture();
		else
			StopCapture();
		ROS_INFO("Record turned %s\n", g_pRecorder ? "on" : "off");
		break;
	}
}
void glInit (int * pargc, char ** argv)
{
	if (g_bDebug) std::cout<<"DEBUG: glut init"<<std::endl;

	glutInit(pargc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
	glutCreateWindow ("PrimeSense User Tracker Viewer");
	//glutFullScreen();
	glutSetCursor(GLUT_CURSOR_NONE);

	glutKeyboardFunc(glutKeyboard);
	glutDisplayFunc(glutDisplay);
	glutIdleFunc(glutIdle);
	//glutTimerFunc(40, timerCallback, 0);


	glDisable(GL_DEPTH_TEST);
	glEnable(GL_TEXTURE_2D);

	glEnableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);
}
#endif

#define CHECK_RC(rc, what)											\
	if (rc != XN_STATUS_OK)											\
	{																\
		ROS_ERROR("%s failed: %s\n", what, xnGetStatusString(rc));		\
		return rc;													\
	}

#define CHECK_ERRORS(rc, errors, what)		\
	if (rc == XN_STATUS_NO_NODE_PRESENT)	\
{										\
	XnChar strError[1024];				\
	errors.ToString(strError, 1024);	\
	ROS_ERROR("%s\n", strError);			\
	return (rc);						\
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "primesense_user_tracker_viewer");
	ros::NodeHandle nh;
	
	XnStatus rc = XN_STATUS_OK;
	xn::EnumerationErrors errors;

    std::string configFilename = ros::package::getPath("primesense_user_tracker_viewer") + "/Sample-User.xml";

	rc = g_Context.InitFromXmlFile(configFilename.c_str(), g_ScriptNode, &errors);
	CHECK_ERRORS(rc, errors, "InitFromXmlFile");
	CHECK_RC(rc, "InitFromXml");

	rc = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
	CHECK_RC(rc, "Find depth generator");
	rc = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
	CHECK_RC(rc, "Find user generator");

	if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON) ||
		!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION))
	{
		ROS_WARN("User generator doesn't support either skeleton or pose detection.\n");
		return XN_STATUS_ERROR;
	}

	g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

	rc = g_Context.StartGeneratingAll();
	CHECK_RC(rc, "StartGenerating");

	XnCallbackHandle hUserCBs, hCalibrationStartCB, hCalibrationCompleteCB, hPoseCBs;
	g_UserGenerator.RegisterUserCallbacks(NewUser, LostUser, NULL, hUserCBs);
	rc = g_UserGenerator.GetSkeletonCap().RegisterToCalibrationStart(CalibrationStarted, NULL, hCalibrationStartCB);
	CHECK_RC(rc, "Register to calibration start");
	rc = g_UserGenerator.GetSkeletonCap().RegisterToCalibrationComplete(CalibrationCompleted, NULL, hCalibrationCompleteCB);
	CHECK_RC(rc, "Register to calibration complete");
	rc = g_UserGenerator.GetPoseDetectionCap().RegisterToPoseDetected(PoseDetected, NULL, hPoseCBs);
	CHECK_RC(rc, "Register to pose detected");

	// change to 25 for the timer function
	//ros::Rate r(30);
	ros::Rate r(25);

	ros::NodeHandle pnh("~");
	string frame_id("openni_depth_frame");
	pnh.getParam("camera_frame_id", frame_id);
	//TODO: how to use frame_id as timerCallback argument?
	
	#ifdef USE_GLUT
	glInit(&argc, argv);
	
	glutTimerFunc(40, timerCallback, 0);
	glutMainLoop();
	
	
	//TODO: glutMainLoop only called once
	//while (ros::ok()){
		//g_Context.WaitAndUpdateAll();
		//Ref: http://stackoverflow.com/questions/10683925/is-it-possible-to-replace-glutmainloop-with-simple-loop
		//glutMainLoopEvent();
		//publishTransforms(frame_id);
		//r.sleep();
	//}
	

	#else
	if (!opengles_init(GL_WIN_SIZE_X, GL_WIN_SIZE_Y, &display, &surface, &context))
	{
		ROS_ERROR("Error initing opengles\n");
		CleanupExit();
	}

	glDisable(GL_DEPTH_TEST);
//	glEnable(GL_TEXTURE_2D);
	glEnableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);

	while ((!_kbhit()) && (!g_bQuit))
	{
		glutDisplay();
		eglSwapBuffers(display, surface);
	}

	opengles_shutdown(display, surface, context);

	CleanupExit();

	#endif
}
