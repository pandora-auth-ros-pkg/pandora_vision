/*######################################################################
 * HoleFindNode.cpp
 *
 * File Description :
 *	ROS Node responsible for the administration of image
 *  segmentation and hole detection procedure, created for
 *  use in RoboCup RoboRescue Competition Arena 
 *	
 * Contents :	completeTransition()
 * 				getGeneralParams()
 * 				getHoleParams()
 * 				getTimerParams()
 * 				getTrackerParams()
 * 				holeCallback()
 * 				imageCallback()
 * 				spin()
 * 				startTransition()
 *
 * Author : Michael Skolarikis
 *
 * Date :	24-10-2011
 *
 *######################################################################
 */

#include "pandora_vision_hole/hole_detection.h"

using namespace std;

/**
 * Constructor
 */
HoleFindNode::HoleFindNode() :	_nh() {	
	// Get General Parameters, such as frame width & height
	getGeneralParams();

	//create holeFinder 
	_holeFinder 		=	new HoleFinder();	
	_tracker 			= 	new Tracker();

	string path = packagePath + "/";
	_holeFinder->setTexturePath(path);

	// Get HoleFinder Parameters
	getHoleParams();

	//Get Tracker Parameters
	getTrackerParams();

	//Convert field of view from degrees to rads
	hfov = HFOV * CV_PI / 180;
	vfov = VFOV * CV_PI / 180;

	ratioX = hfov / frameWidth;
	ratioY = vfov / frameHeight;

	//memory will be allocated in the imageCallback
	holeFrame 	= 0;
	extraFrame = cvCreateImage( cvSize(frameWidth,frameHeight), IPL_DEPTH_8U, 3 );

	//Declare publisher and advertise topic where algorithm results are posted
	_holesDirectionPublisher = _nh.advertise<vision_communications::HolesDirectionsVectorMsg>("hole_direction", 10);

	//Advertise topics for debugging if we are in debug mode
	if (debugHole)
	{
		_holeSourcePublisher = image_transport::ImageTransport(_nh).advertise("debug_source", 1);
		_holeEdgePublisher = image_transport::ImageTransport(_nh).advertise("debug_edge", 1);
		_holeThresholdPublisher = image_transport::ImageTransport(_nh).advertise("debug_threshold", 1);
		_holeResultPublisher = image_transport::ImageTransport(_nh).advertise("debug_result", 1);
		_holeTexturePublisher = image_transport::ImageTransport(_nh).advertise("debug_texture", 1);

		_trackerChainPublisher1 = image_transport::ImageTransport(_nh).advertise("debug_trackerChain1", 1);
		_trackerChainPublisher2 = image_transport::ImageTransport(_nh).advertise("debug_trackerChain2", 1);
		_trackerChainPublisher3 = image_transport::ImageTransport(_nh).advertise("debug_trackerChain3", 1);
		_trackerChainPublisher4 = image_transport::ImageTransport(_nh).advertise("debug_trackerChain4", 1);
		_trackerChainPublisher5 = image_transport::ImageTransport(_nh).advertise("debug_trackerChain5", 1);

		_camShiftPublisher = image_transport::ImageTransport(_nh).advertise("debug_camShift", 1);
	}

	//subscribe to input image's topic
	image_transport::ImageTransport it(_nh);
	std::string transport = "theora";
	_frameSubscriber = image_transport::ImageTransport(_nh).subscribe(imageTopic, 1, &HoleFindNode::imageCallback,this /*,transport*/ );

	//initialize states - robot starts in STATE_OFF 
	curState = state_manager_communications::robotModeMsg::MODE_OFF;
	prevState = state_manager_communications::robotModeMsg::MODE_OFF;

	//initialize state Managing Variables
	holeNowON 	= false;
	frameNum = 0;
    
    clientInitialize();
    
	ROS_INFO("[HoleFindNode] : Created HoleFindNode instance");
}

/**
 * Destructor
 */
HoleFindNode::~HoleFindNode()
{
	ROS_INFO("[HoleFindNode] : Destroying HoleFindNode instance");
	delete _holeFinder;
	delete _tracker;
	cvReleaseImage( &extraFrame);
}

/**
 * Get parameters referring to view and frame characteristics
 */
void HoleFindNode::getGeneralParams()
{
	// Get the debugHole parameter if available;
	if (_nh.hasParam("debugHole")) {
		_nh.getParam("debugHole", debugHole);
		ROS_DEBUG_STREAM("debugHole : " << debugHole);
	}
	else {
		ROS_DEBUG("[HoleFindNode] : Parameter debugHole not found. Using Default");
		debugHole = true;
	}

	// Get the holeDummy parameter if available;
	if (_nh.hasParam("holeDummy")) {
		_nh.getParam("holeDummy", holeDummy);
		ROS_DEBUG("holeDummy: %d", holeDummy);
	}
	else {
		ROS_DEBUG("[HoleFindNode] : Parameter holeDummy not found. Using Default");
		holeDummy = false;
	}

	// Get the Height parameter if available;
	if (_nh.hasParam("height")) {
		_nh.getParam("height", frameHeight);
		ROS_DEBUG_STREAM("height : " << frameHeight);
	}
	else {
		ROS_DEBUG("[HoleFindNode] : Parameter frameHeight not found. Using Default");
		frameHeight = DEFAULT_HEIGHT;
	}

	// Get the Width parameter if available;
	if (_nh.hasParam("width")) {
		_nh.getParam("width", frameWidth);
		ROS_DEBUG_STREAM("width : " << frameWidth);
	}
	else {
		ROS_DEBUG("[HoleFindNode] : Parameter frameWidth not found. Using Default");
		frameWidth = DEFAULT_WIDTH;
	}

	// Get the images's topic;
	if (_nh.hasParam("imageTopic")) {
		_nh.getParam("imageTopic", imageTopic);
		ROS_DEBUG_STREAM("imageTopic : " << imageTopic);
	}
	else {
		ROS_DEBUG("[HoleFindNode] : Parameter imageTopic not found. Using Default");
		imageTopic = "/camera_head/image_raw";
	}

	//get package path in the pc
	if (_nh.hasParam("/vision/packagepath")){
		_nh.getParam("/vision/packagepath", packagePath);
		ROS_DEBUG_STREAM("path : " << packagePath);
	}
	else
	{
		ROS_DEBUG("[HoleFindNode] : Parameter path not found. Using Default");
		packagePath = ros::package::getPath("vision");
	}
}

/**
 * Get parameters referring to hole detection algorithm
 */
void HoleFindNode::getHoleParams()
{
	/// Source Filtering Parameters
	{
		// Get the mode of operation for source filtering
		if (_nh.hasParam("modeA")) {
			_nh.getParam("modeA", _holeFinder->_modeA);
			ROS_DEBUG_STREAM("modeA : " << _holeFinder->_modeA);
		}
		else {
			ROS_DEBUG("[webNode] : Parameter modeA not found. Using Default");
			_holeFinder->_modeA = 0;
		}

		// Equalize source image?
		if (_nh.hasParam("equalizeA")) {
			_nh.getParam("equalizeA", _holeFinder->_equalizeA);
			ROS_DEBUG_STREAM("equalizeA : " << _holeFinder->_equalizeA);
		}
		else {
			ROS_DEBUG("[webNode] : Parameter equalizeA not found. Using Default");
			_holeFinder->_equalizeA = 1;
		}

		// Get the mode of operation for source filtering
		if (_nh.hasParam("smoothA")) {
			_nh.getParam("smoothA", _holeFinder->_smoothA);
			ROS_DEBUG_STREAM("smoothA : " << _holeFinder->_smoothA);
		}
		else {
			ROS_DEBUG("[webNode] : Parameter smoothA not found. Using Default");
			_holeFinder->_smoothA = 15;
		}

		// Get erode iterations for source filtering
		if (_nh.hasParam("erodeA")) {
			_nh.getParam("erodeA", _holeFinder->_erodeA);
			ROS_DEBUG_STREAM("erodeA : " << _holeFinder->_erodeA);
		}
		else {
			ROS_DEBUG("[webNode] : Parameter erodeA not found. Using Default");
			_holeFinder->_erodeA = 4;
		}

		// Get dilate iterations for source filtering
		if (_nh.hasParam("dilateA")) {
			_nh.getParam("dilateA", _holeFinder->_dilateA);
			ROS_DEBUG_STREAM("dilateA : " << _holeFinder->_dilateA);
		}
		else {
			ROS_DEBUG("[webNode] : Parameter dilateA not found. Using Default");
			_holeFinder->_dilateA = 2;
		}

		// Get open iterations for source filtering
		if (_nh.hasParam("openA")) {
			_nh.getParam("openA", _holeFinder->_openA);
			ROS_DEBUG_STREAM("openA : " << _holeFinder->_openA);
		}
		else {
			ROS_DEBUG("[WebNode] : Parameter openA not found. Using Default");
			_holeFinder->_openA = 2;
		}

		// Get close iterations for source filtering
		if (_nh.hasParam("closeA")) {
			_nh.getParam("closeA", _holeFinder->_closeA);
			ROS_DEBUG_STREAM("closeA : " << _holeFinder->_closeA);
		}
		else {
			ROS_DEBUG("[webNode] : Parameter closeA not found. Using Default");
			_holeFinder->_closeA = 2;
		}
	}

	/// Edge Detection Parameters , Mode 0
	{
		// Get mode of operation for edge detection
		if (_nh.hasParam("modeB")) {
			_nh.getParam("modeB", _holeFinder->_modeB);
			ROS_DEBUG_STREAM("modeB : " << _holeFinder->_modeB);
		}
		else {
			ROS_DEBUG("[WebNode] : Parameter modeB not found. Using Default");
			_holeFinder->_modeB = 0;
		}

		// Get canny convolution kernel for edge detection
		if (_nh.hasParam("cannyConvKernelB")) {
			_nh.getParam("cannyConvKernelB", _holeFinder->_cannyConvKernelB);
			ROS_DEBUG_STREAM("cannyConvKernelB : " << _holeFinder->_cannyConvKernelB);
		}
		else {
			ROS_DEBUG("[WebNode] : Parameter cannyConvKernelB not found. Using Default");
			_holeFinder->_cannyConvKernelB = 3;
		}

		// Get canny low threshold for edge detection
		if (_nh.hasParam("cannyLowThresB")) {
			_nh.getParam("cannyLowThresB", _holeFinder->_cannyLowThresB);
			ROS_DEBUG_STREAM("cannyLowThresB : " << _holeFinder->_cannyLowThresB);
		}
		else {
			ROS_DEBUG("[WebNode] : Parameter cannyLowThresB not found. Using Default");
			_holeFinder->_cannyLowThresB = 50;
		}

		// Get canny high threshold for edge detection
		if (_nh.hasParam("cannyHighThresB")) {
			_nh.getParam("cannyHighThresB", _holeFinder->_cannyHighThresB);
			ROS_DEBUG_STREAM("cannyHighThresB : " << _holeFinder->_cannyHighThresB);
		}
		else {
			ROS_DEBUG("[WebNode] : Parameter cannyHighThresB not found. Using Default");
			_holeFinder->_cannyHighThresB = 150;
		}

		// Get dilate iterations for edge detection
		if (_nh.hasParam("dilateB")) {
			_nh.getParam("dilateB", _holeFinder->_dilateB);
			ROS_DEBUG_STREAM("dilateB : " << _holeFinder->_dilateB);
		}
		else {
			ROS_DEBUG("[WebNode] : Parameter dilateB not found. Using Default");
			_holeFinder->_dilateB = 2;
		}

		// Get canny convolution kernel for edge detection
		if (_nh.hasParam("cannyConvKernelB1")) {
			_nh.getParam("cannyConvKernelB1", _holeFinder->_cannyConvKernelB1);
			ROS_DEBUG_STREAM("cannyConvKernelB1 : " << _holeFinder->_cannyConvKernelB1);
		}
		else {
			ROS_DEBUG("[WebNode] : Parameter cannyConvKernelB1 not found. Using Default");
			_holeFinder->_cannyConvKernelB1 = 3;
		}

		// Get canny low threshold for edge detection
		if (_nh.hasParam("cannyLowThresB1")) {
			_nh.getParam("cannyLowThresB1", _holeFinder->_cannyLowThresB1);
			ROS_DEBUG_STREAM("cannyLowThresB1 : " << _holeFinder->_cannyLowThresB1);
		}
		else {
			ROS_DEBUG("[WebNode] : Parameter cannyLowThresB1 not found. Using Default");
			_holeFinder->_cannyLowThresB1 = 0;
		}

		// Get canny high threshold for edge detection
		if (_nh.hasParam("cannyHighThresB1")) {
			_nh.getParam("cannyHighThresB1", _holeFinder->_cannyHighThresB1);
			ROS_DEBUG_STREAM("cannyHighThresB1 : " << _holeFinder->_cannyHighThresB1);
		}
		else {
			ROS_DEBUG("[WebNode] : Parameter cannyHighThresB1 not found. Using Default");
			_holeFinder->_cannyHighThresB1 = 0;
		}
	}

	/// Edge Detection Parameters , Mode 1
	{
		// Get gradient iterations for edge detection
		if (_nh.hasParam("gradientB")) {
			_nh.getParam("gradientB", _holeFinder->_gradientB);
			ROS_DEBUG_STREAM("gradientB : " << _holeFinder->_gradientB);
		}
		else {
			ROS_DEBUG("[WebNode] : Parameter gradientB not found. Using Default");
			_holeFinder->_gradientB = 1;
		}

		// Equalize gradient image?
		if (_nh.hasParam("equalizeB")) {
			_nh.getParam("equalizeB", _holeFinder->_equalizeB);
			ROS_DEBUG_STREAM("equalizeB : " << _holeFinder->_equalizeB);
		}
		else {
			ROS_DEBUG("[WebNode] : Parameter equalizeB not found. Using Default");
			_holeFinder->_equalizeB = 0;
		}

		// Get Low Threshold for gradient image
		if (_nh.hasParam("thresholdLowThresB")) {
			_nh.getParam("thresholdLowThresB", _holeFinder->_thresholdLowThresB);
			ROS_DEBUG_STREAM("thresholdLowThresB : " << _holeFinder->_thresholdLowThresB);
		}
		else {
			ROS_DEBUG("[WebNode] : Parameter thresholdLowThresB not found. Using Default");
			_holeFinder->_thresholdLowThresB = 30;
		}

		// Get High Threshold for gradient image
		if (_nh.hasParam("thresholdHighThresB")) {
			_nh.getParam("thresholdHighThresB", _holeFinder->_thresholdHighThresB);
			ROS_DEBUG_STREAM("thresholdHighThresB : " << _holeFinder->_thresholdHighThresB);
		}
		else {
			ROS_DEBUG("[WebNode] : Parameter thresholdHighThresB not found. Using Default");
			_holeFinder->_thresholdHighThresB = 255;
		}

		// Get dilate iterations for edge detection
		if (_nh.hasParam("dilateB1")) {
			_nh.getParam("dilateB1", _holeFinder->_dilateB1);
			ROS_DEBUG_STREAM("dilateB1 : " << _holeFinder->_dilateB1);
		}
		else {
			ROS_DEBUG("[WebNode] : Parameter dilateB1 not found. Using Default");
			_holeFinder->_dilateB1 = 2;
		}

		// Get canny convolution kernel for edge detection
		if (_nh.hasParam("cannyConvKernelB2")) {
			_nh.getParam("cannyConvKernelB2", _holeFinder->_cannyConvKernelB2);
			ROS_DEBUG_STREAM("cannyConvKernelB2 : " << _holeFinder->_cannyConvKernelB2);
		}
		else {
			ROS_DEBUG("[WebNode] : Parameter cannyConvKernelB2 not found. Using Default");
			_holeFinder->_cannyConvKernelB2 = 3;
		}

		// Get canny low threshold for edge detection
		if (_nh.hasParam("cannyLowThresB2")) {
			_nh.getParam("cannyLowThresB2", _holeFinder->_cannyLowThresB2);
			ROS_DEBUG_STREAM("cannyLowThresB2 : " << _holeFinder->_cannyLowThresB2);
		}
		else {
			ROS_DEBUG("[WebNode] : Parameter cannyLowThresB2 not found. Using Default");
			_holeFinder->_cannyLowThresB2 = 0;
		}

		// Get canny high threshold for edge detection
		if (_nh.hasParam("cannyHighThresB2")) {
			_nh.getParam("cannyHighThresB2", _holeFinder->_cannyHighThresB2);
			ROS_DEBUG_STREAM("cannyHighThresB2 : " << _holeFinder->_cannyHighThresB2);
		}
		else {
			ROS_DEBUG("[WebNode] : Parameter cannyHighThresB2 not found. Using Default");
			_holeFinder->_cannyHighThresB2 = 0;
		}
	}

	/// Threshold Image
	{
		// Get erode iterations for thresholded image
		if (_nh.hasParam("erodeC")) {
			_nh.getParam("erodeC", _holeFinder->_erodeC);
			ROS_DEBUG_STREAM("erodeC : " << _holeFinder->_erodeC);
		}
		else {
			ROS_DEBUG("[WebNode] : Parameter erodeC not found. Using Default");
			_holeFinder->_erodeC = 4;
		}

		// Get dilate iterations for thresholded image
		if (_nh.hasParam("dilateC")) {
			_nh.getParam("dilateC", _holeFinder->_dilateC);
			ROS_DEBUG_STREAM("dilateC : " << _holeFinder->_dilateC);
		}
		else {
			ROS_DEBUG("[WebNode] : Parameter dilateC not found. Using Default");
			_holeFinder->_dilateC = 2;
		}

		// Get dilate iterations for thresholded image
		if (_nh.hasParam("openC")) {
			_nh.getParam("openC", _holeFinder->_openC);
			ROS_DEBUG_STREAM("openC : " << _holeFinder->_openC);
		}
		else {
			ROS_DEBUG("[WebNode] : Parameter openC not found. Using Default");
			_holeFinder->_openC = 2;
		}

		// Get close iterations for thresholded image
		if (_nh.hasParam("closeC")) {
			_nh.getParam("closeC", _holeFinder->_closeC);
			ROS_DEBUG_STREAM("closeC : " << _holeFinder->_closeC);
		}
		else {
			ROS_DEBUG("[WebNode] : Parameter closeC not found. Using Default");
			_holeFinder->_closeC = 4;
		}

		// Get Low Threshold to threshold source image
		if (_nh.hasParam("thresholdLowThresC")) {
			_nh.getParam("thresholdLowThresC", _holeFinder->_thresholdLowThresC);
			ROS_DEBUG_STREAM("thresholdLowThresC : " << _holeFinder->_thresholdLowThresC);
		}
		else {
			ROS_DEBUG("[WebNode] : Parameter thresholdLowThresC not found. Using Default");
			_holeFinder->_thresholdLowThresC = 60;
		}

		// Get High Threshold to threshold source image
		if (_nh.hasParam("thresholdHighThresC")) {
			_nh.getParam("thresholdHighThresC", _holeFinder->_thresholdHighThresC);
			ROS_DEBUG_STREAM("thresholdHighThresC : " << _holeFinder->_thresholdHighThresC);
		}
		else {
			ROS_DEBUG("[WebNode] : Parameter thresholdHighThresC not found. Using Default");
			_holeFinder->_thresholdHighThresC = 255;
		}
	}

	/// Texture Image
	{
		// Get erode iterations for textured image
		if (_nh.hasParam("erodeD")) {
			_nh.getParam("erodeD", _holeFinder->_erodeD);
			ROS_DEBUG_STREAM("erodeD : " << _holeFinder->_erodeD);
		}
		else {
			ROS_DEBUG("[WebNode] : Parameter erodeD not found. Using Default");
			_holeFinder->_erodeD = 8;
		}

		// Get dilate iterations for textured image
		if (_nh.hasParam("dilateD")) {
			_nh.getParam("dilateD", _holeFinder->_dilateD);
			ROS_DEBUG_STREAM("dilateD : " << _holeFinder->_dilateD);
		}
		else {
			ROS_DEBUG("[WebNode] : Parameter dilateD not found. Using Default");
			_holeFinder->_dilateD = 10;
		}

		// Get open iterations for textured image
		if (_nh.hasParam("openD")) {
			_nh.getParam("openD", _holeFinder->_openD);
			ROS_DEBUG_STREAM("openD : " << _holeFinder->_openD);
		}
		else {
			ROS_DEBUG("[WebNode] : Parameter openD not found. Using Default");
			_holeFinder->_openD = 6;
		}

		// Get open iterations for textured image
		if (_nh.hasParam("closeD")) {
			_nh.getParam("closeD", _holeFinder->_closeD);
			ROS_DEBUG_STREAM("closeD : " << _holeFinder->_closeD);
		}
		else {
			ROS_DEBUG("[WebNode] : Parameter closeD not found. Using Default");
			_holeFinder->_closeD = 6;
		}

		// Get Low Threshold to threshold texture image
		if (_nh.hasParam("thresholdLowThresD")) {
			_nh.getParam("thresholdLowThresD", _holeFinder->_thresholdLowThresD);
			ROS_DEBUG_STREAM("thresholdLowThresC : " << _holeFinder->_thresholdLowThresD);
		}
		else {
			ROS_DEBUG("[WebNode] : Parameter thresholdLowThresD not found. Using Default");
			_holeFinder->_thresholdLowThresD = 127;
		}

		// Get High Threshold to threshold texture image
		if (_nh.hasParam("thresholdHighThresD")) {
			_nh.getParam("thresholdHighThresD", _holeFinder->_thresholdHighThresD);
			ROS_DEBUG_STREAM("thresholdHighThresD : " << _holeFinder->_thresholdHighThresD);
		}
		else {
			ROS_DEBUG("[WebNode] : Parameter thresholdHighThresD not found. Using Default");
			_holeFinder->_thresholdHighThresC = 255;
		}
	}

	/// Contour Filtering
	{

		// Get Minimum Contour Length
		if (_nh.hasParam("lengthContour")) {
			_nh.getParam("lengthContour", _holeFinder->_lengthContour);
			ROS_DEBUG_STREAM("lengthContour : " << _holeFinder->_lengthContour);
		}
		else {
			ROS_DEBUG("[WebNode] : Parameter lengthContour not found. Using Default");
			_holeFinder->_lengthContour = 50;
		}

		// Get Minimum Contour Area
		if (_nh.hasParam("areaContour")) {
			_nh.getParam("areaContour", _holeFinder->_areaContour);
			ROS_DEBUG_STREAM("areaContour : " << _holeFinder->_areaContour);
		}
		else {
			ROS_DEBUG("[WebNode] : Parameter areaContour not found. Using Default");
			_holeFinder->_areaContour = 100;
		}

		// Get Maximum Contour Area
		if (_nh.hasParam("maxAreaContour")) {
			_nh.getParam("maxAreaContour", _holeFinder->_maxAreaContour);
			ROS_DEBUG_STREAM("maxAreaContour : " << _holeFinder->_maxAreaContour);
		}
		else {
			ROS_DEBUG("[WebNode] : Parameter maxAreaContour not found. Using Default");
			_holeFinder->_maxAreaContour = 0;
		}

		// Get Minimum Height For Bounding Rect
		if (_nh.hasParam("rectHeightContour")) {
			_nh.getParam("rectHeightContour", _holeFinder->_rectHeightContour);
			ROS_DEBUG_STREAM("rectHeightContour : " << _holeFinder->_rectHeightContour);
		}
		else {
			ROS_DEBUG("[WebNode] : Parameter rectHeightContour not found. Using Default");
			_holeFinder->_rectHeightContour = 25;
		}

		// Get Minimum Width For Bounding Rect
		if (_nh.hasParam("rectWidthContour")) {
			_nh.getParam("rectWidthContour", _holeFinder->_rectWidthContour);
			ROS_DEBUG_STREAM("rectWidthContour : " << _holeFinder->_rectWidthContour);
		}
		else {
			ROS_DEBUG("[WebNode] : Parameter rectWidthContour not found. Using Default");
			_holeFinder->_rectWidthContour = 25;
		}

		// Get Maximum Height to Width Ratio
		if (_nh.hasParam("heightToWidthContour")) {
			_nh.getParam("heightToWidthContour", _holeFinder->_heightToWidthContour);
			ROS_DEBUG_STREAM("heightToWidthContour : " << _holeFinder->_heightToWidthContour);
		}
		else {
			ROS_DEBUG("[WebNode] : Parameter heightToWidthContour not found. Using Default");
			_holeFinder->_heightToWidthContour = 3.5;
		}

		// Get Maximum Width to Height Ratio
		if (_nh.hasParam("widthToHeightContour")) {
			_nh.getParam("widthToHeightContour", _holeFinder->_widthToHeightContour);
			ROS_DEBUG_STREAM("widthToHeightContour : " << _holeFinder->_widthToHeightContour);
		}
		else {
			ROS_DEBUG("[WebNode] : Parameter widthToHeightContour not found. Using Default");
			_holeFinder->_widthToHeightContour = 3.5;
		}

		// Get Minimum Area to Bounding Rect Area ratio
		if (_nh.hasParam("pixelToRectAreaContour")) {
			_nh.getParam("pixelToRectAreaContour", _holeFinder->_pixelToRectAreaContour);
			ROS_DEBUG_STREAM("pixelToRectAreaContour : " << _holeFinder->_pixelToRectAreaContour);
		}
		else {
			ROS_DEBUG("[WebNode] : Parameter pixelToRectAreaContour not found. Using Default");
			_holeFinder->_pixelToRectAreaContour = 0.6;
		}

		// Get Minimum Dark Pixels to Area ratio
		if (_nh.hasParam("darkToPixelAreaContour")) {
			_nh.getParam("darkToPixelAreaContour", _holeFinder->_darkToPixelAreaContour);
			ROS_DEBUG_STREAM("darkToPixelAreaContour : " << _holeFinder->_darkToPixelAreaContour);
		}
		else {
			ROS_DEBUG("[WebNode] : Parameter darkToPixelAreaContour not found. Using Default");
			_holeFinder->_darkToPixelAreaContour = 0.7;
		}
	}

	/// Blob Filtering
	{
		// Get Minimum FormFactor
		if (_nh.hasParam("minFormFactor")) {
			_nh.getParam("minFormFactor", _holeFinder->_minFormFactor);
			ROS_DEBUG_STREAM("minFormFactor : " << _holeFinder->_minFormFactor);
		}
		else {
			ROS_DEBUG("[WebNode] : Parameter minFormFactor not found. Using Default");
			_holeFinder->_minFormFactor = 0.75;
		}

		// Get Minimum ellipseFactor
		if (_nh.hasParam("ellipseFactorMin")) {
			_nh.getParam("ellipseFactorMin", _holeFinder->_ellipseFactorMin);
			ROS_DEBUG_STREAM("ellipseFactorMin : " << _holeFinder->_ellipseFactorMin);
		}
		else {
			ROS_DEBUG("[WebNode] : Parameter ellipseFactorMin not found. Using Default");
			_holeFinder->_ellipseFactorMin = 0.8;
		}

		// Get Maximum ellipseFactor
		if (_nh.hasParam("ellipseFactorMax")) {
			_nh.getParam("ellipseFactorMax", _holeFinder->_ellipseFactorMax);
			ROS_DEBUG_STREAM("ellipseFactorMax : " << _holeFinder->_ellipseFactorMax);
		}
		else {
			ROS_DEBUG("[WebNode] : Parameter ellipseFactorMax not found. Using Default");
			_holeFinder->_ellipseFactorMax = 1.3;
		}

		// Get minimum Axis Value
		if (_nh.hasParam("minAxisValue")) {
			_nh.getParam("minAxisValue", _holeFinder->_minAxisValue);
			ROS_DEBUG_STREAM("minAxisValue : " << _holeFinder->_minAxisValue);
		}
		else {
			ROS_DEBUG("[WebNode] : Parameter minAxisValue not found. Using Default");
			_holeFinder->_minAxisValue = 60;
		}		

		// Get vertical Axis Ratio
		if (_nh.hasParam("verticalAxisRatio")) {
			_nh.getParam("verticalAxisRatio", _holeFinder->_verticalAxisRatio);
			ROS_DEBUG_STREAM("verticalAxisRatio : " << _holeFinder->_verticalAxisRatio);
		}
		else {
			ROS_DEBUG("[WebNode] : Parameter verticalAxisRatio not found. Using Default");
			_holeFinder->_verticalAxisRatio = 1.2;
		}

		// Get horizontal Axis Ratio
		if (_nh.hasParam("horizontalAxisRatio")) {
			_nh.getParam("horizontalAxisRatio", _holeFinder->_horizontalAxisRatio);
			ROS_DEBUG_STREAM("horizontalAxisRatio : " << _holeFinder->_horizontalAxisRatio);
		}
		else {
			ROS_DEBUG("[WebNode] : Parameter horizontalAxisRatio not found. Using Default");
			_holeFinder->_horizontalAxisRatio = 3.3;
		}

		// Get thetaHigh 
		if (_nh.hasParam("thetaHigh")) {
			_nh.getParam("thetaHigh", _holeFinder->_thetaHigh);
			ROS_DEBUG_STREAM("thetaHigh : " << _holeFinder->_thetaHigh);
		}
		else {
			ROS_DEBUG("[WebNode] : Parameter thetaHigh not found. Using Default");
			_holeFinder->_thetaHigh = 56;
		}	

		// Get thetaLow 
		if (_nh.hasParam("thetaLow")) {
			_nh.getParam("thetaLow", _holeFinder->_thetaLow);
			ROS_DEBUG_STREAM("thetaHigh : " << _holeFinder->_thetaLow);
		}
		else {
			ROS_DEBUG("[WebNode] : Parameter thetaLow not found. Using Default");
			_holeFinder->_thetaLow = 55;
		}		
	}

}

/**
 * Get parameters referring to tracker algorithm
 */
void HoleFindNode::getTrackerParams(){

	// Get CamShift Tracker HSV Vmin
	if (_nh.hasParam("csVmin")) {
		_nh.getParam("csVmin", _tracker->csVmin);
		ROS_DEBUG_STREAM("csVmin : " << _tracker->csVmin);
	}
	else {
		ROS_DEBUG("[WebNode] : Parameter csVmin not found. Using Default");
		_tracker->csVmin = 10;
	}

	// Get CamShift Tracker HSV Vmax
	if (_nh.hasParam("csVmax")) {
		_nh.getParam("csVmax", _tracker->csVmax);
		ROS_DEBUG_STREAM("csVmax : " << _tracker->csVmax);
	}
	else {
		ROS_DEBUG("[WebNode] : Parameter csVmax not found. Using Default");
		_tracker->csVmax = 70;
	}

	// Get CamShift Tracker HSV Smin
	if (_nh.hasParam("csSmin")) {
		_nh.getParam("csSmin", _tracker->csSmin);
		ROS_DEBUG_STREAM("csSmin : " << _tracker->csSmin);
	}
	else {
		ROS_DEBUG("[WebNode] : Parameter csSmin not found. Using Default");
		_tracker->csSmin = 20;
	}

	// Get CamShift Tracker HSV Smax
	if (_nh.hasParam("csSmax")) {
		_nh.getParam("csSmax", _tracker->csSmax);
		ROS_DEBUG_STREAM("csSmax : " << _tracker->csSmax);
	}
	else {
		ROS_DEBUG("[WebNode] : Parameter csSmax not found. Using Default");
		_tracker->csSmax = 255;
	}

}

/**
 * Called only in case of incoming ROS message
 * @param msg
 */
void HoleFindNode::imageCallback(const sensor_msgs::ImageConstPtr& msg){
	//update image contents
	cv_bridge::CvImagePtr in_msg;
	in_msg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	cv::Mat temp = in_msg->image.clone();
	holeFrame = new IplImage(temp);
	holeFrameTimestamp = msg->header.stamp;
	frameNum++;
	if ( holeFrame == NULL )			
	{               
		ROS_ERROR("[HoleFindNode] : No more Frames");
		ros::shutdown();
		return;
	}
	//if lock was successful: update frame, set boolean and unlock
	memcpy( extraFrame->imageData , holeFrame->imageData , holeFrame->imageSize );
	holeCallback();
}

/**
 * Called as soon as a ROS message becomes available. Detects possible holes.
 */
void HoleFindNode::holeCallback(){
	if(!holeNowON){
		return;
	}

	//ROS_INFO("Holes");

	vision_communications::HolesDirectionsVectorMsg holesVectorMessage;
	vision_communications::HoleDirectionMsg holeMessage;

	ros::Time timestamp = holeFrameTimestamp;

	holesVectorMessage.header.frame_id = "buttomCamera";
	holesVectorMessage.header.stamp = timestamp;

	if (holeDummy)
	{
		// 
		// Dummy Hole Message
		//
		for( int i = 0; i < 3; i++){ 

			holeMessage.yaw = ratioX * ( 300 - frameWidth/2 );
			holeMessage.pitch = -1 * ratioY * ( 200 + frameHeight/2 );
			//holeMessage.area = 1000;

			//holeMessage.header.frame_id="Holes";
			holeMessage.probability = 1;
			//holeMessage.type = vision_communications::victimIdentificationDirectionMsg::HOLE;
			//holeMessage.header.stamp = timestamp;
			holesVectorMessage.holesDirections.push_back(holeMessage);
		}
		_holesDirectionPublisher.publish(holesVectorMessage);

		//dummy delay
		usleep(1000 * 150);
	}
	else
	{ 
		//
		// Hole Message
		//
		cv::Mat local_temp = cv::Mat(extraFrame);
		vector<Thing*> holes = _holeFinder->findHoles(local_temp);
		vector<Thing*> blobs;
		// if possible holes were found, track them
		if (holes.size() > 0)
			_tracker->track( &holes , &blobs , frameNum );
		else
			_tracker->track( NULL , &blobs , frameNum );
		IplImage* camShiftResult = _tracker->camShiftTrack(extraFrame , _holeFinder->imgTexture, &blobs );

#if DEBUG_HOLES ==1
		debug_holes(local_temp, blobs);
#endif

		if( !(curState == state_manager_communications::robotModeMsg::MODE_IDENTIFICATION) ){
			//post Hole Tracker Message
			for( int i = 0; i < blobs.size(); i++)
			{ 
				Thing* current = blobs.at(i);

				//if(current->probability>0.35){
				holeMessage.yaw = ratioX * ( current->center.x - (double)frameWidth/2 );
				holeMessage.pitch = -ratioY * ( current->center.y - (double)frameHeight/2 );
				//holeMessage.area = current->area;
				holeMessage.probability = current->probability;
				holeMessage.holeId = current->m_chainId;
				//holeMessage.type = vision_communications::victimIdentificationDirectionMsg::HOLE;

				holesVectorMessage.holesDirections.push_back(holeMessage);
				//}

			}
			if(blobs.size()>0)_holesDirectionPublisher.publish(holesVectorMessage);

		}
		else
		{
			//vision_communications::victimIdentificationDirectionMsg camShiftMessage;


			for(int i = 0 ; i<_tracker->camShiftBlobs.size(); i++){

				int center_x = _tracker->camShiftBlobs[i].trackRect.x + (int)(_tracker->camShiftBlobs[i].trackRect.width/2);
				int center_y = _tracker->camShiftBlobs[i].trackRect.y + (int)(_tracker->camShiftBlobs[i].trackRect.height/2);

				holeMessage.yaw = ratioX * ( center_x - (double)frameWidth/2 );
				holeMessage.pitch = -ratioY * ( center_y  - (double)frameHeight/2 );
				//holeMessage.area = _tracker->camShiftBlobs[i].area;
				holeMessage.probability = 1;
				holeMessage.holeId = _tracker->camShiftBlobs[i].id;

				ROS_DEBUG("CamshiftBlob id: %d , has probability: %f" , _tracker->camShiftBlobs[i].id , _tracker->camShiftBlobs[i].probability);


				holesVectorMessage.holesDirections.push_back(holeMessage);

			}
			if(_tracker->camShiftBlobs.size()>0)_holesDirectionPublisher.publish(holesVectorMessage);


			if( _tracker->camShiftBlobs.size() == 0 )
			{
				//post Hole Tracker Message
				for( int i = 0; i < blobs.size(); i++)
				{ 
					Thing* current = blobs.at(i);

					holeMessage.yaw = ratioX * ( current->center.x - (double)frameWidth/2 );
					holeMessage.pitch = -ratioY * ( current->center.y - (double)frameHeight/2 );
					//holeMessage.area = current->area;
					holeMessage.probability = current->probability;
					holeMessage.holeId = current->m_chainId;

					//holeMessage.header.frame_id = "headCamera";


					//holeMessage.type = vision_communications::victimIdentificationDirectionMsg::HOLE;
					//holeMessage.header.stamp = timestamp;
					holesVectorMessage.holesDirections.push_back(holeMessage);
				}
				if(blobs.size()>0)_holesDirectionPublisher.publish(holesVectorMessage);
			}
		}

		//if in debug mode, publish images
		if (debugHole)
		{
			//debug_publisher(timestamp, _holeFinder->imgSrc, _holeSourcePublisher, sensor_msgs::image_encodings::MONO8);
			cv_bridge::CvImage src_img_msg;
			src_img_msg.header.stamp   = timestamp;
			src_img_msg.encoding = sensor_msgs::image_encodings::MONO8;
			src_img_msg.image    = _holeFinder->imgSrc;
			_holeSourcePublisher.publish(src_img_msg.toImageMsg());

			debug_publisher(timestamp, _holeFinder->imgEdge, _holeEdgePublisher, sensor_msgs::image_encodings::MONO8);
			debug_publisher(timestamp, _holeFinder->imgThreshold, _holeThresholdPublisher, sensor_msgs::image_encodings::MONO8);
			debug_publisher(timestamp, _holeFinder->imgContours, _holeResultPublisher, sensor_msgs::image_encodings::MONO8);
			debug_publisher(timestamp, _holeFinder->imgTexture, _holeTexturePublisher, sensor_msgs::image_encodings::MONO8);
			// Publish Tracker Chains
			debug_publisher(timestamp, _tracker->imageBuffer[0], _trackerChainPublisher1, sensor_msgs::image_encodings::BGR8);
			debug_publisher(timestamp, _tracker->imageBuffer[1], _trackerChainPublisher2, sensor_msgs::image_encodings::BGR8);
			debug_publisher(timestamp, _tracker->imageBuffer[2], _trackerChainPublisher3, sensor_msgs::image_encodings::BGR8);
			debug_publisher(timestamp, _tracker->imageBuffer[3], _trackerChainPublisher4, sensor_msgs::image_encodings::BGR8);
			debug_publisher(timestamp, _tracker->imageBuffer[4], _trackerChainPublisher5, sensor_msgs::image_encodings::BGR8);
			//Publish Camshift
			debug_publisher(timestamp, camShiftResult, _camShiftPublisher, sensor_msgs::image_encodings::BGR8);
		}
		_holeFinder->cleanup(); 
		holes.clear();
		blobs.clear();
	}
}

/**
 * Debug publisher of dummy message.
 * @param timestamp
 * @param iplImage
 * @param publisher
 * @param image_encoding
 */
void HoleFindNode::debug_publisher(ros::Time timestamp, IplImage* iplImage, image_transport::Publisher publisher, std::string image_encoding){
	cv_bridge::CvImage cv_bridge_image;
	cv_bridge_image.header.stamp   = timestamp;
	cv_bridge_image.encoding = image_encoding;
	cv_bridge_image.image    = iplImage;
	publisher.publish(cv_bridge_image.toImageMsg());
}

/**
 * Generates a series of dummy holes for debugging purposes.
 * @param image
 * @param holes
 */
void HoleFindNode::debug_holes(cv::Mat image, std::vector<Thing*> holes)
{
	for(int i=0;i<holes.size();i++){
		Thing* current = holes.at(i);
		std::stringstream ss;
		ss<<current->probability;
		cv::putText(image,ss.str(),cv::Point(current->center.x,current->center.y),FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,0,255));
	}
	imshow("detected holes", image);
	cv::waitKey(1);
}

/**
 * Commences node's state transition.
 * @param newState
 */
void HoleFindNode::startTransition(int newState)
{

	curState = newState;
	
	//check if hole detection algorithm should be running now
	holeNowON =	( curState == state_manager_communications::robotModeMsg::MODE_EXPLORATION) || 
			( curState == state_manager_communications::robotModeMsg::MODE_IDENTIFICATION ) ||
			( curState == state_manager_communications::robotModeMsg::MODE_TELEOPERATED_LOCOMOTION );

	//shutdown if the robot is switched off
	if (curState == state_manager_communications::robotModeMsg::MODE_TERMINATING){
		ros::shutdown();
		return;
	}

	prevState=curState;

	transitionComplete(curState); //this needs to be called everytime a node finishes transition
}

/**
 * Called as soon as the transition is complete.
 */
void HoleFindNode::completeTransition(void){
	ROS_INFO("[HoleFindNode] : Transition Complete");
}

/**
 * Node's main function.
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char** argv){	
	ros::init(argc,argv,"HoleFindNode");
	HoleFindNode holeFindNode;
	ros::spin();
	return 0;	
}
