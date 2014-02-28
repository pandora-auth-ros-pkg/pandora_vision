/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, P.A.N.D.O.R.A. Team.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: George Aprilis	
* 		  Despoina Paschalidou
*********************************************************************/

#include "pandora_vision_face/face_detector.h"
using namespace cv;
pthread_mutex_t faceFrameLock = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t faceSeqLock = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t detectFeaturesLock = PTHREAD_MUTEX_INITIALIZER;
//pthread_mutex_t rotDisplayLock = PTHREAD_MUTEX_INITIALIZER;

/**
 * Class Constructor
 * Initializes cascade_name and constants
 * and allocates memory for sequence of elements
 * @param cascadeName the name of the cascade to be loaded
 * @param bufferSize number of frames in the frame buffer
 * @param skinEnabled boolean that enables the verification of face algorithm using the Skin detector
 * @param scaleFactor scale factor used in cvHaarDetect function
 * @param mn parameter used in cvHaarDetect function
 * @param minFaceDim parameter used in cvHaarDetect function
 * @param skinHist Histogram - parameter to be passed in the Skin Detector
 * @param wallHist Histogram - parameter to be passed in the Skin Detector
 * @param wall2Hist Histogram - parameter to be passed in the Skin Detector
 */

FaceDetector::FaceDetector(std::string cascadeName,std::string csv, int bufferSize, bool skinEnabled, double scaleFactor, std::string skinHist, std::string wallHist, std::string wall2Hist )
{	

	cascade_name.assign(cascadeName);
	csv_name.assign(csv);
	scale = scaleFactor;
    //~ std::cout<<"cascade_name "<<cascade_name<<std::endl;
    //~ std::cout<<"csv_name "<<csv_name<<std::endl;
	cascade.load(cascade_name);
	if(cascade.empty())
	{
		fprintf(stderr, "ERROR: Could not load classifier cascade \n");
		exit(0);
	}
			
	try 
    {
      read_csv(csv_name, images, labels);
    } catch (cv::Exception& e) 
    {
        std::cout<< "Error opening file \"" << csv_name<< std::endl;
        exit(1);
    }

	model = createFisherFaceRecognizer();
   
    model->train(images, labels);

	N = bufferSize;
	angleNum = 5;
	isSkinDetectorEnabled = skinEnabled;
	isDebugMode = false;
	
	now = 0;
	prev = 0;

	dstArray = new cv::Mat[angleNum];
	partProb = 0;
	probability = 1.;
	
	
    if (isSkinDetectorEnabled == true)
    {
       skinDetector = new SkinDetector( skinHist, wallHist, wall2Hist ); 
    }
        
	storage_total = cvCreateMemStorage(0);
	cvClearMemStorage(storage_total);

	
}

/**
 * Class Destructor
 * Deallocates all memory used for storing sequences of faces,
 * matrices and images
 */

FaceDetector::~FaceDetector()
{	
	
	cvReleaseMemStorage( &storage_total );
	faces.erase (faces.begin(),faces.begin()+faces.size());

	delete [] dstArray;
	
	if (!buffer.empty())
	{
		 buffer.erase (buffer.begin(),buffer.begin()+buffer.size());
	}
	
	if (partProb != 0){
		delete [] partProb;
	}
	
	delete skinDetector;
	
	std::cout << "Destroying FaceDetector instance" << std::endl;
}



/********************** findFaces **************************
 * Type:		PUBLIC
 * Description: 			  
 * @param		frameIN The frame to be scanned for faces 
 * @return 		integer of the sum of faces found in all  
 * 				rotations of the frame. (-2 in case of
 * 				error in loading skin images)
 **********************************************************/
/*
 * main function calculates number of faces in a frame
 */ 
int FaceDetector::findFaces(cv::Mat frameIN)
{
	cv::Mat tmp;
	tmp=cv::Mat::zeros(frameIN.size().width, frameIN.size().height , CV_8UC1);
	
	initFrameProbBuffers(frameIN);
	createRectangles(tmp);  
      //clear Secuence of faces before using it for the next frame  
    faces_total.erase (faces_total.begin(),faces_total.begin()+faces_total.size()); 
	 
	 
	int facesNum = findFaces1Frame(frameIN);
	
	int totalArea = 0;
	
	if(facesNum) 
	{
		totalArea = round( cv::norm(tmp,cv::NORM_L1,cv::noArray())/ 255.);
	}
	
	 
   	
//	cvShowImage("Previous", buffer[prev]);
//	cvShowImage("Now" , buffer[now]);
//	cvShowImage("Diff" , tmp);
       // cvWaitKey(0);
	
   	if(totalArea==0)
   	{
		partProb[now] = 0.; // if no face was found, probability for this frame is 0
	}
	else
	{
		partProb[now] = round( cv::norm(tmp,cv::NORM_L1,cv::noArray())/ 255.) / (float)totalArea;
	}
	
	probability = 0.; //clear value from last scan
	
	// calculate probability 
	for(int i=0 ; i<N ; i++)
	{
		probability += (partProb[i]);
	}
	probability = probability / N;
	
	//Compare Probability with Skin Output
	if(isSkinDetectorEnabled )
	{
        std::cout<<"Skin detector enabled"<<std::endl;
		skinDetector->init();
		
		//when there is a problem with detectSkin() it returns 1
		//in this case findFaces() returns -2
		if ( skinDetector->detectSkin( frameIN ) )
		{
			return -2;
		}
		compareWithSkinDetector(probability, tmp, totalArea);
	}
	
	
	//debugging
	//~ if(isDebugMode)
	//~ {
		//~ faceNow = buffer[now].clone();
		//~ facePrev = buffer[prev].clone();
	//~ }
	
	now = (now + 1) % N; //prepare index for next frame
	
	
	return facesNum;
}
/*
 * set probability according to skinDetector
 */
void FaceDetector::compareWithSkinDetector(float &probability, cv::Mat tmp, int &totalArea)
{
		float skinFactor = 0.; 
		float skinFaceRatio = 0.;
		int skinPixelNum = 0;
                
		//skinImg = skinDetector->getImgContoursForFace();
         skinImg = skinDetector->imgThresholdFiltered;
                                 
                // INITIAL CODE
		
		skinPixelNum = round( cv::norm(skinImg,cv::NORM_L1,cv::noArray())/ 255.);
//                cvShowImage("blob",skinImg);
//                cvWaitKey(0);

		bitwise_and( buffer[now] , skinImg , tmp); //tmp now stores common skin-face pixels
                
		if(totalArea==0)
                {
                    skinFaceRatio = 0.; // if no face was found, skinFaceRatio for this frame is 0
		}
		else
                {                      
                    skinFaceRatio = round( cv::norm(tmp,cv::NORM_L1,cv::noArray())/ 255.) / (float)totalArea;                      
		}
		
		if (skinFaceRatio >= 0.01){
			skinFactor = 1.;
		}
                else if(skinFaceRatio >= 0.005 && skinFaceRatio < 0.01){
			skinFactor = 0.8;
		}
                else if(skinFaceRatio >= 0.001 && skinFaceRatio < 0.005){
			skinFactor = 0.7;
		}
                else if(skinFaceRatio >= 0.0005 && skinFaceRatio < 0.001){
                    skinFactor = 0.4;
                }
                else
                {
                    skinFactor = 0.;
                }
                
		probability = 0.7 * probability + 0.3 * skinFactor;
		std::cout << "skinFaceRatio: " << skinFaceRatio << std::endl;
                std::cout << "skinFaceFactor: " << skinFactor << std::endl;
		//debugging
		if(isDebugMode)
		{
			skinImg = tmp.clone();
			//cvShowImage("SkinOutput" , tmp);
			//cout << "skinFaceRatio: " << skinFaceRatio << endl;
		}
		
		skinDetector->deallocateMemory();
}

void FaceDetector::createRectangles(cv::Mat tmp)
{
	cv::Rect faceRect;
	cv::Point start;
	cv::Point end;
	prev = (now + N - 1) % N; //index of previous frame
	for(int i = 0; i < ( faces_total.size() ? (faces_total.size()) : 0) ; i++)
	{
		faceRect = faces_total.at(i);
		start =cv::Point( faceRect.x , faceRect.y );
		end = cv::Point( faceRect.x + faceRect.width,faceRect.y + faceRect.height );
		cv::rectangle(tmp, start, end, cv::Scalar(255, 255, 255, 0), CV_FILLED);
   	}
}

void FaceDetector::initFrameProbBuffers(cv::Mat frameIN)
{
	//Initialize frame and probability buffers at first run
	if(buffer.empty())
	{
        for(int i=0 ; i<N ; i++)
        {
			cv::Mat tmp=Mat::zeros(frameIN.size().width,frameIN.size().height,CV_8UC1);
			buffer.push_back(tmp);
		}
	}
	if(partProb == 0)
	{
		partProb = new float[N];
		
		for(int i=0 ; i<N ; i++){
			partProb[i] = 0;
		}
	}
}


/****************** findFaces1Frame ************************
 * Type:		PUBLIC
 * Description: Rotates the given frame in 5 main angles
 * 				then searches for faces in each rotated   
 * 				frame.								  
 * @param		frameIN The frame to be scanned for faces 
 * @return 		integer of the sum of faces found in all  
 * 				rotations of the frame.	
 **********************************************************/
/*
 * returns number of faces in each frame
 */ 

int FaceDetector::findFaces1Frame(cv::Mat frameIN)
{		
	float angle[] = { 0, 45, 315, 90, 270 };
	
	int facesNum_total = 0;
	cv::Rect temp(0,0,0,0);
	//create number of threads equal to the number of rotating angles
	pthread_t* thr = new pthread_t[angleNum];
	
	//allocate space for an array of pointers to structs
	ThParams** params = new ThParams*[angleNum];
	
	//create a new struct of parameters for each thread and give values to it
	for(int i=0 ; i<angleNum ; i++)
	{
		params[i] = new ThParams;
		params[i]->frameIN = frameIN;
		params[i]->angle = angle[i];
		params[i]->scale = scale;
		params[i]->thisObj = this;
		params[i]->retVal = i;
		params[i]->cascade = cascade;
	}
	
	//All threads are created
	for(int i=0 ; i<angleNum ; i++)
	{
		pthread_create( &thr[i], NULL, threadRotateThenDetect , (void*)params[i] );
		//~ std::cout<<"Thread "<<params[i]->retVal <<"created"<<std::endl;
	}
	
	//All threads join, summing to the total the number of faces found in each one
	for(int i=0 ; i<angleNum ; i++)
	{
		pthread_join(thr[i], NULL);
		facesNum_total += params[i]->retVal;
		//~ std::cout<<"Thread "<<params[i]->retVal <<"joined"<<std::endl;
	}
	
	//Be tidy
	for(int i=0; i<angleNum ; i++)
	{
		delete params[i];
	}
	delete [] params;
	delete [] thr;
		
	//The returned Number of Faces is the sum of all faces found in each rotated frame
	return facesNum_total;
}



/********************* getFaceRectTable ************************
 * Type:		PUBLIC
 * Description: Creates the continuous table of faces found that
 * 				contains information for each face in every set
 * 				of 4 values:
 * 					table[i*4]		=	face #i position x center
 * 				  	table[i*4+1]	=	face #i position y center
 * 					table[i*4+2]	=	face #i rectangle width
 * 					table[i*4+3]	=	face #i rectangle height
 * @return 		int[] table of face positions and sizes
 ***************************************************************/

int* FaceDetector::getFaceRectTable()
{
	cv::Rect faceRect;
	int* table = new int[ 4*faces_total.size() ];
	for(int i = 0; i < faces_total.size(); i++)
		{
			faceRect = faces_total.at(i);
			table[i*4]   = round( faceRect.x + faceRect.width*0.5 ); //face center x
            table[i*4+1] = round( faceRect.y + faceRect.height*0.5 ); //face center y
			table[i*4+2] = faceRect.width; //width
			table[i*4+3] = faceRect.height; //height
		
			//Debug:
			//cout << table[i*4] << " ";
			//cout << table[i*4+1] << " ";
			//cout << table[i*4+2] << " ";
			//cout << table[i*4+3] << endl;
		}
		
	return table;
}



/***************** getFaceRectTableSize ********************
 * Type:		PUBLIC
 * Description: Returns the size of the table returned by 
 * 				routine	getFaceRectTable.					   
 * @return 		integer size of table 
 ***********************************************************/

int FaceDetector::getFaceRectTableSize()
{
	return 4*faces_total.size();
}



/******************* getProbability ************************
 * Type:		PUBLIC
 * Description: Returns the probability of the faces de-
 * 				tected in the frame, calculated considering
 * 				the consistency in the last frames and (if
 * 				enabled) the relation with SkinDetector 
 * 				output.
 * @return 		float probability value 
 ***********************************************************/

float FaceDetector::getProbability()
{
	return probability;
}

cv::Mat FaceDetector::getFaceNow(){
	return faceNow;
}

cv::Mat FaceDetector::getFacePrev(){
	return facePrev;
}	
cv::Mat FaceDetector::getFaceSkin(){
	return skinImg;
}

/*************** threadRotateThenDetect *********************
 * Type:		PRIVATE
 * Description: Represents one thread. The rotation and face
 * 				detection are threaded each tread working in
 * 				one value of the angle parameter
 * @param		void pointer that will be the arguments of the
 * 				thread. A pointer to struct ThParams is used
 * 				for this reason. 
 * @return 		void pointer
 ***********************************************************/

void* FaceDetector::threadRotateThenDetect(void* arg)
{
	int facesNum = 0;
	float mat[6];		//the affine transformation matrix
	ThParams* params;
	params = (ThParams*)arg;
	
	
	pthread_mutex_lock( &faceFrameLock );
	cv::Mat src(params->frameIN.size().width , params->frameIN.size().height,CV_8UC3);
	src=params->frameIN.clone();
	if(!src.data)
	{
		std::cout<<"No image data"<<std::endl;
	}
	pthread_mutex_unlock( &faceFrameLock );
	
	params->dst = params->thisObj->frameRotate(src , params->angle , mat);
	if(!params->dst .data)
	{
		std::cout<<"No image data"<<std::endl;
	}
	
	//pthread_mutex_lock( &faceSeqLock );
	facesNum = params->thisObj->detectFace(params->dst,params->cascade, mat, params->angle);
											//1.1 , 3 , CV_HAAR_DO_CANNY_PRUNING , cvSize(40, 40)											
	//pthread_mutex_unlock( &faceSeqLock );
	
	
	params->retVal = facesNum;
	//terminates the calling thread
	pthread_exit(NULL);
}



/********************** detectFace *************************
 * Type:		PRIVATE
 * Description: Called by findfaces()
 * 				calls cvHaarDetectObjects to scan frame for
 * 				faces, and drawFace to create rectangles a-
 * 				round the face/faces found in each frame.						  
 * 
 * detectMultiScale is called!!!face detection happens now!
 */ 
int FaceDetector::detectFace(cv::Mat img,cv::CascadeClassifier	cascade, float* rotMat, float angle)
{
	cv::Mat original(img.size().width,img.size().height,CV_8UC1);
	original = img.clone();
	cv::Mat gray(img.size().width,img.size().height,CV_8UC1);
    cvtColor(original, gray, CV_BGR2GRAY);
	vector< Rect_<int> > thrfaces;
    int im_width = images[0].cols;
    int im_height = images[0].rows;
    if(!cascade.empty())
    {	
	  						
		// Find the faces in the frame:
	   pthread_mutex_lock( &detectFeaturesLock );
	   	
       cascade.detectMultiScale(gray, thrfaces);								
	   for(int i = 0; i < (thrfaces.size() ? thrfaces.size() : 0); i++)
		{
			// Process face by face:
            cv::Rect face_i = thrfaces[i];
            cv::Mat face = gray(face_i);
            cv::Mat face_resized;
            cv::resize(face, face_resized, Size(im_width, im_height), 1.0, 1.0, INTER_CUBIC);
            int prediction = model->predict(face_resized);
            std::cout<<"Prediction"<<prediction<<std::endl;
            rectangle(original, face_i, CV_RGB(0, 255,0), 1);
            cv_bridge::CvImage faceMSg;
            
			faceMSg.encoding  = sensor_msgs::image_encodings::MONO8;
			faceMSg.image = original.clone();
			//~ _facePublisher.publish(  faceMSg.toImageMsg());
			pthread_mutex_lock( &faceSeqLock );
			//add every element created for each frame, to the total amount of faces
			faces_total.push_back (thrfaces.at(i));
			pthread_mutex_unlock( &faceSeqLock );
        }      
	}
	 int res = thrfaces.size();
	 thrfaces.erase (thrfaces.begin(),thrfaces.begin()+thrfaces.size());
	 pthread_mutex_unlock( &detectFeaturesLock );
     return res;
    
}



/*********************** frameRotate *************************
 * Type:		PRIVATE
 * Description: Rotates the input frame in any angle between
 * 				0 and 360 degrees
 * @param		frameIn the frame to be rotated.
 * @param		thangle angle in degrees (angle>=0)
 * 				any angle more than 360 degrees is reduced to
 * 				a primary circle angle.
 * @param		rotMatData pointer to the data of the rotation
 * 				matrix values produces for this rotation (this
 * 				function feels the values)
 * @return 		the frame rotated
 *************************************************************/
/*
 * rotates frame according to the given angle
 */ 
cv::Mat FaceDetector::frameRotate(cv::Mat frameIN, float thAngle, float* rotMatData)
{
	thAngle = (int)thAngle % 360;								//convert angle to primary circle
	float angleRadians = thAngle * ((float)CV_PI / 180.0f);		//convert angle to radians
	
	float cosine = (float)( cos(angleRadians) );				//Precalculate to save time
	float sine = (float)( sin(angleRadians) );
	
	cv::Point2f srcTri[3], dstTri[3];
	cv::Mat rotMat(2, 3, CV_32FC1, rotMatData);
	
	//calculate the values of the 3 corners of the image other than the start.
	int x0 = cvRound( (float)frameIN.size().height * sine );
	int y0 = cvRound( (float)frameIN.size().height * cosine );
	int x1 = cvRound( (float)frameIN.size().width * cosine + (float)frameIN.size().height * sine );
	int y1 = cvRound( (float)frameIN.size().height * cosine - (float)frameIN.size().width * sine );
	int x2 = cvRound( (float)frameIN.size().width * cosine );
	int y2 = cvRound( (float)-frameIN.size().width * sine );

	int minx = std::min(0,std::min(x0, std::min(x1,x2)));
	int miny = std::min(0,std::min(y0, std::min(y1,y2)));
	int maxx = std::max(0,std::max(x0, std::max(x1,x2)));
	int maxy = std::max(0,std::max(y0, std::max(y1,y2)));

	//calculate new dimensions
	int w = maxx - minx;
	int h = maxy - miny;
	
	cv::Mat frameRot = cv::Mat::zeros(w, h, CV_8UC3);

	//3 points of the first image are given, stored in srcTri[]
	srcTri[0].x = 0;
	srcTri[0].y = frameIN.size().height - 1;
	srcTri[1].x = frameIN.size().width - 1;
	srcTri[1].y = frameIN.size().height - 1;
	srcTri[2].x = frameIN.size().width - 1;
	srcTri[2].y = 0;

	//the corresponding 3 points (to the ones in srcTri) are calculated
	//according to their values after the rotation
	dstTri[0].x = x0-minx;
	dstTri[0].y = y0-miny;
	dstTri[1].x = x1-minx;
	dstTri[1].y = y1-miny;
	dstTri[2].x = x2-minx;
	dstTri[2].y = y2-miny;
	//get the rotation matrix
    rotMat = cv::getAffineTransform( srcTri, dstTri);
	//calculate the transformation
	cv::warpAffine( frameIN, frameRot, rotMat, frameRot.size() );		

	return frameRot;
}

void FaceDetector::read_csv(const std::string& filename, std::vector<cv::Mat>& images, std::vector<int>& labels)
 {
    std::ifstream file(filename.c_str(), std::ifstream::in);
    if (!file)
     {
        std::string error_message = "No valid input file was given, please check the given filename.";
        CV_Error(CV_StsBadArg, error_message);
    }
    std::string line, path, classlabel;
    char separator = ';';
    while (getline(file, line))
     {
        std::stringstream liness(line);
        getline(liness, path, separator);
        getline(liness, classlabel);
        if(!path.empty() && !classlabel.empty()) {
            images.push_back(cv::imread(path, 0));
            labels.push_back(atoi(classlabel.c_str()));
        }
    }
}
// int main(int argc, char** argv)
// {
//	 //IplImage* image = cvLoadImage(argv[1]);
//     CvCapture* cap = cvCaptureFromCAM(0);
//     IplImage* image = cvQueryFrame(cap);
//         std::string cascade_name = "/home/alex/Pandora/OpenCV-2.4.3/data/haarcascades/haarcascade_frontalface_default.xml";
//	 int bufferSize = 3;
//         bool skinEnabled = false;
//         double scaleFactor = 2.1124;
//         int mn = 2;
//         int minFaceDim = 5;
//         std::string skinHist = "../../data/histogramms/histogramm_skin.jpg";
//         std::string wallHist = "../../data/histogramms/histogramm_wall.jpg";
//         std::string wall2Hist = "../../data/histogramms/histogramm_wall2.jpg";
//         FaceDetector detector(cascade_name, bufferSize, skinEnabled, scaleFactor, mn, minFaceDim, skinHist, wallHist, wall2Hist);
//         int x = detector.findFaces(image);
//	 std::cout << x << std::endl;
// }
