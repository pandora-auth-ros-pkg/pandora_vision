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
* 		    Despoina Paschalidou
*********************************************************************/

#include "pandora_vision_face/face_detector.h"
using namespace cv;
pthread_mutex_t faceFrameLock = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t faceSeqLock = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t detectFeaturesLock = PTHREAD_MUTEX_INITIALIZER;
//pthread_mutex_t rotDisplayLock = PTHREAD_MUTEX_INITIALIZER;

namespace pandora_vision
{
  /**
   * @brief Class Constructor
   * Initializes cascade_name and constants
   * and allocates memory for sequence of elements
   * @param cascadeName [std::string] the name of 
   *        the cascade to be loaded
   * @param model_path [std::string] the path to the model
   *        to be loaded
   * @param bufferSize [int] number of frames in the frame buffer
   * @param skinEnabled [bool] enables the verification 
   *        of face algorithm using the Skin detector
   * @param scaleFactor [double] scale factor used in cvHaarDetect function
   * @param mn parameter used in cvHaarDetect function
   * @param minFaceDim parameter used in cvHaarDetect function
   * @param skinHist Histogram - parameter to be passed in the Skin Detector
   * @param wallHist Histogram - parameter to be passed in the Skin Detector
   * @param wall2Hist Histogram - parameter to be passed in the Skin Detector
   * @return void
  */

  FaceDetector::FaceDetector(std::string cascadeName,std::string model_path,int bufferSize, bool skinEnabled, double scaleFactor, std::string skinHist, std::string wallHist, std::string wall2Hist )
  {
    cascade_name.assign(cascadeName);
    model_path_name.assign(model_path);
    scale = scaleFactor;
    //~ std::cout<<"cascade_name "<<cascade_name<<std::endl;
    //~ std::cout<<"model_path "<<model_path<<std::endl;
    cascade.load(cascade_name);
    if(cascade.empty())
    {
      fprintf(stderr, "ERROR: Could not load classifier cascade \n");
      exit(0);
    }

    model = createFisherFaceRecognizer();

    model->load(model_path);

    _bufferSize = bufferSize;
    angleNum = 5;
    isSkinDetectorEnabled = skinEnabled;
    isDebugMode = false;

    now = 0;
    prev = 0;

    dstArray = new cv::Mat[angleNum];
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
    
    //!< Erase frame and probability buffers
    if (!frame_buffer.empty())
    {
      frame_buffer.erase (frame_buffer.begin(),frame_buffer.begin()+
        frame_buffer.size());
    }

    if (!probability_buffer.empty()) 
    {
      probability_buffer.erase (probability_buffer.begin(),
          probability_buffer.begin()+probability_buffer.size());
    }

    delete skinDetector;
  }


 /**
    @brief Detects number of faces found in current frame.
      The image buffer contributs to probability.
    @param frameIN [cv::Mat] The frame to be scanned for faces
    @return Integer of the sum of faces found in all
     rotations of the frame 
  */
  int FaceDetector::findFaces(cv::Mat frame)
  {
   
    cv::Mat tmp;
    tmp=cv::Mat::zeros(frame.size().width, frame.size().height , CV_8UC1);

    initFrameProbBuffers(frame);
    createRectangles(tmp);
    
    //!< Clear vector of faces before using it for the current frame
    faces_total.erase (faces_total.begin(),faces_total.begin()+
      faces_total.size());
    
    int facesNum = findFaces1Frame(frame);

    int totalArea = 0;

    if(facesNum)
    {
      totalArea = round( cv::norm(tmp,cv::NORM_L1,cv::noArray())/ 255.);
    }
    
    if(totalArea==0)
    {
      //!< if no face was found, probability for this frame is 0
      probability_buffer[now] = 0.; 
    }
    else
    {
      probability_buffer[now] = round( cv::norm(tmp,cv::NORM_L1,cv::noArray())/ 255.) / (float)totalArea;
    }
    //!< clear value from last scan
    probability = 0.; 

    //!< calculate probability
    for(int i=0 ; i<_bufferSize ; i++)
    {
      probability += (probability_buffer[i]);
    }
    probability = probability / _bufferSize;

    //Compare Probability with Skin Output
    if(isSkinDetectorEnabled )
    {
      std::cout<<"Skin detector enabled"<<std::endl;
      skinDetector->init();

      //when there is a problem with detectSkin() it returns 1
      //in this case findFaces() returns -2
      if ( skinDetector->detectSkin( frame ) )
      {
        return -2;
      }
      compareWithSkinDetector(probability, tmp, totalArea);
    }
    now = (now + 1) % _bufferSize; //prepare index for next frame


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

  bitwise_and( frame_buffer[now] , skinImg , tmp); //tmp now stores common skin-face pixels

  if(totalArea==0)
  {
    skinFaceRatio = 0.; // if no face was found, skinFaceRatio for this frame is 0
  }
  else
  {
    skinFaceRatio = round( cv::norm(tmp,cv::NORM_L1,cv::noArray())/ 255.) / (float)totalArea;
  }

  if (skinFaceRatio >= 0.01) {
    skinFactor = 1.;
  }
  else if(skinFaceRatio >= 0.005 && skinFaceRatio < 0.01) {
    skinFactor = 0.8;
  }
  else if(skinFaceRatio >= 0.001 && skinFaceRatio < 0.005) {
    skinFactor = 0.7;
  }
  else if(skinFaceRatio >= 0.0005 && skinFaceRatio < 0.001) {
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

  

  /**
    @brief Initializes frame and probability buffer
    @param image [cv::Mat] The current frame
    @return void
  */
  void FaceDetector::initFrameProbBuffers(cv::Mat frame)
  {
    if(frame_buffer.empty())
    {
      for(int ii=0; ii<_bufferSize; ii++)
      {
        cv::Mat tmp=Mat::zeros(frame.size().width,frame.size().height,CV_8UC1);
        frame_buffer.push_back(tmp);
      }
    }
    if(probability_buffer.empty())
    {
      for(int ii=0; ii<_bufferSize; ii++) 
      {
        probability_buffer.push_back(0);
      }
    }
  }
  
  /**
    @brief Crate rectangles to current frame according to the positions
      of faces found in previous frames
    @param frameIN [cv::Mat] The frame to be scanned for faces
    @return void
  */
  void FaceDetector::createRectangles(cv::Mat tmp)
  {
    cv::Rect faceRect;
    cv::Point start;
    cv::Point end;
    prev = (now + _bufferSize - 1) % _bufferSize; //index of previous frame
    for(int i = 0; i < ( faces_total.size() ? (faces_total.size()) : 0) ; i++)
    {
      faceRect = faces_total.at(i);
      start =cv::Point( faceRect.x , faceRect.y );
      end = cv::Point( faceRect.x + faceRect.width,faceRect.y + faceRect.height );
      cv::rectangle(tmp, start, end, cv::Scalar(255, 255, 255, 0), CV_FILLED);
    }
  }
  
  /**
    @brief Rotates the given frame in 5 main angles and
      searches for faces in each rotated frame.
    @param frameIN [cv::Mat] The frame to be scanned for faces
    @return 	integer of the sum of faces found in all rotations 
    of the frame.
  */
  int FaceDetector::findFaces1Frame(cv::Mat frameIN)
  {
   
    float angle[] = { 0, 45, 315, 90, 270 };

    int facesNum_total = 0;
    cv::Rect temp(0,0,0,0);
    
    //!< Create number of threads equal to the number of rotating angles
    pthread_t* thr = new pthread_t[angleNum];

    //!< Allocate space for an array of pointers to structs
    ThParams** params = new ThParams*[angleNum];

    //! Create a new struct of parameters for each thread and give values to it
    for(int i=0 ; i<angleNum ; i++)
    {
      params[i] = new ThParams;
      params[i]->frame = frameIN;
      params[i]->angle = angle[i];
      params[i]->scale = scale;
      params[i]->thisObj = this;
      params[i]->retVal = i;
      params[i]->cascade = cascade;
    }

    //!< All threads are created
    for(int i=0 ; i<angleNum ; i++)
    {
      pthread_create( &thr[i], NULL, threadRotateThenDetect , (void*)params[i] );
    }

    //!< All threads join, summing to the total the number of faces found in each one
    for(int i=0 ; i<angleNum ; i++)
    {
      pthread_join(thr[i], NULL);
      facesNum_total += params[i]->retVal;
    }

    for(int i=0; i<angleNum ; i++)
    {
      delete params[i];
    }
    delete [] params;
    delete [] thr;

    //!< Number of Faces is the sum of all faces found in 
    //!< each rotated frame
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

cv::Mat FaceDetector::getFaceNow() {
  return faceNow;
}

cv::Mat FaceDetector::getFacePrev() {
  return facePrev;
}
cv::Mat FaceDetector::getFaceSkin() {
  return skinImg;
}
   
  /**
    @brief Represents one thread. The rotation and face detection are 
      threaded, each tread working in one value of the angle parameter.
    @param void pointer that will be the arguments of the
      thread. A pointer to struct ThParams is used for this reason.
    @return void
  */
  void* FaceDetector::threadRotateThenDetect(void* arg)
  {
   
    int facesNum = 0;
    ThParams* params;
    params = (ThParams*)arg;

    pthread_mutex_lock( &faceFrameLock );
    cv::Mat src(params->frame.size().width , 
      params->frame.size().height,CV_8UC3);
    
    src=params->frame.clone();
    if(!src.data)
    {
      ROS_ERROR("No image data in current thread");
    }
    pthread_mutex_unlock( &faceFrameLock );

    params->dst = params->thisObj->frameRotate(src , params->angle);
    if(!params->dst .data)
    {
      std::cout<<"No image data"<<std::endl;
    }

    //pthread_mutex_lock( &faceSeqLock );
    facesNum = params->thisObj->detectFace(params->dst,params->cascade, params->angle);
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
int FaceDetector::detectFace(cv::Mat img,cv::CascadeClassifier cascade,float angle)
{
 
  cv::Mat original(img.size().width,img.size().height,CV_8UC1);
  original = img.clone();
  cv::Mat gray(img.size().width,img.size().height,CV_8UC1);
  cvtColor(original, gray, CV_BGR2GRAY);
  vector< Rect_<int> > thrfaces;
  int im_width = 92;
  int im_height = 112;

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

  /**
    @brief Rotates input frame according to the given angle
    @param frame [cv::Mat] the frame to be rotated.
    @param thAngle [int] angle in degrees (angle>=0)
      any angle more than 360 degrees is reduced to a primary circle 
      angle.
    @param	rotMatData pointer to the data of the rotation
      matrix values produces for this rotation (this function feels the values)
    @return the frame rotated
  */
  cv::Mat FaceDetector::frameRotate(cv::Mat frame, float thAngle)
  {
    float iImageCenterY = frame.rows / 2;
    float iImageCenterX = frame.cols / 2;
    
    //!< Calculate rotation matrix
    cv::Mat matRotation = getRotationMatrix2D(
      cv::Point( iImageCenterX, iImageCenterY ), (thAngle - 180), 1 );
  
    cv::Mat rotated_frame;
    cv::warpAffine( frame, rotated_frame, matRotation, rotated_frame.size() );

    return rotated_frame;
  }

}
