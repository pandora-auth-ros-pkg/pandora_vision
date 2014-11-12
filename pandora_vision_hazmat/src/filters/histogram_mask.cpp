<<<<<<< HEAD
/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, P.A.N.D.O.R.A. Team.
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
 * Authors: Choutas Vassilis 
 *********************************************************************/


#include "pandora_vision_hazmat/histogram_mask.h"

int HistogramMask::scale_ = 10;
int HistogramMask::channels_[2] = { 0 , 1 };
int HistogramMask::thresh_ = 100;
int HistogramMask::maxValue_ = 255;
float HistogramMask::hueRange_[2] = { 0 , 180};
float HistogramMask::satRange_[2] = { 0 , 255};
float* HistogramMask::ranges_[2]  = {  hueRange_ , satRange_ };


// YUV histogram back projection filter constructor
HistogramMask::HistogramMask() 
{
  // TO DO : READ PARAMETERS FROM XML FILE.

}


/**
 @brief Creates a mask based on the backprojection of an H-S Histogram
        on the input frame.
=======
#include "pandora_vision_hazmat/histogram_mask.h"


// YUV histogram back projection filter constructor
HistogramMask::HistogramMask(HazmatDetector *baseDetector) :
  detector_(baseDetector)
{
  
  }
  
cv::Mat HistogramMask::normImage = cv::Mat();

/**
 @brief Creates a mask based on the histogram ratio of the pattern 
        and the input frame.
>>>>>>> Added new Hazmat detector classes.
 @param frame [const cv::Mat &] : The input frame .
 @param mask [cv::Mat *] : The resulting mask.
 @param keyPoints [const Mat &] : .
**/   
<<<<<<< HEAD
void HistogramMask::createBackProjectionMask(const cv::Mat &frame , 
      cv::Mat *mask , const cv::Mat hist )
{
  cv::Mat hsvFrame;
  cv::Mat backprojection;
  
  if ( !frame.data )
  {
    std::cout << "Invalid Frame received! " << std::endl;
    return;
  }
  
  // Convert the incoming BGR frame to HSV color space
  // to perform the back projection.
  cv::cvtColor( frame , hsvFrame , CV_BGR2HSV );
  
  // Lower the resolution of the frame to speed up processing
  // and create a more dense back projection.
  cv::resize(hsvFrame, hsvFrame, cv::Size( frame.size().width / scale_ ,
     frame.size().height / scale_ ) );
    
  // Calculate the backProjection on the frame.
  cv::calcBackProject( &hsvFrame, 1, channels_ , hist , backprojection , 
    const_cast<const float **> (ranges_) );
  
  
  
  // Resize the resulting mask to the size of the frame so that it 
  // can be used as a mask on the frame.
  cv::resize( backprojection , backprojection , frame.size() );
  
  double t = threshold( backprojection , backprojection , thresh_ , 
    maxValue_ , cv::THRESH_BINARY);
  #ifdef DEBUG
  cv::Mat segmentedFrame;
  frame.copyTo(segmentedFrame , backprojection);
  imshow("Segmented Frame" , segmentedFrame);
  #endif

  // If the mask is not empty :
  // Perform bitwise or between the backprojection and the mask
  if ( mask->data )
    cv::bitwise_or(*mask , backprojection , *mask );
  else
  {
    // Assign the resulting backprojection as the mask.
    *mask = backprojection;
  }
  
  return;
}


=======
void HistogramMask::createMask(const cv::Mat &frame , cv::Mat *mask , 
      const cv::Mat &data  )
{
  
  if ( !frame.data )
  {
    std::cerr << "Invalid frame data " << std::endl;
    mask->data = NULL;
    return ;
  }
  
  cv::Mat frameHist ;
  
  calcNormYUVHist( frame , &frameHist );
  
  if ( !frameHist.data )
  {
    std::cerr << "Invalid frame histogram data " << std::endl;
    mask->data = NULL;
    return ;
  }
  
  if ( !data.data )
  {
    std::cerr << "Invalid extra data " << std::endl;
    // Allocate the mask and make it compatible with the input frame.
    *mask = cv::Mat( frame.size() , CV_8UC1 );
    mask->setTo(255);
    return;
  }
  
  cv::Mat ratioHist;
  
  // Create the ratio histogram . 

  cv::divide( data , frameHist , ratioHist );
  
  
  if ( !ratioHist.data )
  {
    std::cerr << "Invalid Ratio Histogram data " << std::endl;
    mask->data = NULL;
    return ;
  }
  
  static double epsilon = 0.05 ;
  
  cv::Scalar mean =  cv::mean( data )[0]  ;
  double t =   mean[0] ;
  
  cv::Mat yuvFrame;
  
  
  cv::cvtColor(frame,yuvFrame,CV_BGR2YCrCb);
  
  double thresh = cv::threshold( ratioHist , ratioHist , t , 255 
    , cv::THRESH_TOZERO );
  //~ double thresh = cv::threshold(  , ratioHist , t , 255 
    //~ , cv::THRESH_TOZERO );
  //~ cv::normalize( ratioHist, ratioHist, 0, 255, CV_MINMAX);  

  normalizeImage( yuvFrame , 0 );

  int histDims = 2 ;
  
  int UBins = 256 ;
  int VBins = 256 ;
  int histSize[]  = { UBins , VBins };
  int histChannels[] = { 1 , 2 } ;
  
  // U / Cb : blue difference chroma component .

  float blueDiffRange[] = { 0 , 256 } ;
  
  // V / Cr : red difference chroma component .
  
  float redDiffRange[] = { 0 , 256 } ; 
  
  const float *ranges[] = { blueDiffRange , redDiffRange } ;
  
  
  cv::Mat backProj ;
  
  // Calculate the backprojection of the ratio histogram on the
  // normalized image.
  
  

  cv::calcBackProject( &normImage , 1 , histChannels ,  ratioHist , 
    backProj , ranges );
  
  
  
  std::vector<cv::Mat> Ychannels ;
  std::vector<cv::Mat> channels ;
  
  // Split the image to its channels.
  cv::split( yuvFrame , Ychannels );
  //~ cv::split( normImage , channels );
    
  
  cv::multiply( backProj , Ychannels[0] , backProj );
  
  
  *mask = backProj;
  cv::bitwise_or( *mask , backProj , *mask );
  detector_->createMask( frame , mask , data );
  
  //~ backProj.convertTo(backProj,CV_8UC1,255);
  //~ cv::imshow("BackProj",backProj);
  
  Ychannels.clear();
  channels.clear();
  
  return ;
}


// Calculate the normalized YUV histogram.
void HistogramMask::calcNormYUVHist(const cv::Mat &image , 
  cv::Mat *hist)
{
  // YUV destination image.
  cv::Mat yuv;
  
  // Convert the input image to YUV format.
  cv::cvtColor( image , yuv , CV_BGR2YCrCb );
  
  // Normalize the image according to its Y component.
  normalizeImage(yuv , 0 );
  
  int histDims = 2 ;
  
  int UBins = 256 ;
  int VBins = 256 ;
  int histSize[]  = { UBins , VBins };
  const int histChannels[] = { 1 , 2 } ;
  
  // U / Cb : blue difference chroma component .

  
  float blueDiffRange[] = { 0 , 256 } ;
  
  // V / Cr : red difference chroma component .
  
  float redDiffRange[] = { 0 , 256 } ; 
  
  const float *ranges[] = { blueDiffRange , redDiffRange } ;
  

  
  // Calculate the histogram of the normalized YUV image from the 
  // u & v channels.
  cv::calcHist( &normImage , 1 , histChannels , cv::Mat() , *hist , 
    histDims , histSize , ranges ) ;
  
  
  }


// Normalizes an image by its #ch channel.

void HistogramMask::normalizeImage(cv::Mat &image , int ch)
{
  // Check if the image is valid.
  if ( !image.data )
  {
      std::cerr << " The image array is empty! " << std::endl;
      normImage.data =  NULL;
      return ;
  }
  
  // Check if the image has enough channels.
  if ( image.channels() <= 1 )
  {
    std::cerr << "The input image has only one channel! " << std::endl;
    normImage.data =  NULL;
    return;
  }
  
  
  float maxVal ;
  
  // Check the image type and find its maximum possible value.
  switch ( image.type() )
  {
    case CV_8UC3 : 
      maxVal = 255.f ;
      break;
    case CV_16UC3 : 

      maxVal = 65535.f ;
      break;
    case CV_32FC3 : 

      maxVal = 1.f ;
      break;
  }
  
  cv::Mat floatImg ;
  // Convert the input image to a 3-channel float image to prevent
  // information loss due to saturation casts.
  // May change to be made more generic for arbitraty number of 
  // channels.
  image.convertTo( floatImg , CV_32FC3 , 1.f/maxVal );
  
  std::vector<cv::Mat> channels ;
  std::vector<cv::Mat> normChannels( image.channels() ) ;
  
  // Split the image to its channels.
  cv::split( floatImg , channels );
  
  
  for (int i = 0 ; i < floatImg.channels() ; i++ )
  {
    cv::divide( channels[i] , channels[ch] , normChannels[i] );
  }

  // Merge the normalized image channels into a single image.
  cv::merge(normChannels , normImage );
  
  // Return an image that is of the same type as the input image.
  normImage.convertTo( normImage , image.type() , maxVal );
  
  
  
  channels.clear();
  normChannels.clear();
  return;
  }
>>>>>>> Added new Hazmat detector classes.
