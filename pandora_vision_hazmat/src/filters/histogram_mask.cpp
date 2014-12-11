#include "pandora_vision_hazmat/histogram_mask.h"

int HistogramMask::scale_ = 10 ;
int HistogramMask::channels_[2] = {0,1};
int HistogramMask::thresh_ = 100 ;
int HistogramMask::maxValue_ = 255 ;
float HistogramMask::hueRange_[2] = {0,180} ;
float HistogramMask::satRange_[2] = {0,255} ;
float* HistogramMask::ranges_[2]  = {  hueRange_ , satRange_ };


// YUV histogram back projection filter constructor
HistogramMask::HistogramMask() 
{
  // TO DO : READ PARAMETERS FROM XML FILE.

}


/**
 @brief Creates a mask based on the backprojection of an H-S Histogram
        on the input frame.
 @param frame [const cv::Mat &] : The input frame .
 @param mask [cv::Mat *] : The resulting mask.
 @param keyPoints [const Mat &] : .
**/   
void HistogramMask::createBackProjectionMask(const cv::Mat &frame , 
      cv::Mat *mask , const cv::Mat hist )
{
  cv::Mat hsvFrame;
  cv::Mat backprojection ;
  
  if ( !frame.data )
  {
    std::cout << "Invalid Frame received! " << std::endl;
    return ;
  }
  
  // Convert the incoming BGR frame to HSV color space
  // to perform the back projection.
  cv::cvtColor( frame , hsvFrame , CV_BGR2HSV );
  
  // Lower the resolution of the frame to speed up processing
  // and create a more dense back projection.
  cv::resize(hsvFrame, hsvFrame, cv::Size( frame.size().width / scale_ ,
     frame.size().height / scale_ ) ) ;
    
  // Calculate the backProjection on the frame.
  cv::calcBackProject( &hsvFrame, 1, channels_ , hist , backprojection , 
    const_cast<const float **> (ranges_)  );
  
  
  
  // Resize the resulting mask to the size of the frame so that it 
  // can be used as a mask on the frame.
  cv::resize( backprojection , backprojection , frame.size() );
  
  double t = threshold( backprojection , backprojection , thresh_ , 
    maxValue_ , cv::THRESH_BINARY);
  #ifdef DEBUG
  cv::Mat segmentedFrame;
  frame.copyTo(segmentedFrame,backprojection);
  imshow("Segmented Frame",segmentedFrame);
  #endif

  // If the mask is not empty :
  // Perform bitwise or between the backprojection and the mask
  if ( mask->data )
    cv::bitwise_or(*mask , backprojection , *mask );
  else
  {
    // Assign the resulting backprojection as the mask.
    *mask = backprojection ;
  }
  
  return ;
}


