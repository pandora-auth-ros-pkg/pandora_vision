#ifndef UTILITIES_H
#define UTILITIES_H

/**
  Header file used for initializing useful data structures and 
  including the necessary libraries.
**/

#include <iostream>
#include <string>
#include <vector>
#include <limits>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/imgproc/imgproc.hpp"


/** 
 @enum TrainerType
 @brief Type of keypoint detector used .
 **/

enum TrainerType { SIFT , SURF };


struct Pattern
{
  public : 
  
    // Name of the pattern.
    std::string name ; 
    
    // A vector of 2D points that contains the bounding box
    // and the center of the pattern.
    std::vector<cv::Point2f> boundingBox ;
    
    // Vector of detected keypoints in the pattern.
    std::vector<cv::Point2f> keyPoints ;
    
    // Matrix of image descriptors .
    cv::Mat descriptors;
    
  };
  
#endif
